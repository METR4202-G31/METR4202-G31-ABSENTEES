"""
Frontier-based exploration goal selection.

This module is ROS-node-free: it only needs the message types. Given the
SLAM map (nav_msgs/OccupancyGrid), the robot pose and optionally Nav2's
costmaps, it ranks frontier goals best-first. It does not talk to Nav2.

Pipeline:
  - find frontier cells (free cells bordering unknown space) and group them
    into clusters, dropping clusters too small to be worth visiting;
  - work out how far the robot has to drive to every free cell (a
    shortest-path search over known free space, with walls inflated by the
    robot's clearance), so frontiers behind walls are not mistaken for
    close ones and frontiers the robot cannot reach are dropped;
  - for each cluster pick one goal cell: a reachable free frontier cell
    close to the cluster centroid that is far enough from the robot, safe
    in the costmaps, not blacklisted and not close to a goal already visited;
  - orient the goal towards the unexplored side of the frontier;
  - score clusters (shorter drive and bigger is better) and return them
    best-first.

Optional scoring terms (all off by default, switched on in the YAML config):
  - information gain: how much unknown space actually lies behind a
    frontier, found by flood-filling into the unknown cells next to it.
    A small unmapped speck in the middle of a mapped room is enclosed by
    free cells, so its flood fill stops almost immediately and its gain is
    tiny, while a corridor leading into unexplored space has a large gain.
    Clusters below min_gain are ignored.
  - turn penalty: frontiers that need the robot to turn around cost more
    than frontiers roughly ahead of it, so it finishes the corridor or room
    it is in before going back.

Exploration is finished when no cluster yields a goal. The result records
whether clusters were dropped because of the blacklist, so the caller can
tell "nothing left" from "only blacklisted frontiers left".
"""

from collections import deque
from dataclasses import dataclass, field
import heapq
import math
from typing import Callable, List, Optional, Sequence, Tuple

from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import OccupancyGrid
import numpy as np

FREE_MAX = 10   # occupancy values 0..FREE_MAX are treated as "free"
UNKNOWN = -1
COSTMAP_COST_LIMIT = 75   # Nav2 cost is 0-100; above this is too close to an obstacle
TRAVEL_CELL_SIZE = 0.1    # m; the travel-distance search runs on cells about this big

NEIGHBOURS_8 = [
    (dx, dy)
    for dy in (-1, 0, 1)
    for dx in (-1, 0, 1)
    if not (dx == 0 and dy == 0)
]


def _in_bounds(x: int, y: int, width: int, height: int) -> bool:
    return 0 <= x < width and 0 <= y < height


def _grid_array(grid: OccupancyGrid) -> np.ndarray:
    """Return the grid data as a (height, width) int16 array."""
    return np.asarray(grid.data, dtype=np.int16).reshape(
        grid.info.height, grid.info.width
    )


def _frontier_mask(data: np.ndarray) -> np.ndarray:
    """Return a boolean mask of free cells that touch an unknown cell."""
    height, width = data.shape
    free = (data >= 0) & (data <= FREE_MAX)
    padded = np.pad(data == UNKNOWN, 1, constant_values=False)
    near_unknown = np.zeros_like(free)
    for dx, dy in NEIGHBOURS_8:
        near_unknown |= padded[1 + dy:1 + dy + height, 1 + dx:1 + dx + width]
    return free & near_unknown


def _dilate(mask: np.ndarray, radius: int) -> np.ndarray:
    """Grow the True cells of mask by a disc of radius cells."""
    if radius <= 0:
        return mask.copy()
    height, width = mask.shape
    padded = np.pad(mask, radius, constant_values=False)
    out = mask.copy()
    for dy in range(-radius, radius + 1):
        for dx in range(-radius, radius + 1):
            if dx * dx + dy * dy <= radius * radius:
                out |= padded[
                    radius + dy:radius + dy + height,
                    radius + dx:radius + dx + width,
                ]
    return out


def _yaw_of(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def _wrap_angle(angle: float) -> float:
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


NEIGHBOURS_4 = [(1, 0), (-1, 0), (0, 1), (0, -1)]


def information_gain(data: np.ndarray, cluster, max_depth: int, max_cells: int) -> int:
    """
    Count the unknown cells "behind" a frontier cluster.

    Flood-fills (breadth-first search) through UNKNOWN cells only, starting
    from the unknown cells that touch the cluster, and stops after
    max_depth steps (roughly the sensor range in cells) or once max_cells
    cells have been counted. Returns the number of unknown cells reached.

    4-connected moves are used so the fill cannot squeeze diagonally
    between two occupied cells of a wall.
    """
    height, width = data.shape
    seen = set()
    queue = deque()
    for cx, cy in cluster:
        for dx, dy in NEIGHBOURS_4:
            nx, ny = cx + dx, cy + dy
            if (_in_bounds(nx, ny, width, height) and data[ny, nx] == UNKNOWN
                    and (nx, ny) not in seen):
                seen.add((nx, ny))
                queue.append((nx, ny, 0))
    while queue and len(seen) < max_cells:
        x, y, depth = queue.popleft()
        if depth >= max_depth:
            continue
        for dx, dy in NEIGHBOURS_4:
            nx, ny = x + dx, y + dy
            if (_in_bounds(nx, ny, width, height) and data[ny, nx] == UNKNOWN
                    and (nx, ny) not in seen):
                seen.add((nx, ny))
                queue.append((nx, ny, depth + 1))
    return min(len(seen), max_cells)


def cell_to_world(grid: OccupancyGrid, x: float, y: float) -> Tuple[float, float]:
    """Convert grid cell coordinates (may be fractional) to map-frame metres."""
    info = grid.info
    local_x = (x + 0.5) * info.resolution
    local_y = (y + 0.5) * info.resolution
    yaw = _yaw_of(info.origin.orientation)
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    world_x = info.origin.position.x + cos_yaw * local_x - sin_yaw * local_y
    world_y = info.origin.position.y + sin_yaw * local_x + cos_yaw * local_y
    return world_x, world_y


def yaw_to_quaternion(yaw: float) -> Tuple[float, float, float, float]:
    """Return (x, y, z, w) for a rotation about Z only."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def world_to_cell(grid: OccupancyGrid, x: float, y: float) -> Tuple[int, int]:
    """Convert map-frame metres to the (x, y) grid cell (map yaw ignored)."""
    info = grid.info
    return (
        math.floor((x - info.origin.position.x) / info.resolution),
        math.floor((y - info.origin.position.y) / info.resolution),
    )


class TravelMap:
    """
    Driving distance from the robot to every known free cell.

    Occupied cells are inflated by the clearance, unknown cells are never
    entered, and the search runs on cells about TRAVEL_CELL_SIZE wide to
    keep it fast in pure Python. Distances are approximate (8-connected
    grid paths), which is enough to rank frontiers.
    """

    def __init__(self, data: np.ndarray, resolution: float, robot_cell, clearance: float):
        height, width = data.shape
        free = (data >= 0) & (data <= FREE_MAX)
        blocked = _dilate(data > FREE_MAX, int(round(clearance / resolution)))
        passable = free & ~blocked

        # The robot may stand inside the inflated band (e.g. next to a wall):
        # let it drive out of the free cells around it.
        rx, ry = robot_cell
        reach = int(round(clearance / resolution)) + 1
        y0, y1 = max(ry - reach, 0), min(ry + reach + 1, height)
        x0, x1 = max(rx - reach, 0), min(rx + reach + 1, width)
        passable[y0:y1, x0:x1] |= free[y0:y1, x0:x1]

        f = max(1, int(round(TRAVEL_CELL_SIZE / resolution)))
        rows, cols = -(-height // f), -(-width // f)
        padded = np.zeros((rows * f, cols * f), dtype=bool)
        padded[:height, :width] = passable
        coarse = padded.reshape(rows, f, cols, f).any(axis=(1, 3))

        self._factor = f
        self._cols = cols
        sx, sy = rx // f, ry // f
        # valid is False when the robot is not on known free space, so no
        # distance can be trusted; callers should fall back to straight lines.
        self.valid = 0 <= sx < cols and 0 <= sy < rows and bool(coarse[sy, sx])
        self._dist = self._search(coarse, sx, sy, resolution * f) if self.valid else []

    @staticmethod
    def _search(passable: np.ndarray, sx: int, sy: int, step: float) -> List[float]:
        rows, cols = passable.shape
        size = rows * cols
        dist = [math.inf] * size
        open_cells = passable.ravel().tolist()
        diagonal = step * math.sqrt(2.0)
        moves = [
            (dy * cols + dx, dx, diagonal if dx and dy else step)
            for dx, dy in NEIGHBOURS_8
        ]
        start = sy * cols + sx
        dist[start] = 0.0
        heap = [(0.0, start)]
        while heap:
            d, i = heapq.heappop(heap)
            if d > dist[i]:
                continue
            x = i % cols
            for offset, dx, cost in moves:
                nx = x + dx
                j = i + offset
                if nx < 0 or nx >= cols or j < 0 or j >= size or not open_cells[j]:
                    continue
                nd = d + cost
                if nd < dist[j]:
                    dist[j] = nd
                    heapq.heappush(heap, (nd, j))
        return dist

    def distance(self, cell_x: int, cell_y: int) -> float:
        """Return how far the robot drives (m) to a grid cell; inf if unreachable."""
        f = self._factor
        index = (cell_y // f) * self._cols + cell_x // f
        if cell_x < 0 or cell_y < 0 or index >= len(self._dist):
            return math.inf
        return self._dist[index]


def frontier_near(grid: OccupancyGrid, x: float, y: float, radius: float) -> bool:
    """Return True if any frontier cell lies within radius metres of (x, y)."""
    info = grid.info
    cx = int((x - info.origin.position.x) / info.resolution)
    cy = int((y - info.origin.position.y) / info.resolution)
    reach = int(math.ceil(radius / info.resolution))
    x0, x1 = max(cx - reach, 0), min(cx + reach + 1, info.width)
    y0, y1 = max(cy - reach, 0), min(cy + reach + 1, info.height)
    if x0 >= x1 or y0 >= y1:
        return False
    mask = _frontier_mask(_grid_array(grid))[y0:y1, x0:x1]
    for row, col in np.argwhere(mask):
        wx, wy = cell_to_world(grid, x0 + col, y0 + row)
        if math.hypot(wx - x, wy - y) <= radius:
            return True
    return False


@dataclass
class CostmapLayer:
    """
    A Nav2 costmap plus how to express a map-frame point in its frame.

    map_to_costmap is None when the costmap is already in the map frame.
    strict_bounds means a point outside the costmap is unsafe; a rolling
    local costmap only covers the area around the robot, so it uses False.
    """

    grid: OccupancyGrid
    map_to_costmap: Optional[Callable[[float, float], Tuple[float, float]]] = None
    strict_bounds: bool = True

    def is_safe(self, x: float, y: float, cost_limit: int) -> bool:
        """Return True if the map-frame point is not too close to an obstacle."""
        if self.map_to_costmap is not None:
            x, y = self.map_to_costmap(x, y)
        info = self.grid.info
        cell_x = math.floor((x - info.origin.position.x) / info.resolution)
        cell_y = math.floor((y - info.origin.position.y) / info.resolution)
        if not _in_bounds(cell_x, cell_y, info.width, info.height):
            return not self.strict_bounds
        cost = self.grid.data[cell_y * info.width + cell_x]
        return 0 <= cost <= cost_limit


@dataclass
class FrontierCandidate:
    """One frontier goal: where to go, how good it is, and how big it is."""

    pose: PoseStamped
    score: float
    cluster_size: int
    gain: float = 0.0   # m^2 of unknown space behind the frontier (0 if unused)


@dataclass
class SelectionResult:
    """Candidates best-first plus counts explaining why some were dropped."""

    candidates: List[FrontierCandidate] = field(default_factory=list)
    clusters: List[List[Tuple[int, int]]] = field(default_factory=list)
    blocked_clusters: int = 0
    unreachable_clusters: int = 0
    low_gain_clusters: int = 0


class FrontierSelector:
    """Rank frontier goals from a SLAM map."""

    def __init__(
        self,
        min_cluster_size: int = 20,
        min_goal_distance: float = 0.6,
        revisit_radius: float = 0.5,
        max_history: int = 20,
        distance_weight: float = 1.0,
        size_weight: float = 0.1,
        cost_limit: int = COSTMAP_COST_LIMIT,
        use_travel_distance: bool = True,
        travel_clearance: float = 0.15,
        gain_weight: float = 0.0,
        min_gain: float = 0.0,
        gain_radius: float = 3.0,
        max_gain_area: float = 12.0,
        turn_weight: float = 0.0,
    ):
        self.min_cluster_size = min_cluster_size
        self.use_travel_distance = use_travel_distance
        self.travel_clearance = travel_clearance
        self.min_goal_distance = min_goal_distance
        self.revisit_radius = revisit_radius
        self.distance_weight = distance_weight
        self.size_weight = size_weight
        self.cost_limit = cost_limit
        self.gain_weight = gain_weight      # score per m^2 of unknown behind a frontier
        self.min_gain = min_gain            # m^2; frontiers hiding less are ignored
        self.gain_radius = gain_radius      # m; how far the flood fill reaches
        self.max_gain_area = max_gain_area  # m^2; cap so one frontier cannot dominate
        self.turn_weight = turn_weight      # score per radian the robot must turn
        self._history: deque = deque(maxlen=max_history)

    # ---- history: avoiding goals that were already visited -------------

    def mark_visited(self, x: float, y: float) -> None:
        """Record a goal (map-frame metres) as attempted, whatever its outcome."""
        self._history.append((x, y))

    def clear_history(self) -> None:
        """Forget all visited goals."""
        self._history.clear()

    def _too_close_to_history(self, x: float, y: float) -> bool:
        return any(
            math.hypot(x - hx, y - hy) < self.revisit_radius
            for hx, hy in self._history
        )

    # ---- frontier detection ---------------------------------------------

    def find_clusters(self, grid: OccupancyGrid) -> List[List[Tuple[int, int]]]:
        """Group frontier cells into clusters of (x, y) cells, dropping small ones."""
        mask = _frontier_mask(_grid_array(grid))
        frontier_cells = {(int(x), int(y)) for y, x in np.argwhere(mask)}

        clusters = []
        visited = set()
        for start in frontier_cells:
            if start in visited:
                continue
            queue = deque([start])
            visited.add(start)
            cluster = []
            while queue:
                cx, cy = queue.popleft()
                cluster.append((cx, cy))
                for dx, dy in NEIGHBOURS_8:
                    neighbour = (cx + dx, cy + dy)
                    if neighbour in frontier_cells and neighbour not in visited:
                        visited.add(neighbour)
                        queue.append(neighbour)
            clusters.append(cluster)

        # Tiny clusters are usually noise from a stray unknown cell.
        return [c for c in clusters if len(c) >= self.min_cluster_size]

    # ---- goal generation ------------------------------------------------

    @staticmethod
    def _cluster_centroid(cluster) -> Tuple[float, float]:
        cx = sum(c[0] for c in cluster) / len(cluster)
        cy = sum(c[1] for c in cluster) / len(cluster)
        return cx, cy

    @staticmethod
    def _frontier_facing_yaw(data: np.ndarray, cluster) -> float:
        """Average direction from frontier cells towards their unknown neighbours."""
        height, width = data.shape
        dx_sum = 0.0
        dy_sum = 0.0
        count = 0
        for cx, cy in cluster:
            for dx, dy in NEIGHBOURS_8:
                nx, ny = cx + dx, cy + dy
                if _in_bounds(nx, ny, width, height) and data[ny, nx] == UNKNOWN:
                    dx_sum += dx
                    dy_sum += dy
                    count += 1
        if count == 0:
            return 0.0
        return math.atan2(dy_sum, dx_sum)

    def _score(
        self, dist: float, cluster_size: int, gain: float = 0.0, turn: float = 0.0
    ) -> float:
        """
        Return the cost of a goal (lower is better).

        dist  : driving distance (m)              -> shorter is better
        size  : frontier cells in the cluster     -> bigger is better
        gain  : m^2 of unknown space behind it    -> more is better
        turn  : radians the robot must turn to it -> less is better
        """
        return (
            self.distance_weight * dist
            - self.size_weight * cluster_size
            - self.gain_weight * gain
            + self.turn_weight * turn
        )

    def _pick_goal_cell(
        self,
        grid: OccupancyGrid,
        cluster,
        robot_x: float,
        robot_y: float,
        costmaps: Sequence[CostmapLayer],
        is_blocked: Optional[Callable[[float, float], bool]],
        travel: Optional[TravelMap],
    ) -> Tuple[Optional[Tuple[float, float, float]], bool, bool]:
        """
        Pick the goal cell for a cluster.

        Returns ((x, y, driving distance) or None, blocked, unreachable):
        blocked says a cell was rejected because of the blacklist,
        unreachable that a cell was rejected because no drive reaches it.
        """
        cen_x, cen_y = self._cluster_centroid(cluster)
        blocked = False
        unreachable = False
        for cell in sorted(
            cluster, key=lambda c: math.hypot(c[0] - cen_x, c[1] - cen_y)
        ):
            x, y = cell_to_world(grid, cell[0], cell[1])
            straight = math.hypot(x - robot_x, y - robot_y)
            if straight < self.min_goal_distance:
                continue
            if is_blocked is not None and is_blocked(x, y):
                blocked = True
                continue
            if self._too_close_to_history(x, y):
                continue
            dist = straight
            if travel is not None:
                dist = travel.distance(cell[0], cell[1])
                if math.isinf(dist):
                    unreachable = True
                    continue
            if not all(m.is_safe(x, y, self.cost_limit) for m in costmaps):
                continue
            return (x, y, dist), blocked, unreachable
        return None, blocked, unreachable

    def rank_candidates(
        self,
        grid: OccupancyGrid,
        robot_pose: Pose,
        costmaps: Sequence[CostmapLayer] = (),
        is_blocked: Optional[Callable[[float, float], bool]] = None,
        frame_id: str = "map",
    ) -> SelectionResult:
        """Return frontier goals best-first (empty when exploration is complete)."""
        robot_x = robot_pose.position.x
        robot_y = robot_pose.position.y
        robot_yaw = _yaw_of(robot_pose.orientation)
        use_gain = self.gain_weight > 0.0 or self.min_gain > 0.0
        cell_area = grid.info.resolution ** 2
        gain_depth = int(round(self.gain_radius / grid.info.resolution))
        gain_cap = int(self.max_gain_area / cell_area)

        clusters = self.find_clusters(grid)
        result = SelectionResult(clusters=clusters)
        data = _grid_array(grid)

        travel = None
        if self.use_travel_distance and clusters:
            robot_cell = world_to_cell(grid, robot_x, robot_y)
            if _in_bounds(*robot_cell, grid.info.width, grid.info.height):
                travel = TravelMap(
                    data, grid.info.resolution, robot_cell, self.travel_clearance
                )
                if not travel.valid:
                    travel = None
            # Robot off the map or not on known free space (e.g. before SLAM
            # catches up): fall back to straight-line distances rather than
            # calling every frontier unreachable.

        for cluster in clusters:
            gain = 0.0
            if use_gain:
                gain = information_gain(data, cluster, gain_depth, gain_cap) * cell_area
                if gain < self.min_gain:
                    # e.g. an unmapped speck inside a room: nothing behind it.
                    result.low_gain_clusters += 1
                    continue

            picked, blocked, unreachable = self._pick_goal_cell(
                grid, cluster, robot_x, robot_y, costmaps, is_blocked, travel
            )
            if picked is None:
                if blocked:
                    result.blocked_clusters += 1
                elif unreachable:
                    result.unreachable_clusters += 1
                continue
            goal_x, goal_y, dist = picked

            pose = PoseStamped()
            pose.header.frame_id = frame_id
            pose.pose.position.x = goal_x
            pose.pose.position.y = goal_y
            yaw = self._frontier_facing_yaw(data, cluster)
            qx, qy, qz, qw = yaw_to_quaternion(yaw)
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw

            # How far the robot must turn to face the goal (0 = dead ahead,
            # pi = directly behind). Straight-line bearing is good enough here.
            bearing = math.atan2(goal_y - robot_y, goal_x - robot_x)
            turn = abs(_wrap_angle(bearing - robot_yaw))

            result.candidates.append(
                FrontierCandidate(
                    pose=pose,
                    score=self._score(dist, len(cluster), gain, turn),
                    cluster_size=len(cluster),
                    gain=gain,
                )
            )

        result.candidates.sort(key=lambda c: c.score)
        return result
