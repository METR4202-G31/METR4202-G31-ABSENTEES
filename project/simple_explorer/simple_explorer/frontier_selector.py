"""
Frontier-based exploration goal selection.

This module is ROS-node-free: it only needs the message types. Given the
SLAM map (nav_msgs/OccupancyGrid), the robot pose and optionally Nav2's
costmaps, it ranks frontier goals best-first. It does not talk to Nav2.

Pipeline:
  - find frontier cells (free cells bordering unknown space) and group them
    into clusters, dropping clusters too small to be worth visiting;
  - for each cluster pick one goal cell: a free frontier cell close to the
    cluster centroid that is far enough from the robot, safe in the
    costmaps, not blacklisted and not close to a goal already visited;
  - orient the goal towards the unexplored side of the frontier;
  - score clusters (closer and bigger is better) and return them best-first.

Exploration is finished when no cluster yields a goal. The result records
whether clusters were dropped because of the blacklist, so the caller can
tell "nothing left" from "only blacklisted frontiers left".
"""

from collections import deque
from dataclasses import dataclass, field
import math
from typing import Callable, List, Optional, Sequence, Tuple

from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import OccupancyGrid
import numpy as np

FREE_MAX = 10   # occupancy values 0..FREE_MAX are treated as "free"
UNKNOWN = -1
COSTMAP_COST_LIMIT = 75   # Nav2 cost is 0-100; above this is too close to an obstacle

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


def _yaw_of(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


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


@dataclass
class SelectionResult:
    """Candidates best-first plus counts explaining why some were dropped."""

    candidates: List[FrontierCandidate] = field(default_factory=list)
    clusters: List[List[Tuple[int, int]]] = field(default_factory=list)
    blocked_clusters: int = 0


class FrontierSelector:
    """Rank frontier goals from a SLAM map."""

    def __init__(
        self,
        min_cluster_size: int = 8,
        min_goal_distance: float = 0.6,
        revisit_radius: float = 0.5,
        max_history: int = 20,
        distance_weight: float = 1.0,
        size_weight: float = 0.02,
        cost_limit: int = COSTMAP_COST_LIMIT,
    ):
        self.min_cluster_size = min_cluster_size
        self.min_goal_distance = min_goal_distance
        self.revisit_radius = revisit_radius
        self.distance_weight = distance_weight
        self.size_weight = size_weight
        self.cost_limit = cost_limit
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
    def _frontier_facing_yaw(grid: OccupancyGrid, cluster) -> float:
        """Average direction from frontier cells towards their unknown neighbours."""
        data = _grid_array(grid)
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

    def _score(self, robot_x, robot_y, goal_x, goal_y, cluster_size) -> float:
        """Return the cost of a goal (lower is better): close and big wins."""
        dist = math.hypot(goal_x - robot_x, goal_y - robot_y)
        return self.distance_weight * dist - self.size_weight * cluster_size

    def _pick_goal_cell(
        self,
        grid: OccupancyGrid,
        cluster,
        robot_x: float,
        robot_y: float,
        costmaps: Sequence[CostmapLayer],
        is_blocked: Optional[Callable[[float, float], bool]],
    ) -> Tuple[Optional[Tuple[float, float]], bool]:
        """
        Pick the goal cell for a cluster.

        Returns ((x, y) or None, blocked) where blocked says a cell was
        rejected only because of the blacklist.
        """
        cen_x, cen_y = self._cluster_centroid(cluster)
        blocked = False
        for cell in sorted(
            cluster, key=lambda c: math.hypot(c[0] - cen_x, c[1] - cen_y)
        ):
            x, y = cell_to_world(grid, cell[0], cell[1])
            if math.hypot(x - robot_x, y - robot_y) < self.min_goal_distance:
                continue
            if is_blocked is not None and is_blocked(x, y):
                blocked = True
                continue
            if self._too_close_to_history(x, y):
                continue
            if not all(m.is_safe(x, y, self.cost_limit) for m in costmaps):
                continue
            return (x, y), blocked
        return None, blocked

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

        clusters = self.find_clusters(grid)
        result = SelectionResult(clusters=clusters)

        for cluster in clusters:
            picked, blocked = self._pick_goal_cell(
                grid, cluster, robot_x, robot_y, costmaps, is_blocked
            )
            if picked is None:
                if blocked:
                    result.blocked_clusters += 1
                continue
            goal_x, goal_y = picked

            pose = PoseStamped()
            pose.header.frame_id = frame_id
            pose.pose.position.x = goal_x
            pose.pose.position.y = goal_y
            yaw = self._frontier_facing_yaw(grid, cluster)
            qx, qy, qz, qw = yaw_to_quaternion(yaw)
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw

            result.candidates.append(
                FrontierCandidate(
                    pose=pose,
                    score=self._score(robot_x, robot_y, goal_x, goal_y, len(cluster)),
                    cluster_size=len(cluster),
                )
            )

        result.candidates.sort(key=lambda c: c.score)
        return result
