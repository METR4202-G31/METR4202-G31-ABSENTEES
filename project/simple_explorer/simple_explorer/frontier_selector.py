"""
frontier_selector.py

Frontier-based exploration goal selector.

Given the current map (nav_msgs/OccupancyGrid), the robot's pose, and
(optionally) Nav2's global costmap (also nav_msgs/OccupancyGrid, per
Nav2's own costmap publisher), this module:
  - finds frontier cells (free cells bordering unknown space) and groups
    them into clusters,
  - rejects clusters that are too small to be worth visiting,
  - for each remaining cluster, builds a goal PoseStamped: position at the
    cluster centroid, orientation facing into the unexplored side of the
    frontier,
  - scores each candidate goal (closer + bigger clusters score better),
  - avoids re-sending goals close to ones already attempted,
  - if a global costmap is supplied, rejects/nudges goals that land in a
    high-cost (too close to an obstacle) region of Nav2's costmap,
  - reports exploration complete (returns None) when nothing worthwhile
    is left.

This module does NOT talk to Nav2 or drive the robot -- it only decides
where to go next. A separate node (e.g. navigation_supervisor) is
responsible for subscribing to the map/costmap/robot pose, calling this,
sending the returned PoseStamped to Nav2, and reporting back success or
failure.

Note: only the *global* costmap is checked (it shares the `map` frame
with the SLAM map, so no coordinate transform is needed). Nav2's local
costmap is in the `odom` frame and would need a tf2 lookup to use here --
left for a future pass if it proves necessary.
"""

from collections import deque
from dataclasses import dataclass
import math
from typing import List, Optional, Tuple

from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import OccupancyGrid

FREE_MAX = 10   # cell values 0..FREE_MAX are treated as "free"
UNKNOWN = -1

NEIGHBOURS_8 = [
    (dx, dy)
    for dy in (-1, 0, 1)
    for dx in (-1, 0, 1)
    if not (dx == 0 and dy == 0)
]


def _in_bounds(x: int, y: int, width: int, height: int) -> bool:
    return 0 <= x < width and 0 <= y < height


def _cell_value(grid: OccupancyGrid, x: int, y: int) -> int:
    return grid.data[y * grid.info.width + x]


def _is_free(grid: OccupancyGrid, x: int, y: int) -> bool:
    return 0 <= _cell_value(grid, x, y) <= FREE_MAX


def _is_unknown(grid: OccupancyGrid, x: int, y: int) -> bool:
    return _cell_value(grid, x, y) == UNKNOWN


def cell_to_world(grid: OccupancyGrid, x: float, y: float) -> Tuple[float, float]:
    """Convert grid cell coordinates (can be fractional) to map-frame metres."""
    info = grid.info
    local_x = (x + 0.5) * info.resolution
    local_y = (y + 0.5) * info.resolution

    q = info.origin.orientation
    yaw = math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)

    world_x = info.origin.position.x + cos_yaw * local_x - sin_yaw * local_y
    world_y = info.origin.position.y + sin_yaw * local_x + cos_yaw * local_y
    return world_x, world_y


def yaw_to_quaternion(yaw: float):
    """Rotation about Z only -- all our goals are in the 2D map plane."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


COSTMAP_COST_LIMIT = 75   # Nav2 cost is 0-100; above this means "too close to an obstacle"
NUDGE_STEP_M = 0.3        # how far to try shifting a goal that lands on a high-cost cell


def _costmap_cost(costmap: OccupancyGrid, world_x: float, world_y: float) -> Optional[int]:
    """Cost (0-100, or -1 = unknown) at a map-frame point. Assumes the
    costmap's origin isn't rotated, which holds for slam_toolbox/Nav2 maps."""
    info = costmap.info
    cell_x = int((world_x - info.origin.position.x) / info.resolution)
    cell_y = int((world_y - info.origin.position.y) / info.resolution)
    if not _in_bounds(cell_x, cell_y, info.width, info.height):
        return None
    return _cell_value(costmap, cell_x, cell_y)


def _nudge_to_safety(costmap: OccupancyGrid, x: float, y: float) -> Optional[Tuple[float, float]]:
    """If (x, y) is safe, return it as-is. Otherwise try the 8 cells
    around it; return the first safe one found, or None if this whole
    area is too close to an obstacle."""
    for step_x, step_y in [(0, 0)] + NEIGHBOURS_8:
        nx, ny = x + step_x * NUDGE_STEP_M, y + step_y * NUDGE_STEP_M
        cost = _costmap_cost(costmap, nx, ny)
        if cost is None or cost <= COSTMAP_COST_LIMIT:
            return nx, ny
    return None


@dataclass
class FrontierGoal:
    pose: PoseStamped
    score: float
    cluster_size: int


class FrontierSelector:
    def __init__(
        self,
        min_cluster_size: int = 8,
        revisit_radius: float = 0.5,
        max_history: int = 20,
        distance_weight: float = 1.0,
        size_weight: float = 0.02,
    ):
        self.min_cluster_size = min_cluster_size
        self.revisit_radius = revisit_radius
        self.distance_weight = distance_weight
        self.size_weight = size_weight
        self._history: deque = deque(maxlen=max_history)

    # ---- history: avoiding recent/repeated frontiers -----------------

    def mark_visited(self, x: float, y: float) -> None:
        """Record a goal (map-frame metres) as attempted, so it isn't
        immediately re-selected. Call this whether the goal succeeds or
        fails -- either way there's no point sending the robot straight
        back to it."""
        self._history.append((x, y))

    def _too_close_to_history(self, x: float, y: float) -> bool:
        return any(
            math.hypot(x - hx, y - hy) < self.revisit_radius
            for hx, hy in self._history
        )

    # ---- frontier detection -------------------------------------------

    def _is_frontier_cell(self, grid: OccupancyGrid, x: int, y: int) -> bool:
        if not _is_free(grid, x, y):
            return False
        width, height = grid.info.width, grid.info.height
        for dx, dy in NEIGHBOURS_8:
            nx, ny = x + dx, y + dy
            if _in_bounds(nx, ny, width, height) and _is_unknown(grid, nx, ny):
                return True
        return False

    def find_clusters(self, grid: OccupancyGrid) -> List[List[Tuple[int, int]]]:
        """Find frontier cells and group adjacent ones into clusters,
        rejecting clusters below min_cluster_size."""
        width, height = grid.info.width, grid.info.height

        frontier_cells = set()
        for y in range(height):
            for x in range(width):
                if self._is_frontier_cell(grid, x, y):
                    frontier_cells.add((x, y))

        clusters = []
        visited = set()
        for start in frontier_cells:
            if start in visited:
                continue
            queue = deque([start])
            visited.add(start)
            cluster = []
            while queue:
                cell = queue.popleft()
                cluster.append(cell)
                cx, cy = cell
                for dx, dy in NEIGHBOURS_8:
                    neighbour = (cx + dx, cy + dy)
                    if neighbour in frontier_cells and neighbour not in visited:
                        visited.add(neighbour)
                        queue.append(neighbour)
            clusters.append(cluster)

        # Requirement: reject tiny frontier clusters (usually just noise
        # from a single stray unknown cell rather than a real unexplored
        # region worth driving to).
        return [c for c in clusters if len(c) >= self.min_cluster_size]

    # ---- goal generation -----------------------------------------------

    def _cluster_centroid(self, cluster) -> Tuple[float, float]:
        cx = sum(c[0] for c in cluster) / len(cluster)
        cy = sum(c[1] for c in cluster) / len(cluster)
        return cx, cy

    def _frontier_facing_yaw(self, grid: OccupancyGrid, cluster) -> float:
        """Average direction from frontier cells towards their unknown
        neighbours, so the robot arrives already facing the unexplored
        area (good for both LiDAR and camera coverage on arrival)."""
        width, height = grid.info.width, grid.info.height
        dx_sum = 0.0
        dy_sum = 0.0
        count = 0
        for cx, cy in cluster:
            for dx, dy in NEIGHBOURS_8:
                nx, ny = cx + dx, cy + dy
                if _in_bounds(nx, ny, width, height) and _is_unknown(grid, nx, ny):
                    dx_sum += dx
                    dy_sum += dy
                    count += 1
        if count == 0:
            return 0.0
        return math.atan2(dy_sum, dx_sum)

    def _score(
        self, robot_x: float, robot_y: float, goal_x: float, goal_y: float, cluster_size: int
    ) -> float:
        """Lower is better: prefer close, big frontiers. Same shape as
        m-explore's cost = alpha*dist - beta*size."""
        dist = math.hypot(goal_x - robot_x, goal_y - robot_y)
        return self.distance_weight * dist - self.size_weight * cluster_size

    def choose_goal(
        self,
        grid: OccupancyGrid,
        robot_pose: Pose,
        global_costmap: Optional[OccupancyGrid] = None,
        frame_id: str = "map",
    ) -> Optional[FrontierGoal]:
        """Pick the best frontier to explore next, or None if exploration
        is complete (no worthwhile, unvisited frontier remains)."""
        robot_x = robot_pose.position.x
        robot_y = robot_pose.position.y

        clusters = self.find_clusters(grid)

        candidates: List[FrontierGoal] = []
        for cluster in clusters:
            cell_x, cell_y = self._cluster_centroid(cluster)
            goal_x, goal_y = cell_to_world(grid, cell_x, cell_y)

            # Requirement: avoid recently/repeatedly sent frontiers.
            if self._too_close_to_history(goal_x, goal_y):
                continue

            yaw = self._frontier_facing_yaw(grid, cluster)
            score = self._score(robot_x, robot_y, goal_x, goal_y, len(cluster))

            pose = PoseStamped()
            pose.header.frame_id = frame_id
            pose.pose.position.x = goal_x
            pose.pose.position.y = goal_y
            qx, qy, qz, qw = yaw_to_quaternion(yaw)
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw

            candidates.append(FrontierGoal(pose=pose, score=score, cluster_size=len(cluster)))

        if not candidates:
            # Requirement: determine when exploration is complete.
            return None

        # Requirement: scoring frontiers -- lowest score wins.
        candidates.sort(key=lambda g: g.score)

        for candidate in candidates:
            x = candidate.pose.pose.position.x
            y = candidate.pose.pose.position.y

            if global_costmap is not None:
                safe = _nudge_to_safety(global_costmap, x, y)
                if safe is None:
                    continue  # too close to an obstacle; try the next frontier
                x, y = safe
                candidate.pose.pose.position.x = x
                candidate.pose.pose.position.y = y

            self.mark_visited(x, y)
            return candidate

        return None
