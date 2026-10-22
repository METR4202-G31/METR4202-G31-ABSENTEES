"""
Unit tests for the information-gain and turn-penalty scoring terms.

Like test_frontier_selector.py these use small synthetic maps with 1 m cells,
so one cell = 1 m^2 of area. Run from the package directory with:

    python3 -m pytest test/test_frontier_gain.py -v
"""

import math

from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid
import numpy as np

from simple_explorer.frontier_selector import FrontierSelector, information_gain


def make_grid(rows):
    """'.' free, '#' occupied, '?' unknown. Top row is the highest y."""
    value_map = {".": 0, "#": 100, "?": -1}
    grid = OccupancyGrid()
    grid.info.width = len(rows[0])
    grid.info.height = len(rows)
    grid.info.resolution = 1.0
    grid.info.origin.orientation.w = 1.0
    grid.data = [value_map[ch] for row in reversed(rows) for ch in row]
    return grid


def robot_at(x, y, yaw=0.0):
    pose = Pose()
    pose.position.x = float(x)
    pose.position.y = float(y)
    pose.orientation.z = math.sin(yaw / 2.0)
    pose.orientation.w = math.cos(yaw / 2.0)
    return pose


def as_array(grid):
    return np.asarray(grid.data, dtype=np.int16).reshape(grid.info.height, grid.info.width)


# A mapped room with a 2x2 unmapped speck inside it, and a corridor on the
# right that leads out into a large unknown area.
SPECK_AND_CORRIDOR = [
    "###########??????????",
    "#.........#??????????",
    "#..??.....#??????????",
    "#..??..............??",
    "#.........#??????????",
    "###########??????????",
]


def clusters_by_x(selector, grid):
    """Return clusters sorted left to right (speck first, corridor second)."""
    clusters = selector.find_clusters(grid)
    return sorted(clusters, key=lambda c: min(x for x, _ in c))


def test_enclosed_speck_has_tiny_gain_but_corridor_has_lots():
    grid = make_grid(SPECK_AND_CORRIDOR)
    selector = FrontierSelector(min_cluster_size=1)
    speck, corridor = clusters_by_x(selector, grid)
    data = as_array(grid)
    # The flood fill cannot leave the speck: it is surrounded by free cells.
    assert information_gain(data, speck, max_depth=50, max_cells=10000) == 4
    assert information_gain(data, corridor, max_depth=50, max_cells=10000) > 20


def test_gain_is_capped():
    grid = make_grid(SPECK_AND_CORRIDOR)
    selector = FrontierSelector(min_cluster_size=1)
    _, corridor = clusters_by_x(selector, grid)
    assert information_gain(as_array(grid), corridor, max_depth=50, max_cells=7) == 7


def test_min_gain_drops_the_speck_and_keeps_the_corridor():
    grid = make_grid(SPECK_AND_CORRIDOR)
    selector = FrontierSelector(
        min_cluster_size=1, min_goal_distance=0.0, min_gain=5.0, gain_weight=1.0
    )
    result = selector.rank_candidates(grid, robot_at(5.5, 1.5))
    assert result.low_gain_clusters == 1
    assert result.candidates
    assert all(c.pose.pose.position.x > 10.0 for c in result.candidates)


def test_old_scoring_picks_the_closer_speck():
    # Documents the problem seen in simulation: without gain the speck wins.
    grid = make_grid(SPECK_AND_CORRIDOR)
    selector = FrontierSelector(min_cluster_size=1, min_goal_distance=0.0)
    best = selector.rank_candidates(grid, robot_at(7.5, 2.5)).candidates[0]
    assert best.pose.pose.position.x < 6.0


# A straight corridor with unknown space at both ends.
CORRIDOR = [
    "###############",
    "?.............?",
    "###############",
]


def best_x(selector, yaw):
    grid = make_grid(CORRIDOR)
    best = selector.rank_candidates(grid, robot_at(7.5, 1.5, yaw)).candidates[0]
    return best.pose.pose.position.x


def test_turn_penalty_keeps_going_the_way_the_robot_faces():
    selector = FrontierSelector(min_cluster_size=1, min_goal_distance=0.0, turn_weight=1.0)
    assert best_x(selector, yaw=0.0) > 7.5          # facing east -> east end
    assert best_x(selector, yaw=math.pi) < 7.5      # facing west -> west end


def test_turn_penalty_loses_to_a_much_closer_frontier():
    # Facing east, but the west end is far closer: distance should still win.
    grid = make_grid(CORRIDOR)
    selector = FrontierSelector(min_cluster_size=1, min_goal_distance=0.0, turn_weight=0.5)
    best = selector.rank_candidates(grid, robot_at(3.5, 1.5, 0.0)).candidates[0]
    assert best.pose.pose.position.x < 3.5
