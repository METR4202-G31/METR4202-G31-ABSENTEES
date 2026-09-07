"""
Unit tests for frontier_selector.FrontierSelector.

These run as plain pytest against synthetic OccupancyGrid messages -- no
Gazebo, Nav2, or even a running ROS graph needed. Run with:

    python3 -m pytest test/test_frontier_selector.py -v

(from the simple_explorer package directory, with ROS humble sourced so
nav_msgs/geometry_msgs are importable).
"""

import math

from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid
import pytest

from simple_explorer.frontier_selector import FrontierSelector


def make_grid(rows, resolution=1.0, origin=(0.0, 0.0)):
    """Build an OccupancyGrid from a list of strings, one per row (top row
    is highest y). Characters: '.' free, '#' occupied, '?' unknown."""
    height = len(rows)
    width = len(rows[0])
    assert all(len(r) == width for r in rows), "all rows must be equal width"

    value_map = {".": 0, "#": 100, "?": -1}
    data = []
    # OccupancyGrid data is row-major with row 0 = smallest y, so reverse
    # the visual (top-down) rows before flattening.
    for row in reversed(rows):
        data.extend(value_map[ch] for ch in row)

    grid = OccupancyGrid()
    grid.info.width = width
    grid.info.height = height
    grid.info.resolution = resolution
    grid.info.origin.position.x = origin[0]
    grid.info.origin.position.y = origin[1]
    grid.info.origin.orientation.w = 1.0
    grid.data = data
    return grid


def robot_at(x, y):
    pose = Pose()
    # ROS message fields are strictly typed -- cast in case a caller
    # passes an int (e.g. robot_at(0, 0)) rather than a float.
    pose.position.x = float(x)
    pose.position.y = float(y)
    return pose


def test_rejects_tiny_frontier_cluster():
    # A single free cell surrounded by walls except one unknown neighbour:
    # only a 1-cell frontier, which should be rejected by the default
    # min_cluster_size (8).
    rows = [
        "?##",
        ".##",
        "###",
    ]
    grid = make_grid(rows)
    selector = FrontierSelector(min_cluster_size=8)

    assert selector.find_clusters(grid) == []
    assert selector.choose_goal(grid, robot_at(0, 0)) is None


def test_finds_and_scores_a_real_frontier():
    # A 5x5 free room with unknown space along the top edge -- a large,
    # legitimate frontier that should be found and selected.
    rows = [
        "?????",
        ".....",
        ".....",
        ".....",
        ".....",
    ]
    grid = make_grid(rows, resolution=1.0)
    selector = FrontierSelector(min_cluster_size=3)

    clusters = selector.find_clusters(grid)
    assert len(clusters) == 1
    assert len(clusters[0]) == 5  # the whole row just below the unknown strip

    goal = selector.choose_goal(grid, robot_at(2.0, 4.0))
    assert goal is not None
    # Goal should sit roughly in the middle of the frontier row, near y=3.5m.
    assert goal.pose.pose.position.y == pytest.approx(3.5, abs=0.5)


def test_orients_towards_unknown_area():
    # Frontier is the bottom of the free area, with unknown space below it
    # (negative y direction) -- the chosen yaw should point roughly "down".
    rows = [
        ".....",
        ".....",
        "?????",
    ]
    grid = make_grid(rows, resolution=1.0)
    selector = FrontierSelector(min_cluster_size=3)

    goal = selector.choose_goal(grid, robot_at(2.0, 1.5))
    assert goal is not None

    qz = goal.pose.pose.orientation.z
    qw = goal.pose.pose.orientation.w
    yaw = 2.0 * math.atan2(qz, qw)
    # Facing towards -y is yaw == -pi/2 (i.e. -90 degrees).
    assert yaw == pytest.approx(-math.pi / 2, abs=0.2)


def test_avoids_revisiting_recent_goal():
    rows = [
        "?????",
        ".....",
        ".....",
    ]
    grid = make_grid(rows, resolution=1.0)
    selector = FrontierSelector(min_cluster_size=3, revisit_radius=2.0)

    first = selector.choose_goal(grid, robot_at(2.0, 0.0))
    assert first is not None

    # Same map, same (only) frontier -- but it's now within revisit_radius
    # of the goal we just chose, so it should be skipped.
    second = selector.choose_goal(grid, robot_at(2.0, 0.0))
    assert second is None


def test_exploration_complete_when_no_frontiers_left():
    # Fully explored map: no unknown cells at all.
    rows = [
        ".....",
        ".....",
        ".....",
    ]
    grid = make_grid(rows)
    selector = FrontierSelector()

    assert selector.find_clusters(grid) == []
    assert selector.choose_goal(grid, robot_at(0.0, 0.0)) is None


def test_nudges_goal_away_from_costmap_obstacle():
    rows = [
        "?????",
        ".....",
        ".....",
    ]
    # Use a cell size equal to the nudge step (0.3m) so a single nudge
    # reliably lands in the next cell over, rather than staying inside
    # the same (larger) blocked cell.
    grid = make_grid(rows, resolution=0.3)
    selector = FrontierSelector(min_cluster_size=3)

    # A costmap is the same wire format as the map (0-100 cost, -1 unknown),
    # so we can build one with make_grid too. Here '#' (=100) sits right on
    # the frontier's centroid cell (2, 1) -- too close to an obstacle.
    costmap = make_grid(
        [
            ".....",
            "..#..",
            ".....",
        ],
        resolution=0.3,
    )

    goal = selector.choose_goal(grid, robot_at(0.0, 0.0), global_costmap=costmap)

    assert goal is not None
    x, y = goal.pose.pose.position.x, goal.pose.pose.position.y
    assert (x, y) != pytest.approx((0.75, 0.45))  # nudged away from the blocked cell centre


def test_returns_none_when_only_frontier_is_unsafe():
    rows = [
        "?????",
        ".....",
        ".....",
    ]
    grid = make_grid(rows)
    selector = FrontierSelector(min_cluster_size=3)

    # Whole costmap is high-cost -- nowhere is safe to nudge to.
    costmap = make_grid(["#####", "#####", "#####"])

    goal = selector.choose_goal(grid, robot_at(2.0, 0.0), global_costmap=costmap)
    assert goal is None


def test_prefers_bigger_closer_frontier():
    # Two frontiers: a small one right next to the robot, and a bigger one
    # further away. With default weights the bigger one should still win
    # if it's not too much further -- tune distance_weight/size_weight to
    # see this trade-off shift.
    rows = [
        "?..??????",
        "....?????",
        ".........",
        ".........",
    ]
    grid = make_grid(rows, resolution=1.0)
    selector = FrontierSelector(min_cluster_size=2, distance_weight=1.0, size_weight=0.5)

    goal = selector.choose_goal(grid, robot_at(1.0, 0.0))
    assert goal is not None
    # The big frontier is on the right-hand side (higher x); expect the
    # selector to reach across for it given the strong size_weight here.
    assert goal.pose.pose.position.x > 3.0
