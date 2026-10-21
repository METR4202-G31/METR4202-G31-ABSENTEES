"""
Unit tests for frontier_selector.FrontierSelector.

These run as plain pytest against synthetic OccupancyGrid messages: no
Gazebo, Nav2 or ROS graph needed, only a sourced ROS environment so that
nav_msgs and geometry_msgs import. Run from the package directory with:

    python3 -m pytest test/test_frontier_selector.py -v
"""

import math

from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid
import numpy as np
import pytest

from simple_explorer.frontier_selector import (
    CostmapLayer,
    frontier_near,
    FrontierSelector,
    TravelMap,
)


def make_grid(rows, resolution=1.0, origin=(0.0, 0.0)):
    """
    Build an OccupancyGrid from a list of strings, one per row.

    The top row is the highest y. '.' is free, '#' occupied, '?' unknown.
    """
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
    # ROS message fields are strictly typed, so cast in case a caller
    # passes an int.
    pose.position.x = float(x)
    pose.position.y = float(y)
    return pose


def goal_xy(candidate):
    position = candidate.pose.pose.position
    return position.x, position.y


# A 7x7 map: unknown border ring around a 5x5 free room. The frontier is the
# whole ring of free cells just inside the border, as at the start of a run.
RING = [
    "???????",
    "?.....?",
    "?.....?",
    "?.....?",
    "?.....?",
    "?.....?",
    "???????",
]


def test_rejects_tiny_frontier_cluster():
    # One free cell with a single unknown neighbour: a 1-cell frontier,
    # below the default min_cluster_size of 8.
    grid = make_grid(["?##", ".##", "###"])
    selector = FrontierSelector(min_cluster_size=8)

    assert selector.find_clusters(grid) == []
    assert selector.rank_candidates(grid, robot_at(0, 0)).candidates == []


def test_finds_and_scores_a_real_frontier():
    rows = [
        "?????",
        ".....",
        ".....",
        ".....",
        ".....",
    ]
    grid = make_grid(rows)
    selector = FrontierSelector(min_cluster_size=3)

    clusters = selector.find_clusters(grid)
    assert len(clusters) == 1
    assert len(clusters[0]) == 5

    result = selector.rank_candidates(grid, robot_at(2.0, 0.5))
    assert len(result.candidates) == 1
    x, y = goal_xy(result.candidates[0])
    assert y == pytest.approx(3.5)
    assert x == pytest.approx(2.5)


def test_orients_towards_unknown_area():
    # Unknown space lies below the free area (negative y), so the goal
    # should face roughly -pi/2.
    rows = [
        ".....",
        ".....",
        "?????",
    ]
    grid = make_grid(rows)
    selector = FrontierSelector(min_cluster_size=3)

    result = selector.rank_candidates(grid, robot_at(2.0, 2.5))
    assert result.candidates

    orientation = result.candidates[0].pose.pose.orientation
    yaw = 2.0 * math.atan2(orientation.z, orientation.w)
    assert yaw == pytest.approx(-math.pi / 2, abs=0.2)


def test_avoids_revisiting_visited_goal():
    rows = [
        "?????",
        ".....",
        ".....",
    ]
    grid = make_grid(rows)
    # The radius is wider than the whole 5-cell frontier, so one visited
    # goal covers all of it. (A narrower radius lets a wide frontier be
    # visited again at other spots.)
    selector = FrontierSelector(min_cluster_size=3, revisit_radius=2.5)

    first = selector.rank_candidates(grid, robot_at(2.0, 0.0)).candidates
    assert first

    # Selection alone does not mark anything visited.
    assert selector.rank_candidates(grid, robot_at(2.0, 0.0)).candidates

    selector.mark_visited(*goal_xy(first[0]))
    assert selector.rank_candidates(grid, robot_at(2.0, 0.0)).candidates == []

    selector.clear_history()
    assert selector.rank_candidates(grid, robot_at(2.0, 0.0)).candidates


def test_exploration_complete_when_no_frontiers_left():
    grid = make_grid(["....."] * 3)
    selector = FrontierSelector()

    assert selector.find_clusters(grid) == []
    result = selector.rank_candidates(grid, robot_at(0.0, 0.0))
    assert result.candidates == []
    assert result.blocked_clusters == 0


def test_prefers_bigger_closer_frontier():
    rows = [
        "?..??????",
        "....?????",
        ".........",
        ".........",
    ]
    grid = make_grid(rows)
    selector = FrontierSelector(
        min_cluster_size=2, distance_weight=1.0, size_weight=0.5
    )

    result = selector.rank_candidates(grid, robot_at(1.0, 0.0))
    assert result.candidates
    # The big frontier is on the right; the strong size weight makes the
    # selector reach across for it.
    assert goal_xy(result.candidates[0])[0] > 3.0


def test_goal_is_never_closer_than_min_goal_distance():
    grid = make_grid(RING)
    selector = FrontierSelector(min_cluster_size=3, min_goal_distance=1.5)
    robot = robot_at(3.5, 3.5)    # standing in the middle of the ring

    result = selector.rank_candidates(grid, robot)

    assert len(result.candidates) == 1
    x, y = goal_xy(result.candidates[0])
    assert math.hypot(x - 3.5, y - 3.5) >= 1.5


def test_drops_cluster_when_every_cell_is_too_close():
    grid = make_grid(RING)
    selector = FrontierSelector(min_cluster_size=3, min_goal_distance=10.0)

    assert selector.rank_candidates(grid, robot_at(3.5, 3.5)).candidates == []


def test_goal_lies_on_a_frontier_cell_not_the_centroid():
    # The centroid of a ring is its (free, non-frontier) middle.
    grid = make_grid(RING)
    selector = FrontierSelector(min_cluster_size=3, min_goal_distance=0.0)

    result = selector.rank_candidates(grid, robot_at(3.5, 3.5))

    x, y = goal_xy(result.candidates[0])
    assert (int(x), int(y)) in result.clusters[0]
    assert (int(x), int(y)) != (3, 3)


def test_costmap_obstacle_moves_goal_to_another_frontier_cell():
    rows = [
        "?????",
        ".....",
        ".....",
    ]
    grid = make_grid(rows)
    selector = FrontierSelector(min_cluster_size=3)
    # The centroid cell (2, 1) is blocked in the costmap.
    costmap = make_grid(
        [
            ".....",
            "..#..",
            ".....",
        ]
    )

    clear = selector.rank_candidates(grid, robot_at(2.0, 0.0))
    blocked = selector.rank_candidates(
        grid, robot_at(2.0, 0.0), [CostmapLayer(costmap)]
    )

    assert goal_xy(clear.candidates[0]) == pytest.approx((2.5, 1.5))
    assert blocked.candidates
    assert goal_xy(blocked.candidates[0]) != pytest.approx((2.5, 1.5))


def test_no_goal_when_whole_costmap_is_unsafe():
    grid = make_grid(["?????", ".....", "....."])
    selector = FrontierSelector(min_cluster_size=3)
    costmap = make_grid(["#####", "#####", "#####"])

    result = selector.rank_candidates(
        grid, robot_at(2.0, 0.0), [CostmapLayer(costmap)]
    )

    assert result.candidates == []


def test_unknown_costmap_cell_is_unsafe():
    grid = make_grid(["?????", ".....", "....."])
    selector = FrontierSelector(min_cluster_size=3)
    costmap = make_grid(["?????", "?????", "?????"])

    result = selector.rank_candidates(
        grid, robot_at(2.0, 0.0), [CostmapLayer(costmap)]
    )

    assert result.candidates == []


def test_out_of_bounds_is_unsafe_for_global_but_not_for_local_costmap():
    grid = make_grid(["?????", ".....", "....."])
    selector = FrontierSelector(min_cluster_size=3)
    # A costmap that covers none of the frontier, like a rolling local
    # costmap around a robot that is far away.
    far_costmap = make_grid(["...", "..."], origin=(50.0, 50.0))

    strict = selector.rank_candidates(
        grid, robot_at(2.0, 0.0), [CostmapLayer(far_costmap, strict_bounds=True)]
    )
    lenient = selector.rank_candidates(
        grid, robot_at(2.0, 0.0), [CostmapLayer(far_costmap, strict_bounds=False)]
    )

    assert strict.candidates == []
    assert lenient.candidates


def test_costmap_layer_applies_its_frame_transform():
    grid = make_grid(["?????", ".....", "....."])
    selector = FrontierSelector(min_cluster_size=3)
    # Costmap frame is shifted by +10 m in x; its obstacle sits at the
    # shifted position of the centroid cell (2.5, 1.5) -> (12.5, 1.5).
    local = make_grid(
        [
            "...............",
            "............#..",
            "...............",
        ],
        origin=(0.0, 0.0),
    )
    layer = CostmapLayer(local, lambda x, y: (x + 10.0, y), strict_bounds=False)

    result = selector.rank_candidates(grid, robot_at(2.0, 0.0), [layer])

    assert goal_xy(result.candidates[0]) != pytest.approx((2.5, 1.5))


def test_blacklisted_cells_are_skipped():
    grid = make_grid(["?????", ".....", "....."])
    selector = FrontierSelector(min_cluster_size=3)

    result = selector.rank_candidates(
        grid, robot_at(2.0, 0.0), is_blocked=lambda x, y: x < 2.0
    )

    assert result.candidates
    assert goal_xy(result.candidates[0])[0] >= 2.0


def test_fully_blacklisted_frontier_is_reported_as_blocked():
    grid = make_grid(["?????", ".....", "....."])
    selector = FrontierSelector(min_cluster_size=3)

    result = selector.rank_candidates(
        grid, robot_at(2.0, 0.0), is_blocked=lambda x, y: True
    )

    assert result.candidates == []
    assert result.blocked_clusters == 1


def test_frontier_near_finds_and_misses_frontiers():
    grid = make_grid(["?????", ".....", "....."])

    assert frontier_near(grid, 2.5, 1.5, 0.5)
    assert frontier_near(grid, 2.5, 0.5, 1.5)
    assert not frontier_near(grid, 2.5, 0.5, 0.4)
    assert not frontier_near(grid, 100.0, 100.0, 1.0)


# A wall splits the map. The left frontier is only 2 m away in a straight
# line but the robot has to drive round the end of the wall to reach it;
# the right frontier is further in a straight line but has a direct drive.
#   robot starts at (3.5, 0.5), bottom of the right-hand side.
BEHIND_WALL = [
    "???#?????",
    "...#.....",
    "...#.....",
    "...#.....",
    "...#.....",
    "...#.....",
    "...#.....",
    ".........",
]


def test_travel_map_measures_driving_distance_round_walls():
    grid = make_grid(BEHIND_WALL)
    array = np.asarray(grid.data, dtype=np.int16).reshape(
        grid.info.height, grid.info.width
    )
    travel = TravelMap(array, grid.info.resolution, (4, 1), clearance=0.0)

    assert travel.valid
    assert travel.distance(4, 1) == 0.0
    # (2, 1) is straight across the wall: 2 cells away, but the drive goes
    # diagonally down round the wall end at y=0 and back up.
    assert travel.distance(2, 1) == pytest.approx(2 * math.sqrt(2.0))
    # Further up the wall the detour grows: round the end, then up 5.
    assert travel.distance(2, 6) == pytest.approx(2 * math.sqrt(2.0) + 5)
    # Unknown cells are never entered.
    assert math.isinf(travel.distance(0, 7))


def test_prefers_frontier_with_the_shorter_drive_not_the_straight_line():
    grid = make_grid(BEHIND_WALL)
    robot = robot_at(2.5, 5.5)    # left of the wall, near the top

    straight = FrontierSelector(
        min_cluster_size=3, use_travel_distance=False, size_weight=0.0
    )
    driving = FrontierSelector(min_cluster_size=3, size_weight=0.0)

    # Straight-line distances make the right frontier (just over the wall)
    # look as close as the left one; driving distance does not.
    assert goal_xy(driving.rank_candidates(grid, robot).candidates[0])[0] < 3.0
    straight_best = straight.rank_candidates(grid, robot).candidates
    driving_best = driving.rank_candidates(grid, robot).candidates
    assert driving_best[1].score > straight_best[1].score


def test_unreachable_frontier_is_dropped_and_counted():
    rows = [
        "?????",
        ".....",
        "#####",
        ".....",
    ]
    grid = make_grid(rows)
    selector = FrontierSelector(min_cluster_size=3)

    result = selector.rank_candidates(grid, robot_at(2.5, 0.5))

    assert len(result.clusters) == 1
    assert result.candidates == []
    assert result.unreachable_clusters == 1
    assert result.blocked_clusters == 0


def test_falls_back_to_straight_line_when_robot_is_not_on_free_space():
    rows = [
        "?????",
        ".....",
        ".....",
        "????.",
    ]
    grid = make_grid(rows)
    selector = FrontierSelector(min_cluster_size=3)

    # Robot on an unknown cell: no driving distances, but goals still come.
    result = selector.rank_candidates(grid, robot_at(0.5, 0.5))

    assert result.candidates
    assert result.unreachable_clusters == 0


def test_clearance_closes_gaps_narrower_than_the_robot():
    # 5 cm cells; the wall has a one-cell (5 cm) gap.
    rows = [
        "?????????",
        ".........",
        ".........",
        ".........",
        "####.####",
        ".........",
        ".........",
        ".........",
        ".........",
    ]
    grid = make_grid(rows, resolution=0.05)
    robot = robot_at(0.225, 0.025)    # bottom row, below the gap
    narrow = FrontierSelector(
        min_cluster_size=3, min_goal_distance=0.0, travel_clearance=0.1
    )
    thin = FrontierSelector(
        min_cluster_size=3, min_goal_distance=0.0, travel_clearance=0.0
    )

    # Passable for a point robot, closed once walls are inflated by 0.1 m.
    assert thin.rank_candidates(grid, robot).candidates
    blocked = narrow.rank_candidates(grid, robot)
    assert blocked.candidates == []
    assert blocked.unreachable_clusters == 1
