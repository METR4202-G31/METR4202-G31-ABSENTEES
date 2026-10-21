# simple_explorer

Frontier exploration for the TurtleBot3. The `frontier_explorer` node decides
where to explore next and hands each goal to the navigation supervisor. It does
not drive Nav2 itself.

## How it works

1. Find frontier cells (free cells next to unknown space) in the SLAM `/map` and
   group them into clusters. Clusters under `min_cluster_size` cells (default 20,
   i.e. 1 m) are noise or too small to be worth the trip; the aim is to reach
   unknown space quickly, not to map every corner.
2. Work out the driving distance from the robot to every known free cell: a
   shortest-path search over free space with walls inflated by
   `travel_clearance`, on ~0.1 m cells. Frontiers behind a wall therefore count
   as far away, and frontiers that cannot be reached through known free space
   are dropped without asking the planner. (`use_travel_distance: false` goes
   back to straight-line distances.)
3. For each cluster pick one goal cell: a reachable free frontier cell near the
   cluster centroid that is at least `min_goal_distance` from the robot, safe in
   the global and local costmaps, not blacklisted, and not near a goal already
   visited.
4. Rank clusters by `distance_weight * driving distance - size_weight * cluster
   size - gain_weight * gain + turn_weight * turn` (shorter drive, more unknown space
   behind the frontier and less turning is better). *Gain* is the area of unknown
   space reached by flood-filling from the frontier into unknown cells (capped at
   `max_gain_area`); a frontier with less than `min_gain` behind it, such as an
   unmapped speck inside an already-mapped room, is ignored. *Turn* is how far
   the robot must turn to face the goal, which keeps it finishing the corridor or
   room it is in. If `precheck_path` is set, ask
   the Nav2 planner (`compute_path_to_pose`) whether a path exists. A goal with no
   path is blacklisted and published on `/explore/blacklist`.
5. Publish the goal on `/explore` and send it to the supervisor's `ManagedNavigate`
   action. One outcome comes back per goal.
6. If the frontier disappears after a map update the goal is cancelled and a new
   one is chosen.
7. When no cluster yields a goal for a few cycles the node publishes a `DONE_*`
   status and stops sending goals.

## Blacklist

The explorer owns the blacklist policy. It merges two sources:

* goals it rejected itself (no path) or that `ManagedNavigate` reported as failed;
* the supervisor's `/failed_regions` record (regions with at least
  `min_failure_count` failures).

`/failed_regions` is only a record of failures; whether a region blocks a goal is
decided here.

## Interfaces

| Direction | Topic / action | Type | Notes |
|---|---|---|---|
| sub | `/map` | `nav_msgs/OccupancyGrid` | SLAM map |
| sub | `/global_costmap/costmap`, `/local_costmap/costmap` | `nav_msgs/OccupancyGrid` | local costmap is in `odom`, converted via tf |
| sub | `/failed_regions` | `navigation_interfaces/FailedRegionArray` | from the supervisor |
| sub | `/explore/resume` | `std_msgs/Bool` | from mission control: `false` stops (cancels the active goal), `true` resumes; resuming after `DONE_*` retries blacklisted goals |
| pub | `/explore` | `geometry_msgs/PoseStamped` | every goal sent, just a pose in `map` |
| pub | `/explore/blacklist` | `geometry_msgs/PoseStamped` | every goal the explorer rejected itself |
| pub | `/explore/status` | `std_msgs/String` | `WAITING`, `EXPLORING`, `STOPPED`, `DONE_NO_FRONTIERS`, `DONE_BLACKLISTED_REMAIN` (transient-local) |
| pub | `/explore/frontiers` | `visualization_msgs/MarkerArray` | frontier cells, next goal, blacklist (for RViz) |
| action client | `managed_navigate` | `navigation_interfaces/ManagedNavigate` | needs the supervisor's action server |
| action client | `compute_path_to_pose` | `nav2_msgs/ComputePathToPose` | optional goal pre-check |

`DONE_NO_FRONTIERS` means the map has no reachable frontier left.
`DONE_BLACKLISTED_REMAIN` means only blacklisted frontiers are left, so mission
control may want to resume for a retry pass.

The supervisor must act on the `ManagedNavigate` action only. `/explore` is
informational; if the supervisor also navigated from it, every goal would be sent twice.

Parameters live in `config/frontier_explorer.yaml`.

## Build and run

```bash
colcon build --symlink-install --packages-select simple_explorer
source install/setup.bash

# Gazebo + SLAM + Nav2 + RViz + explorer (world:=, x_pose:=, y_pose:= optional)
TURTLEBOT3_MODEL=waffle_pi ros2 launch simple_explorer exploration_demo.launch.py

# SLAM + Nav2 + RViz + explorer, when the simulator is already running
ros2 launch simple_explorer explore.launch.py

# Complete project stack, when simulation or robot bringup is already running
ros2 launch simple_explorer project.launch.py use_sim_time:=true
```

Until a `ManagedNavigate` server is running the explorer stays in `WAITING`.
`project.launch.py` starts that server together with the explorer, as well as
the ArUco detector and Mission Control. Use `use_sim_time:=false` (the default)
with a physical robot.

Stop or resume from the command line:

```bash
ros2 topic pub --once /explore/resume std_msgs/msg/Bool "{data: false}"
ros2 topic pub --once /explore/resume std_msgs/msg/Bool "{data: true}"
```

## Tests

```bash
colcon test --packages-select simple_explorer && colcon test-result --verbose
python -m pytest project/simple_explorer/test   # needs the ROS environment sourced
```

`frontier_selector.py`, `blacklist.py` and `goal_outcome.py` are ROS-node-free
and are covered by plain pytest; the node itself is a thin wrapper around them.

## Known limitations

* Frontier goals are cells on the frontier, so they can sit close to walls; the
  costmap check keeps them out of high-cost areas but not out of tight gaps.
* A path from the planner does not guarantee the controller can follow it.
* Wide frontiers can be revisited at other spots once a goal there is done,
  because visited goals only cover `revisit_radius`.
* Frontier detection runs over the whole map each cycle (numpy, well under a
  second for 800x800 cells).
