# test_worlds

Generate and/or launch test randomised (seeded) gazebo worlds with specified parameters.

## Example

The below command will generate a new world with the specified features, using the specified seed. Then, it will launch gazebo and spawn a turtlebot in the generated world.

```bash
TURTLEBOT3_MODEL=waffle_pi ros2 launch test_worlds explore.launch.py rows:=50 cols:=50 \
features:="room:3,dead_end:2,narrow_passage:4,large_empty_space:1,l_corridor:3,wall_section:2,plus:2" min_feature_gap:=2 seed:=97
```

`explore.launch.py` is simulation infrastructure only. It starts the generated
world, Gazebo, TurtleBot3 state publishing, and the robot spawn; it does not
start SLAM, Nav2, RViz, frontier exploration, navigation supervision, ArUco
detection, or Mission Control. Start those nodes together with:

```bash
ros2 launch simple_explorer project.launch.py use_sim_time:=true
```

On a physical robot, replace this launch with the robot hardware bringup and
run the project launch with its default `use_sim_time:=false`.
