# test_worlds

Generate and/or launch seeded Gazebo worlds with physical dimensions and
independent raster resolution.

## Example

The below command will generate a new world with the specified features, using the specified seed. Then, it will launch gazebo and spawn a turtlebot in the generated world.

```bash
TURTLEBOT3_MODEL=waffle_pi ros2 launch test_worlds explore.launch.py rows:=50 cols:=50 \
features:="room:3,dead_end:2,narrow_passage:4,large_empty_space:1,l_corridor:3,wall_section:2,plus:2" min_feature_gap:=2 seed:=97
```

For compact maps with thin walls, specify physical dimensions, raster
resolution, and wall thickness independently:

```bash
ros2 launch test_worlds explore.launch.py \
  width:=8 height:=6 resolution:=0.05 wall_thickness:=0.08 \
  angled_wall:="-2,-1,2,1" seed:=97
```

`width`, `height`, `resolution`, and `wall_thickness` are in metres. The
physical wall geometry is emitted as oriented Gazebo boxes, while the raster
grid is derived at the requested resolution for occupancy, connectivity,
spawn selection, and marker placement. The older `rows`, `cols`, and
`cell_size` options remain available for existing commands.

Available vector obstacles include `room`, `dead_end`, `narrow_passage`,
`l_corridor`, `wall_section`, `large_empty_space`, `plus`, `angled_wall`, and
`zigzag`. They can be combined, for example:

```bash
ros2 launch test_worlds explore.launch.py \
  width:=10 height:=7 resolution:=0.05 wall_thickness:=0.08 \
  features:="room:2,l_corridor:2,angled_wall:2,zigzag:1,plus:1" \
  min_feature_gap:=0.25 seed:=97
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
