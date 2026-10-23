"""Generate an exploration world and launch Gazebo with the TurtleBot3."""

import json
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _start_simulation(context, *args, **kwargs):
    """Start Gazebo, robot state publisher, and spawn the robot after the world generator succeeds."""
    event = kwargs.get("event")
    if event is not None and event.returncode != 0:
        raise RuntimeError(f"World generator failed with exit code {event.returncode}")

    # Read spawn position from the generator's JSON output
    output_path = LaunchConfiguration("output").perform(context)
    with open(output_path + ".spawn.json", encoding="utf-8") as spawn_file:
        spawn = json.load(spawn_file)

    # Package paths
    gazebo_share = get_package_share_directory("gazebo_ros")
    tb3_gazebo_share = get_package_share_directory("turtlebot3_gazebo")
    tb3_launch_dir = Path(tb3_gazebo_share) / "launch"

    # Get use_sim_time from launch configuration
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context)

    # Start Gazebo server
    server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(gazebo_share) / "launch" / "gzserver.launch.py")
        ),
        launch_arguments={
            "world": output_path,
            "verbose": "true",
        }.items(),
    )

    # Start Gazebo client (GUI) if requested
    client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(gazebo_share) / "launch" / "gzclient.launch.py")
        ),
        condition=IfCondition(LaunchConfiguration("gui")),
    )

    # Start robot_state_publisher (from turtlebot3_gazebo)
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(tb3_launch_dir / "robot_state_publisher.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    # Spawn the TurtleBot3 model
    spawn_turtlebot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(tb3_launch_dir / "spawn_turtlebot3.launch.py")
        ),
        launch_arguments={
            "x_pose": str(spawn["x"]),
            "y_pose": str(spawn["y"]),
        }.items(),
    )

    return [server, client, robot_state_publisher, spawn_turtlebot]


def generate_launch_description():
    """Create the world generator, set environment, and run the simulator after generation succeeds."""
    package_share = Path(get_package_share_directory("test_worlds"))

    # Environment variables to ensure Gazebo finds models and resources
    model_paths = [
        "/usr/share/gazebo-11/models",
        str(package_share / "models"),
    ]
    resource_paths = [
        "/usr/share/gazebo-11",
        str(package_share / "resource"),
    ]

    # Generator process
    generator = ExecuteProcess(
        cmd=[
            "ros2", "run", "test_worlds", "generate_world",
            "--seed", LaunchConfiguration("seed"),
            "--rows", LaunchConfiguration("rows"),
            "--cols", LaunchConfiguration("cols"),
            "--cell-size", LaunchConfiguration("cell_size"),
            "--width", LaunchConfiguration("width"),
            "--height", LaunchConfiguration("height"),
            "--resolution", LaunchConfiguration("resolution"),
            "--wall-thickness", LaunchConfiguration("wall_thickness"),
            "--angled-wall", LaunchConfiguration("angled_wall"),
            "--features", LaunchConfiguration("features"),
            "--min-feature-gap", LaunchConfiguration("min_feature_gap"),
            "--output", LaunchConfiguration("output"),
        ],
        output="screen",
    )

    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument("seed", default_value="42"),
        DeclareLaunchArgument("rows", default_value="20"),
        DeclareLaunchArgument("cols", default_value="20"),
        DeclareLaunchArgument("cell_size", default_value="0.5"),
        DeclareLaunchArgument("width", default_value="0"),
        DeclareLaunchArgument("height", default_value="0"),
        DeclareLaunchArgument("resolution", default_value="0.05"),
        DeclareLaunchArgument("wall_thickness", default_value="0.1"),
        DeclareLaunchArgument("angled_wall", default_value=""),
        DeclareLaunchArgument(
            "features",
            default_value="room:3,l_corridor:2,dead_end:1,narrow_passage:1",
        ),
        DeclareLaunchArgument("min_feature_gap", default_value="0"),
        DeclareLaunchArgument("output", default_value="/tmp/exploration_world.sdf"),
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("pause", default_value="false"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),

        # Environment setup
        SetEnvironmentVariable(
            name="GAZEBO_MODEL_PATH",
            value=os.pathsep.join(model_paths),
        ),
        SetEnvironmentVariable(
            name="GAZEBO_RESOURCE_PATH",
            value=os.pathsep.join(resource_paths),
        ),

        # Run the generator
        generator,

        # After generator finishes, start simulation
        RegisterEventHandler(
            OnProcessExit(
                target_action=generator,
                on_exit=[OpaqueFunction(function=_start_simulation)],
            )
        ),
    ])