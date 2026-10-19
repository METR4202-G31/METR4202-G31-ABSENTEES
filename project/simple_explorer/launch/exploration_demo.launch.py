import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    # ---------------------------------------------------------
    # Launch arguments
    # ---------------------------------------------------------

    use_sim_time = LaunchConfiguration("use_sim_time")
    model = LaunchConfiguration("model")
    world = LaunchConfiguration("world")

    x_pose = LaunchConfiguration("x_pose")
    y_pose = LaunchConfiguration("y_pose")

    # ---------------------------------------------------------
    # Package locations
    # ---------------------------------------------------------

    turtlebot3_gazebo_dir = get_package_share_directory("turtlebot3_gazebo")

    gazebo_ros_dir = get_package_share_directory("gazebo_ros")

    nav2_bringup_dir = get_package_share_directory("nav2_bringup")
    navigation_supervisor_dir = get_package_share_directory(
        "navigation_supervisor"
    )

    slam_toolbox_dir = get_package_share_directory("slam_toolbox")

    turtlebot3_launch_dir = os.path.join(
        turtlebot3_gazebo_dir,
        "launch",
    )

    # ---------------------------------------------------------
    # Gazebo server
    #
    # Loads the selected .world file.
    # ---------------------------------------------------------

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                gazebo_ros_dir,
                "launch",
                "gzserver.launch.py",
            )
        ),
        launch_arguments={
            "world": world,
        }.items(),
    )

    # ---------------------------------------------------------
    # Gazebo graphical client
    # ---------------------------------------------------------

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                gazebo_ros_dir,
                "launch",
                "gzclient.launch.py",
            )
        )
    )

    # ---------------------------------------------------------
    # TurtleBot3 robot state publisher
    #
    # Publishes the robot's fixed TF relationships from the URDF.
    # ---------------------------------------------------------

    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                turtlebot3_launch_dir,
                "robot_state_publisher.launch.py",
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
        }.items(),
    )

    # ---------------------------------------------------------
    # Spawn TurtleBot3 into Gazebo
    # ---------------------------------------------------------

    spawn_turtlebot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                turtlebot3_launch_dir,
                "spawn_turtlebot3.launch.py",
            )
        ),
        launch_arguments={
            "x_pose": x_pose,
            "y_pose": y_pose,
        }.items(),
    )

    # ---------------------------------------------------------
    # SLAM Toolbox
    #
    # Uses:
    #   /scan
    #   /odom + TF
    #
    # Provides:
    #   /map
    #   map -> odom TF
    # ---------------------------------------------------------

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                slam_toolbox_dir,
                "launch",
                "online_async_launch.py",
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
        }.items(),
    )

    # ---------------------------------------------------------
    # Nav2
    #
    # SLAM Toolbox provides the live map, so use
    # navigation_launch.py rather than starting a map server
    # and AMCL.
    # ---------------------------------------------------------

    nav2_params = os.path.join(
        navigation_supervisor_dir,
        "config",
        "nav2_unknown_space.yaml",
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                nav2_bringup_dir,
                "launch",
                "navigation_launch.py",
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "autostart": "true",
            "params_file": nav2_params,
        }.items(),
    )

    # ---------------------------------------------------------
    # RViz
    # ---------------------------------------------------------

    rviz_config = os.path.join(
        nav2_bringup_dir,
        "rviz",
        "nav2_default_view.rviz",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            "-d",
            rviz_config,
        ],
        parameters=[
            {
                "use_sim_time": use_sim_time,
            }
        ],
    )

    # ---------------------------------------------------------
    # Frontier explorer
    #
    # Uses:
    #   /map, /global_costmap/costmap, /local_costmap/costmap
    #   /failed_regions, /explore/resume
    #   ManagedNavigate action (navigation_supervisor)
    #   Nav2 compute_path_to_pose (goal pre-check)
    # ---------------------------------------------------------

    explorer_params = os.path.join(
        get_package_share_directory("simple_explorer"),
        "config",
        "frontier_explorer.yaml",
    )

    # Needs a ManagedNavigate action server (navigation_supervisor) to send
    # goals to; until it is running the explorer just reports WAITING.
    explorer = Node(
        package="simple_explorer",
        executable="frontier_explorer",
        name="frontier_explorer",
        output="screen",
        parameters=[
            explorer_params,
            {
                "use_sim_time": use_sim_time,
            },
        ],
    )

    # ---------------------------------------------------------
    # Launch description
    # ---------------------------------------------------------

    return LaunchDescription(
        [
            # Simulation clock
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use Gazebo simulation time",
            ),
            # TurtleBot3 model
            DeclareLaunchArgument(
                "model",
                default_value="waffle_pi",
                description="TurtleBot3 model",
            ),
            # Gazebo world
            DeclareLaunchArgument(
                "world",
                default_value=os.path.join(
                    turtlebot3_gazebo_dir,
                    "worlds",
                    "turtlebot3_world.world",
                ),
                description="Full path to Gazebo .world file",
            ),
            # Robot starting position
            DeclareLaunchArgument(
                "x_pose",
                default_value="-2.0",
                description="Initial TurtleBot3 X position",
            ),
            DeclareLaunchArgument(
                "y_pose",
                default_value="-0.5",
                description="Initial TurtleBot3 Y position",
            ),
            # TurtleBot3 launch files use this environment variable
            SetEnvironmentVariable(
                "TURTLEBOT3_MODEL",
                model,
            ),
            # Simulation
            gzserver,
            gzclient,
            # Robot
            robot_state_publisher,
            spawn_turtlebot,
            # Mapping / navigation
            slam,
            nav2,
            # Visualisation / exploration
            rviz,
            explorer,
        ]
    )
