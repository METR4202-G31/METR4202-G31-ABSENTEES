import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    rviz_config = LaunchConfiguration("rviz_config")

    nav2_bringup_dir = get_package_share_directory("nav2_bringup")
    slam_toolbox_dir = get_package_share_directory("slam_toolbox")
    slam_setup_dir = get_package_share_directory("slam_setup")

    # Our own slam_toolbox settings: reads /scan_filled, max_laser_range 3.0,
    # map_update_interval 1.0. See slam_setup/config/slam_toolbox_params.yaml.
    slam_params = os.path.join(
        slam_setup_dir,
        "config",
        "slam_toolbox_params.yaml",
    )

    # Fills "no hit" LiDAR beams so SLAM clears free space in open areas.
    scan_filler = Node(
        package="slam_setup",
        executable="scan_filler_node",
        name="scan_filler",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

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
            "slam_params_file": slam_params,
        }.items(),
    )

    # Our own copy of the stock Nav2 params with a smaller robot_radius and
    # tighter inflation for small mazes. See config/nav2_params.yaml.
    nav2_params = os.path.join(
        get_package_share_directory("simple_explorer"),
        "config",
        "nav2_params.yaml",
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

    default_rviz_config = os.path.join(
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

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "rviz_config",
                default_value=default_rviz_config,
                description="Path to the RViz configuration file",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use Gazebo simulation time",
            ),
            scan_filler, slam, nav2, rviz, explorer,
        ]
    )
