import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Launch the complete non-simulation project stack."""
    use_sim_time = LaunchConfiguration("use_sim_time")
    rviz_config = LaunchConfiguration("rviz_config")

    simple_explorer_share = get_package_share_directory("simple_explorer")
    default_rviz_config = os.path.join(
        simple_explorer_share,
        "rviz",
        "full_project_stack.rviz",
    )

    application_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                simple_explorer_share,
                "launch",
                "explore.launch.py",
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "rviz_config": rviz_config,
        }.items(),
    )

    navigation_supervisor = Node(
        package="navigation_supervisor",
        executable="navigation_supervisor",
        name="navigation_supervisor",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    aruco_detector = Node(
        package="aruco_detector",
        executable="aruco_detector_node",
        name="aruco_detector",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    mission_control = Node(
        package="mission_control",
        executable="mission_control_node",
        name="mission_control",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation time instead of the robot clock",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=default_rviz_config,
                description="Path to the RViz configuration file",
            ),
            navigation_supervisor,
            aruco_detector,
            mission_control,
            application_stack,
        ]
    )
