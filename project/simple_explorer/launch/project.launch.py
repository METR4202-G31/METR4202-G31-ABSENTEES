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

    simple_explorer_share = get_package_share_directory("simple_explorer")

    application_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                simple_explorer_share,
                "launch",
                "explore.launch.py",
            )
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
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
            navigation_supervisor,
            aruco_detector,
            mission_control,
            application_stack,
        ]
    )
