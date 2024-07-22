from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription

use_battlefield_record = True


def generate_launch_description():
    rmcs_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("rmcs_slam"), "/launch", "/offline.py"]
        )
    )

    rmcs_map = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("rmcs_map"), "/launch", "/launch.py"]
        )
    )

    rmcs_navigation = Node(
        package="rmcs_navigation",
        executable="rmcs_navigation_exe",
        parameters=[[FindPackageShare("rmcs_navigation"), "/config", "/offline.yaml"]],
        output="screen",
    )

    # ros2 bag recorded in real battlefield
    battlefield = ExecuteProcess(
        cmd=["ros2", "bag", "play", "/workspaces/sentry/ignore/bag/battlefield_0"],
        output="screen",
    )

    launch = LaunchDescription()
    launch.add_action(rmcs_slam)
    launch.add_action(rmcs_map)
    launch.add_action(rmcs_navigation)

    if use_battlefield_record:
        launch.add_action(battlefield)

    return launch
