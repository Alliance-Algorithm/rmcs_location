from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription

use_battlefield_record = True
bag = "/workspaces/sentry/ignore/bag/2024-08-01-11-00"


def generate_launch_description():
    rmcs_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("rmcs_slam"), "/launch", "/launch.py"]
        )
    )

    rmcs_map = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("rmcs_map"), "/launch", "/launch.py"]
        )
    )

    rmcs_location = Node(
        package="rmcs_location",
        executable="rmcs_location_exe",
        parameters=[[FindPackageShare("rmcs_location"), "/config", "/config.yaml"]],
        output="screen",
    )

    # ros2 bag recorded in real battlefield
    battlefield = ExecuteProcess(
        cmd=["ros2", "bag", "play", bag],
        output="log",
    )

    launch = LaunchDescription()
    launch.add_action(rmcs_slam)
    launch.add_action(rmcs_map)
    launch.add_action(rmcs_location)

    if use_battlefield_record:
        launch.add_action(battlefield)

    return launch
