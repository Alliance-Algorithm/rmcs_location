from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription


def generate_launch_description():
    rmcs_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("rmcs_slam"), "/launch", "/slam.py"]
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
        parameters=[[FindPackageShare("rmcs_navigation"), "/config", "/config.yaml"]],
        output="screen",
    )

    battlefield = ExecuteProcess(
        cmd=["ros2", "bag", "play", "/workspaces/sentry/ignore/bag/battlefield_0"],
        output="screen",
    )

    return LaunchDescription([rmcs_slam, rmcs_map, rmcs_navigation, battlefield])
