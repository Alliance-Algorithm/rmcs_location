from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription


def generate_launch_description():
    livox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("livox_ros_driver2"), "/launch", "/msg_MID360_launch.py"]
        )
    )

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

    rmcs_navigation = Node(
        package="rmcs_navigation",
        executable="rmcs_navigation_exe",
        parameters=[[FindPackageShare("rmcs_navigation"), "/config", "/config.yaml"]],
        output="screen",
    )

    launch = LaunchDescription()
    launch.add_action(livox)
    launch.add_action(rmcs_slam)
    launch.add_action(rmcs_map)
    launch.add_action(rmcs_navigation)

    return launch
