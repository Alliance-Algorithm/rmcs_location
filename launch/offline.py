from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params = [
        PathJoinSubstitution([FindPackageShare("rmcs_navigation"), "config", "config.yaml"])
    ]

    node = Node(
        package="rmcs_navigation",
        executable="rmcs_navigation_exe",
        parameters=params,
        output="screen"
    )

    # Assemble the launch description
    ld = LaunchDescription([node])

    return ld
