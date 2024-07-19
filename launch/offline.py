from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    slam = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([FindPackageShare("rmcs_slam"), "/launch", "/slam.py"])
      )
    
    navigation = Node(
        package="rmcs_navigation",
        executable="rmcs_navigation_exe",
        parameters=[[FindPackageShare("rmcs_navigation"), "/config", "/config.yaml"]],
        output="screen"
    )

    return LaunchDescription([
        slam,
        navigation
    ])