from launch.actions import ExecuteProcess
from launch import LaunchDescription
from datetime import datetime

year = datetime.now().year
mouth = datetime.now().month
day = datetime.now().day
hour = datetime.now().hour
minute = datetime.now().minute

bag = f"[record][{year}-{mouth}-{day}-{hour}-{minute}]"


def generate_launch_description():
    battlefield = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "record",
            "--output",
            bag,
            "/livox/lidar",
            "/livox/imu",
        ],
        output="screen",
    )

    launch = LaunchDescription()
    launch.add_action(battlefield)

    return launch
