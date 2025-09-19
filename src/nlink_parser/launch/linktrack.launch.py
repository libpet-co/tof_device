from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        Node(
            package="nlink_parser",
            executable="linktrack",
            name="linktrack0",
            output="screen",
            parameters=[
                {"port_name": "/dev/ttyUSB0", "baud_rate": 921600},
            ],
        )
    ])
