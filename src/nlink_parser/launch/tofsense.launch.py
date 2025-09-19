from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        Node(
            package="nlink_parser",
            executable="tofsense",
            name="tofsense0",
            output="screen",
            parameters=[
                {
                    "port_name": "/dev/ttyCH343USB0",
                    "baud_rate": 921600,
                    "inquire_mode": False,
                }
            ],
        )
    ])
