from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    fusion_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("nlink_parser"), "launch", "tofsensem_fusion.launch.py"])
        )
    )

    return LaunchDescription([
        Node(
            package="nlink_parser",
            executable="tofsensem",
            name="tofsensem0",
            output="screen",
            parameters=[{"port_name": "/dev/ttyCH343USB0", "baud_rate": 921600}],
        ),
        fusion_launch,
    ])
