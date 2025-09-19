from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    parameter_file = PathJoinSubstitution(
        [FindPackageShare("nlink_parser"), "config", "tofsensem_fusion.yaml"]
    )

    return LaunchDescription([
        Node(
            package="nlink_parser",
            executable="tofsensem_fusion_node.py",
            name="tofsensem_fusion",
            output="screen",
            parameters=[parameter_file, {"debug": False, "debug_port": 5678}],
        )
    ])
