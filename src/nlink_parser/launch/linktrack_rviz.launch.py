from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    map_frame = LaunchConfiguration("map_frame")

    return LaunchDescription([
        DeclareLaunchArgument("map_frame", default_value="linktrack_map"),
        Node(
            package="nlink_parser",
            executable="linktrack_rviz_converter",
            name="linktrack_rviz_converter0",
            output="screen",
            parameters=[{"map_frame": map_frame}],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base2linktrack",
            arguments=[
                "0",
                "0",
                "0",
                "0",
                "0",
                "0",
                "1",
                "map",
                map_frame,
            ],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz",
            arguments=[
                "-d",
                PathJoinSubstitution([FindPackageShare("nlink_parser"), "rviz", "linktrack.rviz"]),
            ],
        ),
    ])
