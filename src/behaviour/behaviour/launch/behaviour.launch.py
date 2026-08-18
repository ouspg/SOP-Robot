from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "facs_config",
            default_value="",
            description="Path to FACS YAML config file",
        ),
        Node(
            package="behaviour",
            executable="face_track",
            name="face_track_node",
            output="screen",
        ),
        Node(
            package="behaviour",
            executable="head_gesture",
            name="head_gesture_node",
            output="screen",
        ),
        Node(
            package="behaviour",
            executable="facs",
            name="facs_node",
            output="screen",
            parameters=[{
                "facs_config": LaunchConfiguration("facs_config"),
            }],
        ),
    ])
