from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="inmoov_i2",
            executable="i2e_webcam",
            name="i2e_webcam_node",
            output="screen",
        ),
    ])
