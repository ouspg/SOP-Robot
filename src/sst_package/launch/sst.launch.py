from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    sst_node = Node(
        package='sst_package',
        executable='sst_node',
        name='sst_node',
        output='screen',
    )

    return LaunchDescription([sst_node])
