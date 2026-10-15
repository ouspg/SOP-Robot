from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    sst_remote_node = Node(
        package='sst_package',
        executable='sst_remote_node',
        name='sst_remote_node',
        output='screen',
    )

    return LaunchDescription([sst_remote_node])
