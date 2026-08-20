from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    tts_node = Node(
        package='tts_package',
        executable='tts_node',
        name='tts_node',
        output='screen',
        additional_env={
            'CUDA_VISIBLE_DEVICES': '-1',
            'TF_CPP_MIN_LOG_LEVEL': '2',
        },
    )
    llm_node = Node(
        package='llm_package',
        executable='llm_node',
        name='llm_node',
        output='screen',
    )
    sst_node = Node(
        package='sst_package',
        executable='sst_node',
        name='sst_node',
        output='screen',
    )

    return LaunchDescription(
        [
            tts_node,
            llm_node,
            sst_node,
        ]
    )
