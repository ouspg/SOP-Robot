from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    tts_node = Node(
        package='tts_package',
        executable='service',
        name='service',
        output='both',
        additional_env={
            'TF_CPP_MIN_LOG_LEVEL': '2',
            'TF_ENABLE_ONEDNN_OPTS': '0',
            'GLOG_minloglevel': '2',
        },
    )
    llm_node = Node(
        package='llm_package',
        executable='llm_node',
        name='llm_node',
        output='both',
    )
    sst_node = Node(
        package='sst_package',
        executable='sst_node',
        name='sst_node',
        output='both',
    )

    return LaunchDescription(
        [
            tts_node,
            llm_node,
            sst_node,
        ]
    )
