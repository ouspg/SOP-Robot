from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    tts_package = Node(
        package='tts_package',
        namespace='tts_package',
        executable='service',
        name='service',
        output='both',
        remappings=[('jaw_topic', '/jaw_topic')],
        additional_env={
            'TF_CPP_MIN_LOG_LEVEL': '2',
            'TF_ENABLE_ONEDNN_OPTS': '0',
            'GLOG_minloglevel': '2',
        },
    )
    llm_package = Node(
        package='llm_package',
        namespace='llm_package',
        executable='llm_node',
        name='llm_node',
        output='both',
    )
    sst_package = Node(
        package='sst_package',
        namespace='sst_package',
        executable='sst_node',
        name='sst_node',
        output='both',
    )

    return LaunchDescription(
        [
            tts_package,
            llm_package,
            sst_package,
        ]
    )
