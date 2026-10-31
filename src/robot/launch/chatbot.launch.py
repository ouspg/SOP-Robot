from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    # Without full_demo routing between the namespaces, wire the nodes to each other:
    # SST -> LLM -> TTS, and the LLM and TTS pause the microphone while they work.
    standalone = LaunchConfiguration('standalone').perform(context).lower() == 'true'

    tts_package = Node(
        package='tts_package',
        namespace='tts_package',
        executable='service',
        name='service',
        output='both',
        remappings=[
            ('jaw_topic', '/jaw_topic'),
            *([('can_listen', '/sst_package/can_listen')] if standalone else []),
        ],
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
        remappings=[
            ('can_listen', '/sst_package/can_listen'),
            ('chatbot_response', '/tts_package/chatbot_response'),
        ]
        if standalone
        else [],
    )
    sst_package = Node(
        package='sst_package',
        namespace='sst_package',
        executable='sst_node',
        name='sst_node',
        output='both',
        remappings=[('recognized_speech', '/llm_package/recognized_speech')] if standalone else [],
    )

    return [tts_package, llm_package, sst_package]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'standalone',
                default_value='false',
                description='Connect SST, LLM and TTS directly instead of through full_demo',
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
