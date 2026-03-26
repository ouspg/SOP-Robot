"""
ROS 2 launch file for the split voice chatbot nodes.

Launches three nodes in the ``/voice_chatbot`` namespace:

- ``voice_stt`` – microphone capture, VAD, Whisper STT
- ``voice_llm`` – LLM chat inference (LLaMA/GGUF)
- ``voice_tts`` – Coqui TTS synthesis + audio playback

All three share the same ``config_path`` and ``load_config_file``
parameters so they read the same ``config.json``.

Usage::

    pixi run ros-launch
    # or: ros2 launch voice_chatbot_ros voice_chatbot.launch.py
"""

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from launch import LaunchDescription


def generate_launch_description() -> LaunchDescription:
    """Build the launch description with shared parameters."""
    config_path = LaunchConfiguration("config_path")
    load_config_file = LaunchConfiguration("load_config_file")

    shared_params = [
        {
            "config_path": ParameterValue(config_path, value_type=str),
            "load_config_file": ParameterValue(load_config_file, value_type=bool),
        }
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument("config_path", default_value="config.json"),
            DeclareLaunchArgument("load_config_file", default_value="true"),
            # STT node: microphone capture, VAD, speech-to-text
            Node(
                package="voice_chatbot_ros",
                executable="voice_stt_node",
                name="voice_stt",
                namespace="voice_chatbot",
                output="screen",
                parameters=shared_params,
            ),
            # LLM node: chat inference
            Node(
                package="voice_chatbot_ros",
                executable="voice_llm_node",
                name="voice_llm",
                namespace="voice_chatbot",
                output="screen",
                parameters=shared_params,
            ),
            # TTS node: text-to-speech synthesis and audio playback
            Node(
                package="voice_chatbot_ros",
                executable="voice_tts_node",
                name="voice_tts",
                namespace="voice_chatbot",
                output="screen",
                parameters=shared_params,
            ),
        ]
    )
