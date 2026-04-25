"""Launch the split voice stack with the unified GUI."""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    voice_stack_launch = os.path.join(os.path.dirname(__file__), "voice_stack.launch.py")

    return LaunchDescription(
        [
            DeclareLaunchArgument("config_path", default_value="config/voice_chatbot.json"),
            DeclareLaunchArgument("load_config_file", default_value="true"),
            DeclareLaunchArgument("asr_cpu_cores", default_value=""),
            DeclareLaunchArgument("llm_cpu_cores", default_value=""),
            DeclareLaunchArgument("tts_cpu_cores", default_value=""),
            DeclareLaunchArgument("log_level", default_value="warn"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(voice_stack_launch),
                launch_arguments={
                    "config_path": LaunchConfiguration("config_path"),
                    "load_config_file": LaunchConfiguration("load_config_file"),
                    "asr_cpu_cores": LaunchConfiguration("asr_cpu_cores"),
                    "llm_cpu_cores": LaunchConfiguration("llm_cpu_cores"),
                    "tts_cpu_cores": LaunchConfiguration("tts_cpu_cores"),
                    "log_level": LaunchConfiguration("log_level"),
                }.items(),
            ),
            Node(
                package="chatbot_app",
                executable="chatbot_app_unified",
                name="chatbot_app",
                output="screen",
            ),
        ]
    )
