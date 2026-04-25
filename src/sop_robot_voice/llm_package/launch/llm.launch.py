"""Launch file for the LLM node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    config_path = LaunchConfiguration("config_path")
    load_config_file = LaunchConfiguration("load_config_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument("config_path", default_value="config/voice_chatbot.json"),
            DeclareLaunchArgument("load_config_file", default_value="true"),
            Node(
                package="llm_package",
                executable="llm_node",
                name="llm_package",
                output="screen",
                parameters=[
                    {
                        "config_path": ParameterValue(config_path, value_type=str),
                        "load_config_file": ParameterValue(
                            load_config_file,
                            value_type=bool,
                        ),
                    }
                ],
            ),
        ]
    )
