"""Launch the split voice stack without the GUI."""

import re

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _taskset_prefix(raw_value: str) -> str | None:
    cpu_cores = raw_value.strip().replace(" ", "")
    if not cpu_cores:
        return None
    cpu_item = r"\d+(?:-\d+)?"
    if not re.fullmatch(rf"{cpu_item}(?:,{cpu_item})*", cpu_cores):
        raise ValueError(
            f"Invalid CPU affinity value {raw_value!r}. "
            "Use a Linux CPU list such as '4', '2,3', or '2-5'."
        )
    return f"taskset -c {cpu_cores}"


def build_launch(context) -> list[Node]:
    config_path = LaunchConfiguration("config_path")
    load_config_file = LaunchConfiguration("load_config_file")
    asr_cpu_cores = LaunchConfiguration("asr_cpu_cores").perform(context)
    llm_cpu_cores = LaunchConfiguration("llm_cpu_cores").perform(context)
    tts_cpu_cores = LaunchConfiguration("tts_cpu_cores").perform(context)
    asr_test_mode = LaunchConfiguration("asr_test_mode")
    log_level = LaunchConfiguration("log_level")
    ros_arguments = ["--ros-args", "--log-level", log_level]

    shared_params = [
        {
            "config_path": ParameterValue(config_path, value_type=str),
            "load_config_file": ParameterValue(load_config_file, value_type=bool),
        }
    ]

    return [
        Node(
            package="asr_package",
            executable="asr_node",
            name="asr_package",
            arguments=ros_arguments,
            output="screen",
            parameters=[
                *shared_params,
                {"test_mode": ParameterValue(asr_test_mode, value_type=bool)},
            ],
            prefix=_taskset_prefix(asr_cpu_cores),
        ),
        Node(
            package="llm_package",
            executable="llm_node",
            name="llm_package",
            arguments=ros_arguments,
            output="screen",
            parameters=shared_params,
            prefix=_taskset_prefix(llm_cpu_cores),
        ),
        Node(
            package="tts_package",
            executable="tts_node",
            name="tts_package",
            arguments=ros_arguments,
            output="screen",
            parameters=shared_params,
            prefix=_taskset_prefix(tts_cpu_cores),
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("config_path", default_value="config/voice_chatbot.json"),
            DeclareLaunchArgument("load_config_file", default_value="true"),
            DeclareLaunchArgument(
                "asr_cpu_cores",
                default_value="",
                description="CPU affinity for asr_node as a Linux taskset CPU list.",
            ),
            DeclareLaunchArgument(
                "llm_cpu_cores",
                default_value="",
                description="CPU affinity for llm_node as a Linux taskset CPU list.",
            ),
            DeclareLaunchArgument(
                "tts_cpu_cores",
                default_value="",
                description="CPU affinity for tts_node as a Linux taskset CPU list.",
            ),
            DeclareLaunchArgument(
                "asr_test_mode",
                default_value="false",
                description="Start ASR without microphone capture for automated graph tests.",
            ),
            DeclareLaunchArgument(
                "log_level",
                default_value="warn",
                description="ROS log level for voice stack nodes.",
            ),
            OpaqueFunction(function=build_launch),
        ]
    )
