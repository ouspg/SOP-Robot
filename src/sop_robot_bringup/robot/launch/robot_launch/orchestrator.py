# Copyright 2020 ROS2-Control Development Team (2020)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import sys
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

_LAUNCH_DIR = Path(__file__).resolve().parents[1]
for _candidate in (_LAUNCH_DIR, _LAUNCH_DIR.parent):
    if str(_candidate) not in sys.path:
        sys.path.insert(0, str(_candidate))

from robot_launch.arguments import launch_arguments  # noqa: E402

from controllers.launch_utils import (  # noqa: E402
    STACK_CONFIG,
    cpu_affinity_prefix,
    create_dynamixel_config_file,
    load_stack_config,
    parse_bool,
    parse_controller_argument,
    resolve_launch_value,
    resolve_project_path,
)


def build_launch(context) -> list[object]:
    stack_config = load_stack_config(LaunchConfiguration("stack_config_path").perform(context))
    common_config = stack_config.get("common", {})
    raw_use_fake = LaunchConfiguration("use_fake").perform(context)
    raw_use_fake_hardware = LaunchConfiguration("use_fake_hardware").perform(context)
    if raw_use_fake_hardware != "":
        fake_requested = parse_bool(raw_use_fake_hardware)
    else:
        fake_requested = parse_bool(raw_use_fake)
    mode = "fake" if fake_requested else "real"
    mode_config = stack_config.get(mode, {})
    common_cpu_affinity = common_config.get("cpu_affinity", {})
    if not isinstance(common_cpu_affinity, dict):
        common_cpu_affinity = {}
    mode_cpu_affinity = mode_config.get("cpu_affinity", {})
    if not isinstance(mode_cpu_affinity, dict):
        mode_cpu_affinity = {}

    robot_parts = str(
        resolve_launch_value(
            LaunchConfiguration("robot_parts").perform(context),
            common_config.get("robot_parts"),
            "all",
        )
    )
    use_fake_hardware = parse_bool(
        resolve_launch_value(raw_use_fake_hardware, fake_requested, False)
    )
    use_rviz = parse_bool(
        resolve_launch_value(
            LaunchConfiguration("use_rviz").perform(context),
            mode_config.get("use_rviz"),
            False,
        )
    )
    enabled_controllers = str(
        resolve_launch_value(
            LaunchConfiguration("enabled_controllers").perform(context),
            mode_config.get("enabled_controllers"),
            "",
        )
    )
    camera_source = str(
        resolve_launch_value(
            LaunchConfiguration("camera_source").perform(context),
            common_config.get("camera_source"),
            "0",
        )
    )
    face_tracker_max_processing_fps = str(
        resolve_launch_value(
            LaunchConfiguration("face_tracker_max_processing_fps").perform(context),
            common_config.get("face_tracker_max_processing_fps"),
            "0",
        )
    )
    face_tracker_processing_width = str(
        resolve_launch_value(
            LaunchConfiguration("face_tracker_processing_width").perform(context),
            common_config.get("face_tracker_processing_width"),
            "640",
        )
    )
    face_image_publish_every_n_frames = str(
        resolve_launch_value(
            LaunchConfiguration("face_image_publish_every_n_frames").perform(context),
            common_config.get("face_image_publish_every_n_frames"),
            "2",
        )
    )
    face_image_publish_fps = str(
        resolve_launch_value(
            LaunchConfiguration("face_image_publish_fps").perform(context),
            common_config.get("face_image_publish_fps"),
            "0.0",
        )
    )
    face_recognition_model = str(
        resolve_launch_value(
            LaunchConfiguration("face_recognition_model").perform(context),
            common_config.get("face_recognition_model"),
            "SFace",
        )
    )
    face_detection_model = str(
        resolve_launch_value(
            LaunchConfiguration("face_detection_model").perform(context),
            common_config.get("face_detection_model"),
            "yunet",
        )
    )
    face_tracker_prefer_gpu = str(
        resolve_launch_value(
            LaunchConfiguration("face_tracker_prefer_gpu").perform(context),
            common_config.get("face_tracker_prefer_gpu"),
            "false",
        )
    )
    gpu_face_recognition_model = str(
        resolve_launch_value(
            LaunchConfiguration("gpu_face_recognition_model").perform(context),
            common_config.get("gpu_face_recognition_model"),
            "SFace",
        )
    )
    gpu_face_detection_model = str(
        resolve_launch_value(
            LaunchConfiguration("gpu_face_detection_model").perform(context),
            common_config.get("gpu_face_detection_model"),
            "yolov8n",
        )
    )
    face_detection_confidence = str(
        resolve_launch_value(
            LaunchConfiguration("face_detection_confidence").perform(context),
            common_config.get("face_detection_confidence"),
            "0.35",
        )
    )
    face_detection_imgsz = str(
        resolve_launch_value(
            LaunchConfiguration("face_detection_imgsz").perform(context),
            common_config.get("face_detection_imgsz"),
            "640",
        )
    )
    no_face_detection_interval_frames = str(
        resolve_launch_value(
            LaunchConfiguration("no_face_detection_interval_frames").perform(context),
            common_config.get("no_face_detection_interval_frames"),
            "1",
        )
    )
    no_face_detection_warmup_frames = str(
        resolve_launch_value(
            LaunchConfiguration("no_face_detection_warmup_frames").perform(context),
            common_config.get("no_face_detection_warmup_frames"),
            "3",
        )
    )
    face_correlation_tracking = str(
        resolve_launch_value(
            LaunchConfiguration("face_correlation_tracking").perform(context),
            common_config.get("face_correlation_tracking"),
            "false",
        )
    )
    face_detection_interval_frames = str(
        resolve_launch_value(
            LaunchConfiguration("face_detection_interval_frames").perform(context),
            common_config.get("face_detection_interval_frames"),
            "1",
        )
    )
    face_tracker_async_processing = str(
        resolve_launch_value(
            LaunchConfiguration("face_tracker_async_processing").perform(context),
            common_config.get("face_tracker_async_processing"),
            "true",
        )
    )
    face_tracker_slow_frame_warning_seconds = str(
        resolve_launch_value(
            LaunchConfiguration("face_tracker_slow_frame_warning_seconds").perform(context),
            common_config.get("face_tracker_slow_frame_warning_seconds"),
            "1.0",
        )
    )
    face_tracker_processing_status_log_interval_seconds = str(
        resolve_launch_value(
            LaunchConfiguration("face_tracker_processing_status_log_interval_seconds").perform(
                context
            ),
            common_config.get("face_tracker_processing_status_log_interval_seconds"),
            "5.0",
        )
    )
    face_image_max_width = str(
        resolve_launch_value(
            LaunchConfiguration("face_image_max_width").perform(context),
            common_config.get("face_image_max_width"),
            "640",
        )
    )
    identity_store_max_identities = str(
        resolve_launch_value(
            LaunchConfiguration("identity_store_max_identities").perform(context),
            common_config.get("identity_store_max_identities"),
            "8",
        )
    )
    identity_store_ttl_seconds = str(
        resolve_launch_value(
            LaunchConfiguration("identity_store_ttl_seconds").perform(context),
            common_config.get("identity_store_ttl_seconds"),
            "300.0",
        )
    )
    webcam_width = str(
        resolve_launch_value(
            LaunchConfiguration("webcam_width").perform(context),
            common_config.get("webcam_width"),
            "640",
        )
    )
    webcam_height = str(
        resolve_launch_value(
            LaunchConfiguration("webcam_height").perform(context),
            common_config.get("webcam_height"),
            "480",
        )
    )
    webcam_fps = str(
        resolve_launch_value(
            LaunchConfiguration("webcam_fps").perform(context),
            common_config.get("webcam_fps"),
            "15",
        )
    )
    webcam_capture_buffer_size = str(
        resolve_launch_value(
            LaunchConfiguration("webcam_capture_buffer_size").perform(context),
            common_config.get("webcam_capture_buffer_size"),
            "1",
        )
    )
    webcam_read_warning_seconds = str(
        resolve_launch_value(
            LaunchConfiguration("webcam_read_warning_seconds").perform(context),
            common_config.get("webcam_read_warning_seconds"),
            "1.0",
        )
    )
    webcam_reconnect_cooldown_seconds = str(
        resolve_launch_value(
            LaunchConfiguration("webcam_reconnect_cooldown_seconds").perform(context),
            common_config.get("webcam_reconnect_cooldown_seconds"),
            "2.0",
        )
    )
    webcam_status_log_interval_seconds = str(
        resolve_launch_value(
            LaunchConfiguration("webcam_status_log_interval_seconds").perform(context),
            common_config.get("webcam_status_log_interval_seconds"),
            "5.0",
        )
    )
    webcam_low_latency_stream = str(
        resolve_launch_value(
            LaunchConfiguration("webcam_low_latency_stream").perform(context),
            common_config.get("webcam_low_latency_stream"),
            "true",
        )
    )
    webcam_async_capture = str(
        resolve_launch_value(
            LaunchConfiguration("webcam_async_capture").perform(context),
            common_config.get("webcam_async_capture"),
            "true",
        )
    )
    enable_face_tracker = parse_bool(
        resolve_launch_value(
            LaunchConfiguration("enable_face_tracker").perform(context),
            common_config.get("enable_face_tracker"),
            True,
        )
    )
    enable_face_tracker_movement = parse_bool(
        resolve_launch_value(
            LaunchConfiguration("enable_face_tracker_movement").perform(context),
            common_config.get("enable_face_tracker_movement"),
            True,
        )
    )
    enable_voice_stack = parse_bool(
        resolve_launch_value(
            LaunchConfiguration("enable_voice_stack").perform(context),
            common_config.get("enable_voice_stack"),
            True,
        )
    )
    enable_chatbot_ui = parse_bool(
        resolve_launch_value(
            LaunchConfiguration("enable_chatbot_ui").perform(context),
            common_config.get("enable_chatbot_ui"),
            False,
        )
    )
    enable_calibration_ui = parse_bool(
        resolve_launch_value(
            LaunchConfiguration("enable_calibration_ui").perform(context),
            common_config.get("enable_calibration_ui"),
            False,
        )
    )
    enable_ros2graph_ui = parse_bool(
        resolve_launch_value(
            LaunchConfiguration("enable_ros2graph_ui").perform(context),
            common_config.get("enable_ros2graph_ui"),
            False,
        )
    )
    enable_full_demo = parse_bool(
        resolve_launch_value(
            LaunchConfiguration("enable_full_demo").perform(context),
            common_config.get("enable_full_demo"),
            True,
        )
    )
    enable_hand_gestures = parse_bool(
        resolve_launch_value(
            LaunchConfiguration("enable_hand_gestures").perform(context),
            common_config.get("enable_hand_gestures"),
            False,
        )
    )
    enable_shoulder_controller = parse_bool(
        resolve_launch_value(
            LaunchConfiguration("enable_shoulder_controller").perform(context),
            common_config.get("enable_shoulder_controller"),
            True,
        )
    )
    enable_jaw_movement = parse_bool(
        resolve_launch_value(
            LaunchConfiguration("enable_jaw_movement").perform(context),
            common_config.get("enable_jaw_movement"),
            True,
        )
    )
    face_tracker_functionality = str(
        resolve_launch_value(
            LaunchConfiguration("face_tracker_functionality").perform(context),
            common_config.get("face_tracker_functionality"),
            "full",
        )
    )
    voice_config_path = str(
        resolve_launch_value(
            LaunchConfiguration("voice_config_path").perform(context),
            common_config.get("voice_config_path"),
            "config/voice_chatbot.json",
        )
    )
    joint_calibration_path = resolve_project_path(
        str(
            resolve_launch_value(
                LaunchConfiguration("joint_calibration_path").perform(context),
                common_config.get(
                    "joint_calibration_path",
                    common_config.get("shoulder_calibration_path"),
                ),
                "config/shoulder_calibration.yaml",
            )
        )
    )
    load_voice_config = parse_bool(
        resolve_launch_value(
            LaunchConfiguration("load_voice_config").perform(context),
            common_config.get("load_voice_config"),
            True,
        )
    )
    asr_test_mode = parse_bool(
        resolve_launch_value(
            LaunchConfiguration("asr_test_mode").perform(context),
            common_config.get("asr_test_mode"),
            False,
        )
    )
    ros2graph_web_host = str(
        resolve_launch_value(
            LaunchConfiguration("ros2graph_web_host").perform(context),
            common_config.get("ros2graph_web_host"),
            "127.0.0.1",
        )
    )
    ros2graph_web_port = str(
        resolve_launch_value(
            LaunchConfiguration("ros2graph_web_port").perform(context),
            common_config.get("ros2graph_web_port"),
            "8734",
        )
    )
    ros2graph_update_interval = str(
        resolve_launch_value(
            LaunchConfiguration("ros2graph_update_interval").perform(context),
            common_config.get("ros2graph_update_interval"),
            "1.0",
        )
    )
    runtime_log_level = str(
        resolve_launch_value(
            LaunchConfiguration("runtime_log_level").perform(context),
            common_config.get("runtime_log_level"),
            "warn",
        )
    )

    def resolve_cpu_cores(name: str) -> str:
        return str(
            resolve_launch_value(
                LaunchConfiguration(f"{name}_cpu_cores").perform(context),
                mode_cpu_affinity.get(name, common_cpu_affinity.get(name)),
                "",
            )
        )

    ros2_control_cpu_cores = resolve_cpu_cores("ros2_control")
    rviz_cpu_cores = resolve_cpu_cores("rviz")
    face_tracker_cpu_cores = resolve_cpu_cores("face_tracker")
    webcam_cpu_cores = resolve_cpu_cores("webcam")
    asr_cpu_cores = resolve_cpu_cores("asr")
    llm_cpu_cores = resolve_cpu_cores("llm")
    tts_cpu_cores = resolve_cpu_cores("tts")
    ros_arguments = ["--ros-args", "--log-level", runtime_log_level]

    controllers_to_start = parse_controller_argument(enabled_controllers)
    dynamixel_config_file = create_dynamixel_config_file(robot_parts)

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("inmoov_description"),
                    "robots",
                    "inmoov.urdf.xacro",
                ]
            ),
            " dynamixel_config_file:=",
            dynamixel_config_file,
            f" use_fake_hardware:={'true' if use_fake_hardware else 'false'}",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    controller = os.path.join(
        get_package_share_directory("robot"),
        "controllers",
        "robot.yaml",
    )
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("inmoov_description"), "config", "inmoov.rviz"]
    )

    nodes: list[object] = [
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[robot_description, controller],
            arguments=ros_arguments,
            output={"stdout": "screen", "stderr": "screen"},
            prefix=cpu_affinity_prefix(ros2_control_cpu_cores),
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster"],
            output="screen",
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[robot_description],
        ),
    ]

    nodes.extend(
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[controller_name, "-c", "/controller_manager"],
            output="screen",
        )
        for controller_name in controllers_to_start
    )

    if use_rviz:
        nodes.append(
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config_file],
                output="screen",
                prefix=cpu_affinity_prefix(rviz_cpu_cores),
            )
        )

    if enable_face_tracker:
        nodes.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("face_tracker"),
                        "launch",
                        "face_tracker.test.launch.py",
                    )
                ),
                launch_arguments={
                    "camera_source": camera_source,
                    "face_tracker_cpu_cores": face_tracker_cpu_cores,
                    "webcam_cpu_cores": webcam_cpu_cores,
                    "face_tracker_max_processing_fps": face_tracker_max_processing_fps,
                    "face_tracker_processing_width": face_tracker_processing_width,
                    "face_image_publish_every_n_frames": face_image_publish_every_n_frames,
                    "face_image_publish_fps": face_image_publish_fps,
                    "face_recognition_model": face_recognition_model,
                    "face_detection_model": face_detection_model,
                    "face_tracker_prefer_gpu": face_tracker_prefer_gpu,
                    "gpu_face_recognition_model": gpu_face_recognition_model,
                    "gpu_face_detection_model": gpu_face_detection_model,
                    "face_detection_confidence": face_detection_confidence,
                    "face_detection_imgsz": face_detection_imgsz,
                    "no_face_detection_interval_frames": no_face_detection_interval_frames,
                    "no_face_detection_warmup_frames": no_face_detection_warmup_frames,
                    "face_correlation_tracking": face_correlation_tracking,
                    "face_detection_interval_frames": face_detection_interval_frames,
                    "face_tracker_async_processing": face_tracker_async_processing,
                    "face_tracker_slow_frame_warning_seconds": (
                        face_tracker_slow_frame_warning_seconds
                    ),
                    "face_tracker_processing_status_log_interval_seconds": (
                        face_tracker_processing_status_log_interval_seconds
                    ),
                    "face_image_max_width": face_image_max_width,
                    "identity_store_max_identities": identity_store_max_identities,
                    "identity_store_ttl_seconds": identity_store_ttl_seconds,
                    "webcam_width": webcam_width,
                    "webcam_height": webcam_height,
                    "webcam_fps": webcam_fps,
                    "webcam_capture_buffer_size": webcam_capture_buffer_size,
                    "webcam_read_warning_seconds": webcam_read_warning_seconds,
                    "webcam_reconnect_cooldown_seconds": webcam_reconnect_cooldown_seconds,
                    "webcam_status_log_interval_seconds": webcam_status_log_interval_seconds,
                    "webcam_low_latency_stream": webcam_low_latency_stream,
                    "webcam_async_capture": webcam_async_capture,
                    "log_level": runtime_log_level,
                }.items(),
            )
        )

    if enable_face_tracker_movement:
        nodes.append(
            Node(
                package="face_tracker_movement",
                executable="face_tracker_movement_node",
                name="face_tracker_movement",
                arguments=ros_arguments,
                output="screen",
                parameters=[
                    {
                        "functionality": face_tracker_functionality,
                        "simulation": use_fake_hardware,
                        "joint_calibration_path": joint_calibration_path,
                        "calibration_mode": enable_calibration_ui,
                        "joint_calibration_use_max_limits": enable_calibration_ui,
                    }
                ],
            )
        )

    if enable_voice_stack:
        nodes.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("chatbot_app"),
                        "launch",
                        "voice_stack.launch.py",
                    )
                ),
                launch_arguments={
                    "config_path": voice_config_path,
                    "load_config_file": "true" if load_voice_config else "false",
                    "asr_cpu_cores": asr_cpu_cores,
                    "llm_cpu_cores": llm_cpu_cores,
                    "tts_cpu_cores": tts_cpu_cores,
                    "asr_test_mode": "true" if asr_test_mode else "false",
                    "log_level": runtime_log_level,
                }.items(),
            )
        )

    if enable_chatbot_ui:
        nodes.append(
            Node(
                package="chatbot_app",
                executable="chatbot_app_unified",
                name="chatbot_app",
                output="screen",
                additional_env={
                    "SOP_ROBOT_JOINT_CALIBRATION_PATH": joint_calibration_path,
                    "SOP_ROBOT_SHOULDER_CALIBRATION_PATH": joint_calibration_path,
                },
            )
        )

    if enable_calibration_ui:
        nodes.append(
            Node(
                package="chatbot_app",
                executable="chatbot_app_calibration",
                name="chatbot_app_calibration",
                output="screen",
                additional_env={
                    "SOP_ROBOT_JOINT_CALIBRATION_PATH": joint_calibration_path,
                    "SOP_ROBOT_SHOULDER_CALIBRATION_PATH": joint_calibration_path,
                },
            )
        )

    if enable_ros2graph_ui:
        nodes.append(
            Node(
                package="ros2graph_explorer",
                executable="ros2graph_explorer",
                name="ros2graph_explorer",
                arguments=ros_arguments,
                output="screen",
                parameters=[
                    {
                        "output_format": "none",
                        "update_interval": float(ros2graph_update_interval),
                        "web_enable": True,
                        "web_host": ros2graph_web_host,
                        "web_port": int(ros2graph_web_port),
                    }
                ],
            )
        )

    if enable_jaw_movement:
        nodes.append(
            Node(
                package="jaw_movement",
                executable="jaw_movement_node",
                name="jaw_movement",
                arguments=ros_arguments,
                output="screen",
            )
        )

    if enable_hand_gestures:
        nodes.append(
            Node(
                package="hand_gestures",
                executable="hand_gestures_node",
                name="hand_gestures",
                arguments=ros_arguments,
                output="screen",
                parameters=[
                    {
                        "joint_calibration_path": joint_calibration_path,
                        "joint_calibration_use_max_limits": enable_calibration_ui,
                    }
                ],
            )
        )

    if enable_shoulder_controller:
        nodes.append(
            Node(
                package="shoulder_controller",
                executable="shoulder_controller_node",
                name="shoulder_controller_bridge",
                arguments=ros_arguments,
                output="screen",
                parameters=[
                    {
                        "use_fake": use_fake_hardware,
                        "shoulder_calibration_path": joint_calibration_path,
                        "shoulder_calibration_use_max_limits": enable_calibration_ui,
                    }
                ],
            )
        )

    if enable_full_demo:
        nodes.append(
            Node(
                package="full_demo",
                executable="full_demo_node",
                name="full_demo",
                arguments=ros_arguments,
                output="screen",
            )
        )

    return nodes


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            *launch_arguments(str(STACK_CONFIG)),
            OpaqueFunction(function=build_launch),
        ]
    )
