"""Launch argument declarations for the SOP Robot bring-up launch."""

from launch.actions import DeclareLaunchArgument


def _launch_argument(name: str, description: str, default: str = "") -> DeclareLaunchArgument:
    return DeclareLaunchArgument(name, default_value=default, description=description)


def launch_arguments(
    stack_config_path: str, *, use_fake_default: str = "false"
) -> list[DeclareLaunchArgument]:
    return [
        _launch_argument(
            "stack_config_path",
            "Root robot stack config YAML.",
            default=stack_config_path,
        ),
        _launch_argument(
            "robot_parts",
            "Comma-separated robot sections to enable: head, arm, or all.",
        ),
        _launch_argument(
            "enabled_controllers",
            "Comma-separated controller list to spawn after ros2_control starts.",
        ),
        _launch_argument(
            "use_fake",
            "Use fake robot mode for launch-time nodes and ros2_control hardware.",
            default=use_fake_default,
        ),
        _launch_argument(
            "use_fake_hardware",
            "Use ros2_control fake hardware instead of the real Dynamixel interface.",
            default="",
        ),
        _launch_argument("use_rviz", "Start RViz with the inmoov_description config."),
        _launch_argument(
            "camera_source",
            "Camera device index or network stream URL for face tracking.",
        ),
        _launch_argument(
            "face_tracker_max_processing_fps",
            "Maximum face tracker processing FPS. Use 0 to process every received frame.",
        ),
        _launch_argument(
            "face_tracker_processing_width",
            "Width used for face analysis. Use 0 to process original frame size.",
        ),
        _launch_argument(
            "face_image_publish_every_n_frames",
            "Publish annotated face image every nth processed frame.",
        ),
        _launch_argument(
            "face_image_publish_fps",
            "Cap annotated face image publishing to this FPS. Use 0 for immediate latest-frame publishing.",
        ),
        _launch_argument(
            "face_recognition_model",
            "DeepFace model used for identity embeddings.",
        ),
        _launch_argument(
            "face_detection_model",
            "CPU fallback face detector backend.",
        ),
        _launch_argument(
            "face_tracker_prefer_gpu",
            "Prefer the optional PyTorch/YOLO detector instead of OpenCV YuNet.",
        ),
        _launch_argument(
            "gpu_face_recognition_model",
            "Recognition model used when GPU recognition is selected.",
        ),
        _launch_argument(
            "gpu_face_detection_model",
            "GPU-capable face detector; yolov8n uses local yolov8n-face.pt.",
        ),
        _launch_argument(
            "face_detection_confidence",
            "Minimum confidence for face detections.",
        ),
        _launch_argument(
            "face_detection_imgsz",
            "YOLO detector inference image size. Lower is faster.",
        ),
        _launch_argument(
            "no_face_detection_interval_frames",
            "When no face is present, run detector only every nth processed frame.",
        ),
        _launch_argument(
            "no_face_detection_warmup_frames",
            "Detector frames to run before entering no-face scan mode.",
        ),
        _launch_argument(
            "face_detection_interval_frames",
            "Run heavy face detection every nth processed frame; tracking runs between detections.",
        ),
        _launch_argument(
            "face_correlation_tracking",
            "Use dlib correlation tracking between detector passes.",
        ),
        _launch_argument(
            "face_tracker_async_processing",
            "Process camera frames on a latest-frame worker thread.",
        ),
        _launch_argument(
            "face_tracker_slow_frame_warning_seconds",
            "Warn when one face tracker processing pass exceeds this duration.",
        ),
        _launch_argument(
            "face_tracker_processing_status_log_interval_seconds",
            "Log face tracker timing breakdown at this interval. Use 0 to disable.",
        ),
        _launch_argument(
            "face_image_max_width",
            "Maximum width for the annotated debug image topic. Use 0 for original size.",
        ),
        _launch_argument(
            "identity_store_max_identities",
            "Maximum in-memory session identities. Use 0 for unlimited.",
        ),
        _launch_argument(
            "identity_store_ttl_seconds",
            "Expire unseen in-memory identities after this many seconds. Use 0 to disable.",
        ),
        _launch_argument(
            "webcam_width",
            "Published webcam frame width. Use 0 for source/default width.",
        ),
        _launch_argument(
            "webcam_height",
            "Published webcam frame height. Use 0 for source/default height.",
        ),
        _launch_argument(
            "webcam_fps",
            "Requested webcam FPS and publish timer rate. Use 0 for default.",
        ),
        _launch_argument(
            "webcam_capture_buffer_size",
            "Requested OpenCV capture buffer size. Ignored by some backends.",
        ),
        _launch_argument(
            "webcam_read_warning_seconds",
            "Warn when one webcam read blocks longer than this duration.",
        ),
        _launch_argument(
            "webcam_reconnect_cooldown_seconds",
            "Minimum seconds between webcam reconnect attempts.",
        ),
        _launch_argument(
            "webcam_status_log_interval_seconds",
            "Log measured webcam publish FPS at this interval. Use 0 to disable.",
        ),
        _launch_argument(
            "webcam_low_latency_stream",
            "Request low-latency OpenCV/FFmpeg options for network streams.",
        ),
        _launch_argument(
            "webcam_async_capture",
            "Continuously drain the camera source into a latest-frame buffer.",
        ),
        _launch_argument(
            "enable_face_tracker",
            "Launch the face tracker camera and detection nodes.",
        ),
        _launch_argument(
            "enable_face_tracker_movement",
            "Launch the head and eye motion follower driven by tracked faces.",
        ),
        _launch_argument(
            "enable_voice_stack",
            "Launch the ASR, LLM, and TTS voice stack nodes.",
        ),
        _launch_argument(
            "enable_chatbot_ui",
            "Launch the unified chatbot UI alongside the voice stack.",
        ),
        _launch_argument(
            "enable_calibration_ui",
            "Launch the dedicated joint calibration UI.",
        ),
        _launch_argument(
            "enable_ros2graph_ui",
            "Launch ros2graph_explorer's browser UI for the live ROS graph.",
        ),
        _launch_argument(
            "enable_full_demo",
            "Launch the high-level demo coordinator node.",
        ),
        _launch_argument(
            "enable_hand_gestures",
            "Launch the hand gesture action bridge.",
        ),
        _launch_argument(
            "enable_shoulder_controller",
            "Launch the unified shoulder controller bridge.",
        ),
        _launch_argument(
            "enable_jaw_movement",
            "Launch the jaw motion follower for speech playback.",
        ),
        _launch_argument(
            "face_tracker_functionality",
            "Select face-tracker movement mode: full, head, or eyes.",
        ),
        _launch_argument(
            "voice_config_path",
            "JSON config file passed to the split voice stack nodes.",
        ),
        _launch_argument(
            "joint_calibration_path",
            "YAML file storing saved centers for shoulders, head, eyes, and hands.",
        ),
        _launch_argument(
            "shoulder_calibration_path",
            "Deprecated alias for joint_calibration_path.",
        ),
        _launch_argument(
            "load_voice_config",
            "Load the JSON voice config file instead of in-code defaults.",
        ),
        _launch_argument(
            "asr_test_mode",
            "Start ASR without microphone capture for automated graph tests.",
        ),
        _launch_argument(
            "ros2graph_web_host",
            "Host/IP for ros2graph_explorer's web UI.",
        ),
        _launch_argument(
            "ros2graph_web_port",
            "Port for ros2graph_explorer's web UI.",
        ),
        _launch_argument(
            "ros2graph_update_interval",
            "Seconds between ros2graph_explorer graph refreshes.",
        ),
        _launch_argument(
            "runtime_log_level",
            "Default ROS log level for runtime nodes, for example warn or info.",
        ),
        _launch_argument(
            "ros2_control_cpu_cores",
            "CPU affinity for ros2_control_node as a Linux taskset CPU list.",
        ),
        _launch_argument(
            "rviz_cpu_cores",
            "CPU affinity for rviz2 as a Linux taskset CPU list.",
        ),
        _launch_argument(
            "face_tracker_cpu_cores",
            "CPU affinity for face_tracker_node as a Linux taskset CPU list.",
        ),
        _launch_argument(
            "webcam_cpu_cores",
            "CPU affinity for webcam_node as a Linux taskset CPU list.",
        ),
        _launch_argument(
            "asr_cpu_cores",
            "CPU affinity for asr_node as a Linux taskset CPU list.",
        ),
        _launch_argument(
            "llm_cpu_cores",
            "CPU affinity for llm_node as a Linux taskset CPU list.",
        ),
        _launch_argument(
            "tts_cpu_cores",
            "CPU affinity for tts_node as a Linux taskset CPU list.",
        ),
    ]
