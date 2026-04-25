import re

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from sop_robot_common.contracts import FACE_IMAGE_TOPIC, FACES_TOPIC, RAW_IMAGE_TOPIC


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


# index should be lowest index for the camera as usb devices usually have multiple
# index 0 is for first or only webcam like integrated laptop webcam for example
# for virtual machines it is easiest to port thought only the camera you are going to use and changing the camera from virtual machine settings is easier.
def build_launch(context):
    camera_source = LaunchConfiguration("camera_source")
    face_tracker_cpu_cores = LaunchConfiguration("face_tracker_cpu_cores").perform(context)
    webcam_cpu_cores = LaunchConfiguration("webcam_cpu_cores").perform(context)
    max_processing_fps = LaunchConfiguration("face_tracker_max_processing_fps")
    processing_width = LaunchConfiguration("face_tracker_processing_width")
    face_image_publish_every_n_frames = LaunchConfiguration("face_image_publish_every_n_frames")
    face_image_publish_fps = LaunchConfiguration("face_image_publish_fps")
    face_correlation_tracking = LaunchConfiguration("face_correlation_tracking")
    face_recognition_model = LaunchConfiguration("face_recognition_model")
    face_detection_model = LaunchConfiguration("face_detection_model")
    face_tracker_prefer_gpu = LaunchConfiguration("face_tracker_prefer_gpu")
    gpu_face_recognition_model = LaunchConfiguration("gpu_face_recognition_model")
    gpu_face_detection_model = LaunchConfiguration("gpu_face_detection_model")
    face_detection_confidence = LaunchConfiguration("face_detection_confidence")
    face_detection_imgsz = LaunchConfiguration("face_detection_imgsz")
    no_face_detection_interval_frames = LaunchConfiguration("no_face_detection_interval_frames")
    no_face_detection_warmup_frames = LaunchConfiguration("no_face_detection_warmup_frames")
    face_detection_interval_frames = LaunchConfiguration("face_detection_interval_frames")
    async_processing = LaunchConfiguration("face_tracker_async_processing")
    slow_frame_warning_seconds = LaunchConfiguration("face_tracker_slow_frame_warning_seconds")
    processing_status_log_interval_seconds = LaunchConfiguration(
        "face_tracker_processing_status_log_interval_seconds"
    )
    face_image_max_width = LaunchConfiguration("face_image_max_width")
    identity_store_max_identities = LaunchConfiguration("identity_store_max_identities")
    identity_store_ttl_seconds = LaunchConfiguration("identity_store_ttl_seconds")
    webcam_width = LaunchConfiguration("webcam_width")
    webcam_height = LaunchConfiguration("webcam_height")
    webcam_fps = LaunchConfiguration("webcam_fps")
    webcam_capture_buffer_size = LaunchConfiguration("webcam_capture_buffer_size")
    webcam_read_warning_seconds = LaunchConfiguration("webcam_read_warning_seconds")
    webcam_reconnect_cooldown_seconds = LaunchConfiguration("webcam_reconnect_cooldown_seconds")
    webcam_status_log_interval_seconds = LaunchConfiguration("webcam_status_log_interval_seconds")
    webcam_low_latency_stream = LaunchConfiguration("webcam_low_latency_stream")
    webcam_async_capture = LaunchConfiguration("webcam_async_capture")
    log_level = LaunchConfiguration("log_level")
    ros_arguments = ["--ros-args", "--log-level", log_level]

    tracker_node = Node(
        package="face_tracker",
        executable="face_tracker_node",
        namespace="face_tracker",
        prefix=_taskset_prefix(face_tracker_cpu_cores),
        arguments=ros_arguments,
        parameters=[
            {
                "lip_movement_detection": False,
                "face_recognition": True,
                "correlation_tracking": ParameterValue(
                    face_correlation_tracking,
                    value_type=bool,
                ),
                "cluster_similarity_threshold": 0.3,
                "subcluster_similarity_threshold": 0.2,
                "pair_similarity_maximum": 1.0,
                "face_recognition_model": face_recognition_model,
                "face_detection_model": face_detection_model,
                "prefer_gpu": ParameterValue(
                    face_tracker_prefer_gpu,
                    value_type=bool,
                ),
                "gpu_face_recognition_model": gpu_face_recognition_model,
                "gpu_face_detection_model": gpu_face_detection_model,
                "face_detection_confidence": ParameterValue(
                    face_detection_confidence,
                    value_type=float,
                ),
                "face_detection_imgsz": ParameterValue(
                    face_detection_imgsz,
                    value_type=int,
                ),
                "no_face_detection_interval_frames": ParameterValue(
                    no_face_detection_interval_frames,
                    value_type=int,
                ),
                "no_face_detection_warmup_frames": ParameterValue(
                    no_face_detection_warmup_frames,
                    value_type=int,
                ),
                "face_detection_interval_frames": ParameterValue(
                    face_detection_interval_frames,
                    value_type=int,
                ),
                "max_processing_fps": ParameterValue(max_processing_fps, value_type=float),
                "processing_width": ParameterValue(processing_width, value_type=int),
                "face_image_publish_every_n_frames": ParameterValue(
                    face_image_publish_every_n_frames,
                    value_type=int,
                ),
                "face_image_publish_fps": ParameterValue(
                    face_image_publish_fps,
                    value_type=float,
                ),
                "async_processing": ParameterValue(async_processing, value_type=bool),
                "slow_frame_warning_seconds": ParameterValue(
                    slow_frame_warning_seconds,
                    value_type=float,
                ),
                "processing_status_log_interval_seconds": ParameterValue(
                    processing_status_log_interval_seconds,
                    value_type=float,
                ),
                "face_image_max_width": ParameterValue(
                    face_image_max_width,
                    value_type=int,
                ),
                "identity_store_max_identities": ParameterValue(
                    identity_store_max_identities,
                    value_type=int,
                ),
                "identity_store_ttl_seconds": ParameterValue(
                    identity_store_ttl_seconds,
                    value_type=float,
                ),
                "image_topic": RAW_IMAGE_TOPIC,
                "face_image_topic": FACE_IMAGE_TOPIC,
                "face_topic": FACES_TOPIC,
                "predictor": "shape_predictor_68_face_landmarks.dat",
                "lip_movement_detector": "1_32_False_True_0.25_lip_motion_net_model.h5",
            }
        ],
    )

    webcam_node = Node(
        package="face_tracker",
        executable="webcam_node",
        # namespace="face_tracker",
        prefix=_taskset_prefix(webcam_cpu_cores),
        arguments=ros_arguments,
        parameters=[
            {
                "raw_image": RAW_IMAGE_TOPIC,
                "source": camera_source,
                "index": 0,
                "width": ParameterValue(webcam_width, value_type=int),
                "height": ParameterValue(webcam_height, value_type=int),
                "fps": ParameterValue(webcam_fps, value_type=int),
                "capture_buffer_size": ParameterValue(
                    webcam_capture_buffer_size,
                    value_type=int,
                ),
                "read_warning_seconds": ParameterValue(
                    webcam_read_warning_seconds,
                    value_type=float,
                ),
                "reconnect_cooldown_seconds": ParameterValue(
                    webcam_reconnect_cooldown_seconds,
                    value_type=float,
                ),
                "status_log_interval_seconds": ParameterValue(
                    webcam_status_log_interval_seconds,
                    value_type=float,
                ),
                "low_latency_stream": ParameterValue(
                    webcam_low_latency_stream,
                    value_type=bool,
                ),
                "async_capture": ParameterValue(
                    webcam_async_capture,
                    value_type=bool,
                ),
                "mjpg": True,
            }
        ],
    )

    return [tracker_node, webcam_node]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "camera_source",
                default_value="0",
                description="Camera device index or network stream URL (for example DroidCam MJPEG URL)",
            ),
            DeclareLaunchArgument(
                "face_tracker_cpu_cores",
                default_value="",
                description="CPU affinity for face_tracker_node as a Linux taskset CPU list.",
            ),
            DeclareLaunchArgument(
                "webcam_cpu_cores",
                default_value="",
                description="CPU affinity for webcam_node as a Linux taskset CPU list.",
            ),
            DeclareLaunchArgument(
                "face_tracker_max_processing_fps",
                default_value="0",
                description="Maximum face tracker processing FPS. Use 0 to process every received frame.",
            ),
            DeclareLaunchArgument(
                "face_tracker_processing_width",
                default_value="640",
                description="Width used for face analysis. Use 0 to process original frame size.",
            ),
            DeclareLaunchArgument(
                "face_image_publish_every_n_frames",
                default_value="1",
                description="Publish annotated face image every nth processed frame.",
            ),
            DeclareLaunchArgument(
                "face_image_publish_fps",
                default_value="0.0",
                description="Cap annotated face image publishing to this FPS. Use 0 for immediate latest-frame publishing.",
            ),
            DeclareLaunchArgument(
                "face_recognition_model",
                default_value="SFace",
                description="DeepFace model used for identity embeddings.",
            ),
            DeclareLaunchArgument(
                "face_detection_model",
                default_value="yunet",
                description="CPU fallback face detector backend.",
            ),
            DeclareLaunchArgument(
                "face_tracker_prefer_gpu",
                default_value="false",
                description="Prefer the optional PyTorch/YOLO detector instead of OpenCV YuNet.",
            ),
            DeclareLaunchArgument(
                "gpu_face_recognition_model",
                default_value="SFace",
                description="Recognition model used when GPU recognition is selected.",
            ),
            DeclareLaunchArgument(
                "gpu_face_detection_model",
                default_value="yolov8n",
                description="GPU-capable face detector; yolov8n uses local yolov8n-face.pt.",
            ),
            DeclareLaunchArgument(
                "face_detection_confidence",
                default_value="0.6",
                description="Minimum confidence for face detections.",
            ),
            DeclareLaunchArgument(
                "face_detection_imgsz",
                default_value="640",
                description="YOLO detector inference image size. Lower is faster.",
            ),
            DeclareLaunchArgument(
                "no_face_detection_interval_frames",
                default_value="3",
                description="When no face is present, run detector only every nth processed frame.",
            ),
            DeclareLaunchArgument(
                "no_face_detection_warmup_frames",
                default_value="3",
                description="Detector frames to run before entering no-face scan mode.",
            ),
            DeclareLaunchArgument(
                "face_detection_interval_frames",
                default_value="5",
                description="Run heavy face detection every nth processed frame; tracking runs between detections.",
            ),
            DeclareLaunchArgument(
                "face_correlation_tracking",
                default_value="true",
                description="Use dlib correlation tracking between detector passes.",
            ),
            DeclareLaunchArgument(
                "face_tracker_async_processing",
                default_value="true",
                description="Process camera frames on a latest-frame worker thread.",
            ),
            DeclareLaunchArgument(
                "face_tracker_slow_frame_warning_seconds",
                default_value="1.0",
                description="Warn when one face tracker processing pass exceeds this duration.",
            ),
            DeclareLaunchArgument(
                "face_tracker_processing_status_log_interval_seconds",
                default_value="5.0",
                description="Log face tracker timing breakdown at this interval. Use 0 to disable.",
            ),
            DeclareLaunchArgument(
                "face_image_max_width",
                default_value="640",
                description="Maximum width for the annotated debug image topic. Use 0 for original size.",
            ),
            DeclareLaunchArgument(
                "identity_store_max_identities",
                default_value="8",
                description="Maximum in-memory session identities. Use 0 for unlimited.",
            ),
            DeclareLaunchArgument(
                "identity_store_ttl_seconds",
                default_value="300.0",
                description="Expire unseen in-memory identities after this many seconds. Use 0 to disable.",
            ),
            DeclareLaunchArgument(
                "webcam_width",
                default_value="0",
                description="Published webcam frame width. Use 0 for source/default width.",
            ),
            DeclareLaunchArgument(
                "webcam_height",
                default_value="0",
                description="Published webcam frame height. Use 0 for source/default height.",
            ),
            DeclareLaunchArgument(
                "webcam_fps",
                default_value="0",
                description="Requested webcam FPS and publish timer rate. Use 0 for default.",
            ),
            DeclareLaunchArgument(
                "webcam_capture_buffer_size",
                default_value="1",
                description="Requested OpenCV capture buffer size. Ignored by some backends.",
            ),
            DeclareLaunchArgument(
                "webcam_read_warning_seconds",
                default_value="1.0",
                description="Warn when one webcam read blocks longer than this duration.",
            ),
            DeclareLaunchArgument(
                "webcam_reconnect_cooldown_seconds",
                default_value="2.0",
                description="Minimum seconds between webcam reconnect attempts.",
            ),
            DeclareLaunchArgument(
                "webcam_status_log_interval_seconds",
                default_value="5.0",
                description="Log measured webcam publish FPS at this interval. Use 0 to disable.",
            ),
            DeclareLaunchArgument(
                "webcam_low_latency_stream",
                default_value="true",
                description="Request low-latency OpenCV/FFmpeg options for network streams.",
            ),
            DeclareLaunchArgument(
                "webcam_async_capture",
                default_value="true",
                description="Continuously drain the camera source into a latest-frame buffer.",
            ),
            DeclareLaunchArgument(
                "log_level",
                default_value="warn",
                description="ROS log level for face tracker nodes.",
            ),
            OpaqueFunction(function=build_launch),
        ]
    )
