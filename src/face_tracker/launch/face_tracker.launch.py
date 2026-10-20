from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    tracker_node = Node(
        package='face_tracker',
        executable='face_tracker_node',
        namespace='face_tracker',
        parameters=[
            {
                'lip_movement_detection': ParameterValue(
                    LaunchConfiguration('lip_movement_detection'), value_type=bool
                ),
                'face_recognition': False,
                'correlation_tracking': True,
                'cluster_similarity_threshold': 0.3,
                'subcluster_similarity_threshold': 0.2,
                'pair_similarity_maximum': 1.0,
                'face_recognition_model': 'SFace',
                'face_detection_model': 'yolov8',
                'inference_device': 'cuda:0',
                'detection_interval': 10,
                'detection_scale': 0.5,
                'publish_debug_image': True,
                'debug_image_scale': 0.5,
                'image_topic': '/image_raw',
                'face_image_topic': 'image_face',
                'face_topic': 'faces',
                'predictor': 'shape_predictor_68_face_landmarks.dat',
                'lip_movement_detector': ('1_32_False_True_0.25_lip_motion_net_model.h5'),
            }
        ],
    )
    webcam_node = Node(
        package='face_tracker',
        executable='webcam_node',
        parameters=[
            {
                'raw_image': '/image_raw',
                'index': 0,
                'width': 1920,
                'height': 1080,
                'fps': 60,
                'mjpg': True,
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument('lip_movement_detection', default_value='false'),
            tracker_node,
            webcam_node,
        ]
    )
