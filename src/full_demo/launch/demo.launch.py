from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="tts_package",
            executable="service"
        ),
        Node(
            package="speech_recognizer",
            executable="speech_recognizer_node",
            namespace="speech_recognizer"
        ),
        Node(
            package="qabot",
            executable="client",
            namespace="chatbot"
        ),
        
        Node(
            package="full_demo",
            executable="full_demo_node"
        )
    ])
"""
Node(
        package="opencv_cam",
        executable="opencv_cam_main",
        parameters=[
            {
                "index": 0,
            }
        ],
        ),
        Node(
            package="face_tracker",
            executable="face_tracker_node",
            namespace="face_tracker",
            parameters=[
            {
                "predictor": "shape_predictor_68_face_landmarks.dat",
                "image_topic": "/image_raw",
                "face_image_topic": "image_face",
                "face_topic": "faces",
                "lip_movement_detector": "1_32_False_True_0.25_lip_motion_net_model.h5",
            }
        ],
        ),
        """