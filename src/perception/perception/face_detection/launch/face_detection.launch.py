from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="perception",
            executable="face_detection",
            name="face_detection_node",
            output="screen",
            parameters=[{
                "face_cascade_path": "",
                "predictor_path": "",
                "face_recognizer_model_path": "",
                "face_reference_images_path": "",
            }],
        ),
    ])
