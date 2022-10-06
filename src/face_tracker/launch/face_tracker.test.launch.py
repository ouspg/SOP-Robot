import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    camera_node = Node(
        package="opencv_cam",
        executable="opencv_cam_main",
        parameters=[
            {
                "index": 0,
            }
        ],
    )

    tracker_node = Node(
        package="face_tracker",
        executable="face_tracker_node",
        namespace="face_tracker",
        parameters=[
            {
                "predictor": "shape_predictor_68_face_landmarks.dat",
                "image_topic": "/image_raw",
                "face_image_topic": "image_face",
                "face_topic": "faces",
            }
        ],
    )

    return LaunchDescription([camera_node, tracker_node])
