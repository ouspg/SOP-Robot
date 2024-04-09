import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

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
                "lip_movement_detector": "1_32_False_True_0.25_lip_motion_net_model.h5",
            }
        ],
    )

    webcam_node = Node(
        package="face_tracker",
        executable="webcam_node",
        # namespace="face_tracker",
        parameters=[
            {
                "raw_image": "/image_raw",
                "index": 0,
                "width": 0,
                "height": 0,
                "fps": 0,
                "mjpg": False,
            }
        ],
    )

    return LaunchDescription([tracker_node, webcam_node])