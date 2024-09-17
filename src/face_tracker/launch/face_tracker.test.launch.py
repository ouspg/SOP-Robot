import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

#index should be lowest index for the camera as usb devices usually have multiple
#index 0 is for first or only webcam like integrated laptop webcam for example
#for virtual machines it is easiest to port thought only the camera you are going to use and changing the camera from virtual machine settings is easier.
def generate_launch_description():

    tracker_node = Node(
        package="face_tracker",
        executable="face_tracker_node",
        namespace="face_tracker",
        parameters=[
            {
                "lip_movement_detection": True,
                "face_recognition": True,
                "correlation_tracking": False,
                "cluster_similarity_threshold": 0.3,
                "subcluster_similarity_threshold": 0.2,
                "pair_similarity_maximum": 1.0,
                "face_recognition_model": "SFace",
                "face_detection_model": "yunet",
                "image_topic": "/image_raw",
                "face_image_topic": "image_face",
                "face_topic": "faces",
                "predictor": "shape_predictor_68_face_landmarks.dat",
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
                "width": 1280,
                "height": 960,
                "fps": 30,
                "mjpg": True,
            }
        ],
    )

    return LaunchDescription([tracker_node, webcam_node])