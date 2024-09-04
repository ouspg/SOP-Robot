import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

#index should be lowest index for the camera as usb devices usually have multiple
#index 0 is for first or only webcam like integrated laptop webcam for example
#for virtual machines it is easiest to port thought only the camera you are going to use and changing the camera from virtual machine settings is easier.
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
                "lip_movement_detector": "1_32_False_True_0.25_lip_motion_net_model.h5",
            }
        ],
    )

    return LaunchDescription([camera_node, tracker_node])
