import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

#index should be lowest index for the camera as usb devices usually have multiple
#index 0 is for first or only webcam like integrated laptop webcam for example
#for virtual machines it is easiest to port thought only the camera you are going to use and changing the camera from virtual machine settings is easier.
def generate_launch_description():
    camera_source = LaunchConfiguration("camera_source")

    tracker_node = Node(
        package="face_tracker",
        executable="face_tracker_node",
        namespace="face_tracker",
        parameters=[
            {
                "lip_movement_detection": False,
                "face_recognition": True,
                "correlation_tracking": False,
                "cluster_similarity_threshold": 0.3,
                "subcluster_similarity_threshold": 0.2,
                "pair_similarity_maximum": 1.0,
                "face_recognition_model": "SFace",
                "face_detection_model": "yunet",
                "prefer_gpu": True,
                "gpu_face_recognition_model": "SFace",
                "gpu_face_detection_model": "yolov8n",
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
                "source": camera_source,
                "index": 0,
                "width": 1280,
                "height": 960,
                "fps": 30,
                "mjpg": True,
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "camera_source",
                default_value="0",
                description="Camera device index or network stream URL (for example DroidCam MJPEG URL)",
            ),
            tracker_node,
            webcam_node,
        ]
    )
