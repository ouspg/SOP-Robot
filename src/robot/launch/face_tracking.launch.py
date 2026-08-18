from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    webcam_node = Node(
        package="inmoov_i2",
        executable="i2e_webcam",
        name="i2e_webcam_node",
        output="screen",
        parameters=[{
            "topic_name": "/i2e_webcam",
        }],
    )

    face_detection_node = Node(
        package="perception",
        executable="face_detection",
        name="face_detection_node",
        output="screen",
        parameters=[{
            "webcam_topic": "/i2e_webcam",
            "face_topic": "/faces",
        }],
    )

    face_track_node = Node(
        package="behaviour",
        executable="face_track",
        name="face_track_node",
        output="screen",
    )

    head_gesture_node = Node(
        package="behaviour",
        executable="head_gesture",
        name="head_gesture_node",
        output="screen",
    )

    return LaunchDescription([
        webcam_node,
        face_detection_node,
        head_gesture_node,
        face_track_node,
    ])
