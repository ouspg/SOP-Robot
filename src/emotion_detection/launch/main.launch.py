from launch import LaunchDescription
from launch_ros.actions import Node
import os


package_name = os.path.realpath(__file__).split(os.sep)[-3] # get name of this package
#package_name = "emotion_detection"




def generate_launch_description():
    
    launch_description = LaunchDescription()


    face_detection_node = Node(
        package=package_name,
        executable="face_detection",
    )

    emotion_detection_node = Node(
        package=package_name,
        executable="emotion_detection",

    )

    action_client_node = Node(
        package=package_name,
        executable="action_client",
        # output='screen',
        # emulate_tty=True,
    )

    launch_description.add_action(face_detection_node)
    launch_description.add_action(emotion_detection_node)
    launch_description.add_action(action_client_node)

    return launch_description
