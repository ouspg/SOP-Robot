from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    full_demo = Node(
        package='full_demo',
        executable='full_demo_node',
        name='full_demo',
        output='screen',
    )
    hand_gestures = Node(
        package='hand_gestures',
        executable='hand_gestures_node',
        name='hand_gestures',
        output='screen',
        parameters=[{'simulation': True}],
    )
    unified_arms = Node(
        package='unified_arms',
        executable='unified_arms_node',
        name='unified_arms',
        output='screen',
    )
    face_movement = Node(
        package='face_tracker_movement',
        executable='face_tracker_movement_node',
        name='face_tracker_movement',
        output='screen',
        parameters=[{'simulation': True}],
    )
    image_view = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='image_view',
        output='screen',
        arguments=['--clear-config', '/face_tracker/image_face'],
        additional_env={
            'XDG_CONFIG_HOME': '/tmp/sop_robot_full_demo_rqt',
        },
    )

    return LaunchDescription(
        [
            full_demo,
            hand_gestures,
            unified_arms,
            face_movement,
            image_view,
        ]
    )
