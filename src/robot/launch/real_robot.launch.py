from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare('robot'),
                    'launch',
                    'robot.launch.py',
                ]
            )
        ),
        launch_arguments={'hardware_output': 'own_log'}.items(),
    )

    chatbot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare('robot'),
                    'launch',
                    'chatbot.launch.py',
                ]
            )
        )
    )

    face_tracker = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare('face_tracker'),
                    'launch',
                    'face_tracker.launch.py',
                ]
            )
        ),
        launch_arguments={
            'lip_movement_detection': LaunchConfiguration('lip_movement_detection'),
        }.items(),
    )

    full_demo = Node(
        package='full_demo',
        executable='full_demo_node',
        name='full_demo',
        output='both',
    )
    hand_gestures = Node(
        package='hand_gestures',
        executable='hand_gestures_node',
        name='hand_gestures',
        output={'both': 'log'},
        parameters=[{'simulation': False}],
    )
    unified_arms = Node(
        package='unified_arms',
        executable='unified_arms_node',
        name='unified_arms',
        output={'both': 'log'},
    )
    face_movement = Node(
        package='face_tracker_movement',
        executable='face_tracker_movement_node',
        name='face_tracker_movement',
        output={'both': 'log'},
        parameters=[{'simulation': False}],
    )
    image_view = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='image_view',
        output={'both': 'log'},
        arguments=['--clear-config', '/face_tracker/image_face'],
        additional_env={
            'XDG_CONFIG_HOME': '/tmp/sop_robot_full_demo_rqt',
        },
    )

    application_nodes = TimerAction(
        period=5.0,
        actions=[
            chatbot,
            face_tracker,
            full_demo,
            hand_gestures,
            unified_arms,
            face_movement,
            image_view,
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument('lip_movement_detection', default_value='false'),
            robot,
            application_nodes,
        ]
    )
