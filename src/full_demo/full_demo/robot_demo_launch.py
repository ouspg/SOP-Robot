# Copyright 2026 Aapo2001
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_robot_demo_launch_description(*, use_fake_hardware):
    robot_launch_file = 'robot.fake.launch.py' if use_fake_hardware else 'robot.launch.py'

    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare('robot'),
                    robot_launch_file,
                ]
            )
        )
    )
    chatbot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare('full_demo'),
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
        )
    )

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
        parameters=[{'simulation': use_fake_hardware}],
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

    return LaunchDescription([robot, application_nodes])
