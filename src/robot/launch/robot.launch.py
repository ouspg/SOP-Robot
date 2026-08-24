# Copyright 2020 ROS2-Control Development Team (2020)
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

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

dynamixel_config_file = 'NOT_SET'

DYNAMIXEL_CONFIG_FILE_PREFIX = 'config/'
DYNAMIXEL_CONFIG_FILEPATH_HEAD = DYNAMIXEL_CONFIG_FILE_PREFIX + 'dynamixel_head.yaml'
DYNAMIXEL_CONFIG_FILEPATH_ARM = DYNAMIXEL_CONFIG_FILE_PREFIX + 'dynamixel_arm.yaml'
ALL_AVAILABLE_DYNAMIXEL_CONFIG_FILES = [
    DYNAMIXEL_CONFIG_FILEPATH_HEAD,
    DYNAMIXEL_CONFIG_FILEPATH_ARM,
]

DYNAMIXEL_CONFIG_FILEPATH_FOR_LAUNCH = (
    DYNAMIXEL_CONFIG_FILE_PREFIX + 'temp_dynamixel_for_launch.yaml'
)  # Dynamic file created during launch if both arm and head are enabled


def generate_launch_description():
    dynamixel_config_file = generate_dynamixel_config_file()

    robot_description_content = Command(
        [
            # Get URDF via xacro
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [
                    FindPackageShare('inmoov_description'),
                    'robots',
                    'inmoov.urdf.xacro',
                ]
            ),
            ' dynamixel_config_file:=',
            dynamixel_config_file,
            ' use_fake_hardware:=false',  # No fake hardware, this is real.
        ]
    )
    robot_description = {'robot_description': robot_description_content}

    controller = os.path.join(get_package_share_directory('robot'), 'controllers', 'robot.yaml')
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare('inmoov_description'), 'config', 'inmoov.rviz']
    )

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )
    spawn_jsb_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    _rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
    )

    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controller],
        output={
            'stdout': 'screen',
            'stderr': 'screen',
        },
    )

    controllers_to_start = [
        'head_controller',
        'eyes_controller',
        # "jaw_controller",
        # "r_hand_controller",
        # "r_shoulder_controller",
        'l_hand_controller',
    ]

    controller_spawners = [
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=[controller_name, '-c', '/controller_manager'],
        )
        for controller_name in controllers_to_start
    ]
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

    nodes = [
        ros2_control_node,
        spawn_jsb_controller,
        *controller_spawners,
        node_robot_state_publisher,
        # _rviz_node
        full_demo,
        hand_gestures,
        unified_arms,
        face_movement,
        image_view,
    ]

    return LaunchDescription(nodes)


# Generates a "launch-time" configuration file for Dynamixel servos.
# The idea is to enable using only arm or only head without the need to touch the code.
# Launch only arm (and hand):
#   ros2 launch robot robot.launch.py robot_parts:=arm
# Launch only head:
#   ros2 launch robot robot.launch.py robot_parts:=head
# If no valid parameter is given, include every available part (arm, hand, and head).
def generate_dynamixel_config_file():
    included_files = []
    for arg in sys.argv:
        if arg.startswith('robot_parts:='):
            parts = arg.split(':=')[1]
            if len(parts) <= 0:
                # Use all if parts not correctly specified
                included_files = ALL_AVAILABLE_DYNAMIXEL_CONFIG_FILES
                print('Warning, robot parts not correctly specified! Using all available parts.')
            else:
                if 'head' in parts:
                    print('Note: Configuring servos only for robot head!')
                    included_files.append(DYNAMIXEL_CONFIG_FILEPATH_HEAD)
                elif 'arm' in parts:
                    print('Note: Configuring servos only for robot arm!')
                    included_files.append(DYNAMIXEL_CONFIG_FILEPATH_ARM)

    if len(included_files) == 0:
        # Use all if argument was not given
        print('Note: Configuring servos for all robot parts!')
        included_files = ALL_AVAILABLE_DYNAMIXEL_CONFIG_FILES

    with open(DYNAMIXEL_CONFIG_FILEPATH_FOR_LAUNCH, 'w') as outfile:
        outfile.write(
            '# NOTE! This temporary launch file is based on the enabled robot parts.\n'
            '# It configures only the required Dynamixel servos, avoiding errors for '
            'missing servo IDs.\n\n'
        )
        for filename in included_files:
            with open(filename) as infile:
                outfile.write(infile.read())

    return DYNAMIXEL_CONFIG_FILEPATH_FOR_LAUNCH
