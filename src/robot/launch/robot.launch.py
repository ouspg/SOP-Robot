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
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

sys.path.insert(0, os.path.dirname(__file__))
from launch_utils import create_dynamixel_config_file, parse_controller_argument


DEFAULT_CONTROLLERS = "head_controller,eyes_controller,l_hand_controller"


def build_launch(context):
    robot_parts = LaunchConfiguration("robot_parts").perform(context)
    use_rviz = LaunchConfiguration("use_rviz").perform(context).lower() == "true"
    controllers_to_start = parse_controller_argument(
        LaunchConfiguration("enabled_controllers").perform(context)
    )
    dynamixel_config_file = create_dynamixel_config_file(robot_parts)

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("inmoov_description"),
                    "robots",
                    "inmoov.urdf.xacro",
                ]
            ),
            " dynamixel_config_file:=",
            dynamixel_config_file,
            " use_fake_hardware:=false",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    controller = os.path.join(
        get_package_share_directory('robot'),
        'controllers',
        'robot.yaml'
    )
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("inmoov_description"), "config", "inmoov.rviz"]
    )

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )
    spawn_jsb_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
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

    controller_spawners = [
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[controller_name, "-c", "/controller_manager"]
        ) for controller_name in controllers_to_start
    ]

    nodes = [
        ros2_control_node,
        spawn_jsb_controller,
        *controller_spawners,
        node_robot_state_publisher,
    ]

    if use_rviz:
        nodes.append(
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config_file],
                output="screen",
            )
        )

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot_parts", default_value="all"),
        DeclareLaunchArgument("enabled_controllers", default_value=DEFAULT_CONTROLLERS),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        OpaqueFunction(function=build_launch),
    ])
