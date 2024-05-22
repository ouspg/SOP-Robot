# Copyright 2021 Open Source Robotics Foundation, Inc.
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

# Based on ROS2 control demo project (foxy branch)
# Read more about those descriptions and launching robot at https://github.com/ros-controls/ros2_control_demos

def generate_launch_description():

    robot_description_content = Command(
      [
          # Get URDF via xacro
          PathJoinSubstitution([FindExecutable(name="xacro")]),
          " ",
          PathJoinSubstitution(
              [
                  FindPackageShare("inmoov_description"),
                  "robots",
                  "inmoov.urdf.xacro",
              ]
          ),
          " use_fake_hardware:=true",
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

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
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
    
    controllers_to_start = [
        "head_controller",
        "eyes_controller",
        "jaw_controller",
        "r_hand_controller",
        "r_shoulder_controller",
        "l_hand_controller"
    ]
    
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
        rviz_node
    ]

    return LaunchDescription(nodes)
