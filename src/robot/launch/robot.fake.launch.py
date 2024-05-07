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
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch.actions import RegisterEventHandler
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit
import xacro

# Based on ROS2 control demo project (foxy branch)
# Read more about those descriptions and launching robot at https://github.com/ros-controls/ros2_control_demos

def generate_launch_description():


    #robot_description = {"robot_description": robot_description_content}
    robot_description = os.path.join(get_package_share_directory("inmoov_description"), "robots", "inmoov.urdf.xacro")
    robot_description_config = xacro.process_file(robot_description)
    controller_config = os.path.join(
        get_package_share_directory(
            "robot"), "controllers", "robot.yaml"
    )

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description_config.toxml()}],
    )

    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{"robot_description": robot_description_config.toxml()}, controller_config],
        output="screen",
    )

    spawn_jsb_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("inmoov_description"), "config", "inmoov.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
        output="screen",
    )

  

    head_fake_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["head_controller", "-c", "/controller_manager"],
    )

    eyes_fake_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["eyes_controller", "-c", "/controller_manager"],
    )

    jaw_fake_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["jaw_controller", "-c", "/controller_manager"],
    )

    r_shoulder_fake_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["r_shoulder_controller", "-c", "/controller_manager"],
    )

    r_hand_fake_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["r_hand_controller", "-c", "/controller_manager"],
    )
    l_hand_fake_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["l_hand_controller", "-c", "/controller_manager"],
    )

    nodes = [
        controller_manager_node,
        spawn_jsb_controller,
        head_fake_controller_spawner,
        eyes_fake_controller_spawner,
        jaw_fake_controller_spawner,
        r_shoulder_fake_controller_spawner,
        r_hand_fake_controller_spawner,
        l_hand_fake_controller_spawner,
        node_robot_state_publisher,
        rviz_node,
    ]
    return LaunchDescription(nodes)
