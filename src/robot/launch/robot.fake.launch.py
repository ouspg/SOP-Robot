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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch.actions import RegisterEventHandler
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit

# Based on ROS2 control demo project (foxy branch)
# Read more about those descriptions and launching robot at https://github.com/ros-controls/ros2_control_demos

def generate_launch_description():

    # Setting arguments, currently fake robot works without using arguments so default is always used
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "start_rviz",
            default_value="true",
            description="start RViz automatically with the launch file",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix of the joint names, useful for \
        multi-robot setup. If changed than also joint names in the controllers' configuration \
        have to be updated.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="true",
            description="Start robot with fake hardware mirroring command to its states.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "fake_sensor_commands",
            default_value="true",
            description="Enable fake command interfaces for sensors used for simple simulations. \
            Used only if 'use_fake_hardware' parameter is true.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "slowdown", default_value="3.0", description="Slowdown factor of the RRbot."
        )
    )

    # Get URDF via xacro
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
            " prefix:=",
            LaunchConfiguration("prefix"),
            " use_fake_hardware:=",
            LaunchConfiguration("use_fake_hardware"),
            " fake_sensor_commands:=",
            LaunchConfiguration("fake_sensor_commands"),
            " slowdown:=",
            LaunchConfiguration("slowdown"),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    rrbot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("robot"),
            "controllers",
            "head.yaml",
        ]
    )

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, rrbot_controllers],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
    )

    spawn_jsb_controller = Node(
        package="controller_manager",
        executable="spawner.py",
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
        condition=IfCondition(LaunchConfiguration("start_rviz")),
    )

    # Not currently used/needed, but could be useful later when needing delayed start
    delayed_rviz_node = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_jsb_controller,
            on_exit=[rviz_node],
        )
    )

    head_fake_controller_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["head_controller", "-c", "/controller_manager"],
    )

    eyes_fake_controller_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["eyes_controller", "-c", "/controller_manager"],
    )

    jaw_fake_controller_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["jaw_controller", "-c", "/controller_manager"],
    )

    nodes = [
        controller_manager_node,
        node_robot_state_publisher,
        spawn_jsb_controller,
        rviz_node,
        head_fake_controller_spawner,
        eyes_fake_controller_spawner,
        jaw_fake_controller_spawner
    ]
    return LaunchDescription(declared_arguments + nodes)
