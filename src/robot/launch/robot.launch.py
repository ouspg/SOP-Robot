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

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import xacro




def generate_launch_description():

    # Get URDF via xacro

    # robot_description_path = os.path.join(
    #     get_package_share_directory('inmoov_description'),
    #     'robots',
    #     'inmoov.urdf.xacro')
    # robot_description_config = xacro.process_file(robot_description_path)
    # robot_description = {'robot_description': robot_description_config.toxml()}

    
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "dynamixel_config_file",
            default_value="NOT_SET",
            description="Dynamixel config file for launch. Can be constructed launch-time to include only the needed servos",
        )
    )

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
          LaunchConfiguration("dynamixel_config_file"),
      ]
    )
    robot_description = {"robot_description": robot_description_content}



    controller = os.path.join(
        get_package_share_directory('robot'),
        'controllers',
        'head.yaml'
        )


    # Could this mechanism be used to delete temporary "launch-time" files after launch?
    # rviz_node = Node(
    #   package="rviz2",
    #   executable="rviz2",
    #   name="rviz2",
    #   arguments=["-d", rviz_config_file],
    #   condition=IfCondition(LaunchConfiguration("start_rviz")),
    # ) 
    # delayed_rviz_node = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=spawn_jsb_controller,
    #         on_exit=[rviz_node],
    #     )
    # )

    nodes = [
      Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controller],
        output={
          'stdout': 'screen',
          'stderr': 'screen',
          },
        )

    ]

    
    return LaunchDescription(declared_arguments + nodes)