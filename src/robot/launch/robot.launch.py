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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import xacro

dynamixel_config_file = "NOT_SET"

DYNAMIXEL_CONFIG_FILE_PREFIX = "config/"
DYNAMIXEL_CONFIG_FILEPATH_HEAD = DYNAMIXEL_CONFIG_FILE_PREFIX + "dynamixel_head.yaml"
DYNAMIXEL_CONFIG_FILEPATH_ARM = DYNAMIXEL_CONFIG_FILE_PREFIX + "dynamixel_arm.yaml"
ALL_AVAILABLE_DYNAMIXEL_CONFIG_FILES = [DYNAMIXEL_CONFIG_FILEPATH_HEAD, DYNAMIXEL_CONFIG_FILEPATH_ARM]

DYNAMIXEL_CONFIG_FILEPATH_FOR_LAUNCH = DYNAMIXEL_CONFIG_FILE_PREFIX + "_temp_dynamixel_for_launch.yaml" # Dynamic file created during launch if both arm and head are enabled



# TODO "robot parts" is not very descriptive -> improve terminology.
def create_dynamixel_config_file():
    included_files = []
    for arg in sys.argv:
        if arg.startswith("robot_parts:="):
            parts = arg.split(":=")[1]
            if len(parts) <= 0:
                # Use all if parts not correctly specified
                included_files = ALL_AVAILABLE_DYNAMIXEL_CONFIG_FILES
                print("Warning, robot parts not correctly specified! Using all available parts.")
            else:
                if "head" in parts:
                    print("Note: Configuring servos only for robot head!")
                    included_files.append(DYNAMIXEL_CONFIG_FILEPATH_HEAD)
                elif "arm" in parts:
                    print("Note: Configuring servos only for robot arm!")
                    included_files.append(DYNAMIXEL_CONFIG_FILEPATH_ARM)

    if len(included_files) == 0:
        # Use all if argument was not given
        print("Note: Configuring servos for all robot parts!")
        included_files = ALL_AVAILABLE_DYNAMIXEL_CONFIG_FILES

    with open(DYNAMIXEL_CONFIG_FILEPATH_FOR_LAUNCH, 'w') as outfile:
        for filename in included_files:
            with open(filename) as infile:
                outfile.write(infile.read())

    return DYNAMIXEL_CONFIG_FILEPATH_FOR_LAUNCH




def generate_launch_description():

    dynamixel_config_file = create_dynamixel_config_file()

    # Get URDF via xacro

    # robot_description_path = os.path.join(
    #     get_package_share_directory('inmoov_description'),
    #     'robots',
    #     'inmoov.urdf.xacro')
    # robot_description_config = xacro.process_file(robot_description_path)
    # robot_description = {'robot_description': robot_description_config.toxml()}

    
    # declared_arguments = []
    # declared_arguments.append(
    #     DeclareLaunchArgument(
    #         "dynamixel_config_file",
    #         default_value="NOT_SET",
    #         description="Dynamixel config file for launch. Can be constructed launch-time to include only the needed servos",
    #     )
    # )

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
          #LaunchConfiguration("dynamixel_config_file"),
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

    
    #return LaunchDescription(declared_arguments + nodes)
    return LaunchDescription(nodes)