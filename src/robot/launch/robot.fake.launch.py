import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

import xacro


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():

    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Start robot with fake hardware mirroring command to its states.",
        )
    )


    # Get URDF via xacro
    robot_description_path = os.path.join(
        get_package_share_directory('inmoov_description'),
        'robots',
        'inmoov.urdf.xacro')

    # Have to use seperate descriptions for each controller,
    # because otherwise controllers would publish the default joint value for joints they do not control, causing joint state value conflicts
    robot_head_description_path = os.path.join(
        get_package_share_directory('inmoov_description'),
        'robots',
        'inmoov_head_control.urdf.xacro')

    robot_jaw_description_path = os.path.join(
        get_package_share_directory('inmoov_description'),
        'robots',
        'inmoov_jaw_control.urdf.xacro')
    
    robot_eyes_description_path = os.path.join(
        get_package_share_directory('inmoov_description'),
        'robots',
        'inmoov_eyes_control.urdf.xacro')

    robot_description_config = xacro.process_file(robot_description_path)
    robot_description = {'robot_description': robot_description_config.toxml()}

    robot_head_description_config = xacro.process_file(robot_head_description_path)
    robot_head_description = {'robot_description': robot_head_description_config.toxml()}

    robot_jaw_description_config = xacro.process_file(robot_jaw_description_path)
    robot_jaw_description = {'robot_description': robot_jaw_description_config.toxml()}

    robot_eyes_description_config = xacro.process_file(robot_eyes_description_path)
    robot_eyes_description = {'robot_description': robot_eyes_description_config.toxml()}

    robot_description_semantic_config = load_file(
        'inmoov_description', 'config/inmoov.srdf')
    robot_description_semantic = {
        'robot_description_semantic': robot_description_semantic_config}

    controller = os.path.join(
        get_package_share_directory('robot'),
        'controllers',
        'head.yaml'
    )

    # RViz
    rviz_config_file = os.path.join(get_package_share_directory('inmoov_description'), "config", "inmoov.rviz")
    rviz_node = Node(package='rviz2',
                     executable='rviz2',
                     name='rviz2',
                     output='log',
                     arguments=['-d', rviz_config_file],
                     parameters=[robot_description,
                                 robot_description_semantic])

    # Publish TF
    robot_state_publisher = Node(package='robot_state_publisher',
                                 executable='robot_state_publisher',
                                 name='robot_state_publisher',
                                 output='both',
                                 parameters=[robot_description])

    # Static TF (where the robot is in the world frame)
    static_tf = Node(package='tf2_ros',
                     executable='static_transform_publisher',
                     name='static_transform_publisher',
                     output='log',
                     arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'world', 'base_link'])

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controller],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_state_broadcaster"],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
    )

    joint_state_publisher_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_state_publisher"],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
    )

    head_fake_controller_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["head_controller"],
    )

# Trying to remove fake joint package WIP, based on example at https://github.com/ros-controls/ros2_control_demos/tree/foxy
# Controller and stuff may work but stil some problem that robot does not show up
# This error static_tf,
    return LaunchDescription(declared_arguments + [
        
        control_node,
        robot_state_publisher,
        
        rviz_node,
        joint_state_broadcaster_spawner,
        head_fake_controller_spawner,
    ])
