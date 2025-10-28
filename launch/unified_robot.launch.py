#!/usr/bin/env python3
"""
Launch file for Unified Robot Control System

This launch file starts all necessary nodes for the complete robot system:
- Robot hardware/controllers
- Face tracker
- Speech recognition
- QA bot
- TTS service
- Unified robot control

Usage:
    ros2 launch unified_robot_control unified_robot.launch.py [simulation:=true/false]
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # Get package directories
    unified_robot_control_pkg = FindPackageShare('unified_robot_control')
    robot_pkg = FindPackageShare('robot')
    face_tracker_pkg = FindPackageShare('face_tracker')
    
    # Declare launch arguments
    simulation_arg = DeclareLaunchArgument(
        'simulation',
        default_value='true',
        description='Launch in simulation mode'
    )
    
    enable_face_tracking_arg = DeclareLaunchArgument(
        'enable_face_tracking',
        default_value='true',
        description='Enable face tracking'
    )
    
    enable_speech_arg = DeclareLaunchArgument(
        'enable_speech',
        default_value='true',
        description='Enable speech recognition and dialogue'
    )
    
    enable_arms_arg = DeclareLaunchArgument(
        'enable_arms',
        default_value='true',
        description='Enable arm and hand control'
    )
    
    robot_parts_arg = DeclareLaunchArgument(
        'robot_parts',
        default_value='head',
        description='Robot parts to launch (head/arm/both)'
    )
    
    # Get launch configurations
    simulation = LaunchConfiguration('simulation')
    enable_face_tracking = LaunchConfiguration('enable_face_tracking')
    enable_speech = LaunchConfiguration('enable_speech')
    enable_arms = LaunchConfiguration('enable_arms')
    robot_parts = LaunchConfiguration('robot_parts')
    
    # Robot launch file (hardware or simulation)
    robot_launch_cmd = ExecuteProcess(
        cmd=['ros2', 'launch', 'robot', 
             'robot.fake.launch.py' if simulation == 'true' else 'robot.launch.py',
             #f'robot_parts:={robot_parts}'],
        ],
        output='screen',
        name='robot_launch'
    )
    
    # Face tracker node
    face_tracker_node = Node(
        package='face_tracker',
        executable='face_tracker_node',
        name='face_tracker_node',
        condition=IfCondition(enable_face_tracking),
        parameters=[{
            'use_sim_time': simulation
        }],
        output='screen'
    )
    
    # Speech recognizer node
    speech_recognizer_node = Node(
        package='speech_recognizer',
        executable='speech_recognizer_node',
        name='speech_recognizer_node',
        condition=IfCondition(enable_speech),
        output='screen'
    )
    
    # QA bot node
    qabot_node = Node(
        package='qabot',
        executable='client',
        name='qabot_node',
        condition=IfCondition(enable_speech),
        output='screen'
    )
    
    # TTS service node (with venv activation)
    tts_service_node = Node(
        package='tts_package',
        executable='service',
        name='tts_service_node',
        condition=IfCondition(enable_speech),
        output='screen'
    )
    
    # Hand gestures node
    hand_gestures_node = Node(
        package='hand_gestures',
        executable='hand_gestures_node',
        name='hand_gestures_node',
        condition=IfCondition(enable_arms),
        output='screen'
    )
    
    # Unified robot control node
    unified_robot_control_node = Node(
        package='unified_robot_control',
        executable='unified_robot_control_node',
        name='unified_robot_control_node',
        parameters=[{
            'simulation': simulation,
            'enable_face_tracking': enable_face_tracking,
            'enable_speech': enable_speech,
            'enable_arms': enable_arms,
            'use_sim_time': simulation
        }],
        output='screen'
    )
    
    # RQT image view for face tracker (optional)
    rqt_image_view = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='rqt_image_view',
        arguments=['/face_tracker/image_face'],
        condition=IfCondition(enable_face_tracking),
        output='screen'
    )
    
    # Launch description
    return LaunchDescription([
        # Launch arguments
        simulation_arg,
        enable_face_tracking_arg,
        enable_speech_arg,
        enable_arms_arg,
        robot_parts_arg,
        
        # Robot hardware/simulation launch
        robot_launch_cmd,
        
        # Wait before starting other nodes to ensure robot is ready
        TimerAction(
            period=5.0,
            actions=[
                face_tracker_node,
                speech_recognizer_node,
                qabot_node,
                tts_service_node,
                hand_gestures_node,
                
                # Start unified control after other nodes are ready
                TimerAction(
                    period=2.0,
                    actions=[
                        unified_robot_control_node,
                        rqt_image_view
                    ]
                )
            ]
        )
    ])
