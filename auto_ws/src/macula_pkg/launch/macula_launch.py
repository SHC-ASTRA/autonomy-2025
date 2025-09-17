#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

#Prevent making __pycache__ directories
from sys import dont_write_bytecode
dont_write_bytecode = True

def generate_launch_description():
    declare_arg = DeclareLaunchArgument(
        "camera_ip",
        default_value="12", # Default camera IP, change to front rover
        description="Set camera ID for the camera to use for detection"
    )
    declare_mode = DeclareLaunchArgument(
        "detection_type",
        default_value="1", # Default to ArUco detection
        description="Set detection type: 1 for ArUco, 2 for Object"
    )
    declare_mode = DeclareLaunchArgument(
        "confidence",
        default_value="0.5", # Default to 50%
        description="Set confidence threshold for YOLO object detection"
    )

    return LaunchDescription([
        declare_arg,
        declare_mode,
        Node(
        package='macula_pkg',
        executable='macula',
        name='macula_node',
        output='screen',
        parameters=[
            {'camera_ip': LaunchConfiguration("camera_ip")},
            {'detection_type': LaunchConfiguration("detection_type")},
            ],    
        ) #,
        # Node(
        # package='actions_cpp',
        # executable='navigate_rover_server',
        # name='auto_server',
        # output='screen',    
        # ) ,
        # Node(
        # package='actions_cpp',
        # executable='navigate_rover_client',
        # name='auto_client',
        # output='screen',    
        # )
    ])
