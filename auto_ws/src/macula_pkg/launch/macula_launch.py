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

    return LaunchDescription([
        declare_arg,
        Node(
        package='macula_pkg',
        executable='macula',
        name='macula_node',
        output='screen',
        parameters=[
            {'camera_ip': LaunchConfiguration("camera_ip")}
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
