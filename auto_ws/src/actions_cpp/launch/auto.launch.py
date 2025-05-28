from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='actions_cpp',
            executable='navigate_rover_server',
            name='auto_server',
            output='screen'
        ),
        Node(
            package='actions_cpp',
            executable='goal',
            name='auto_client',
            output='screen',
            parameters=[
                {'mission_type': -1},
                {'lat': 0.00},
                {'long': 0.00},
                {'radius': 1.00},
                {'period': 0.8},
            ]
        ),
    ])