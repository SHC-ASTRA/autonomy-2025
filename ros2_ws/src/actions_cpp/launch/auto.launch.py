from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='actions_cpp',
            executable='astra_auto_server',
            name='auto_server',
            output='screen'
        ),
        Node(
            package='actions_cpp',
            executable='astra_auto_client',
            name='auto_client',
            output='screen'
        )
    ])