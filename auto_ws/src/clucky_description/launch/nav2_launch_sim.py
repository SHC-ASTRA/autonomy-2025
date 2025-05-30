from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import ThisLaunchFileDir


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nav2_bringup',
            executable='bringup_launch.py',
            output='screen',
            parameters=[
                ThisLaunchFileDir() + '/../config/nav2_params_sim.yaml'
            ],
            arguments=[
                '--use_sim_time', 'true'
            ]
        )
    ])
