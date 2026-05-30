from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            # Node to receive external goals and send NavigateToPose actions
            Node(
                package="clucky_control",
                executable="goal_sender",
                name="goal_sender",
                output="screen",
                parameters=[{"use_sim_time": False}],
            ),
            # Node to bridge /cmd_vel into your drive command format
            Node(
                package="clucky_control",
                executable="cmd_vel_bridge",
                name="cmd_vel_bridge",
                output="screen",
                parameters=[{"use_sim_time": False}],
            ),
        ]
    )
