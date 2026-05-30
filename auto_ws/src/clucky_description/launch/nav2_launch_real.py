from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import ThisLaunchFileDir
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    urdf_path = os.path.join(
        get_package_share_directory("clucky_description"), "urdf", "clucky.urdf"
    )
    return LaunchDescription(
        [
            Node(
                package="nav2_bringup",
                executable="bringup_launch.py",
                output="screen",
                parameters=[ThisLaunchFileDir() + "/../config/nav2_params_real.yaml"],
                arguments=["--use_sim_time", "false"],
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[{"use_sim_time": False}],
                arguments=[urdf_path],
            ),
        ]
    )
