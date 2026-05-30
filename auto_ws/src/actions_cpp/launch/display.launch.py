# ==========================================================
# Imports
# ==========================================================
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


# ==========================================================
# Generate Description
# ==========================================================
def generate_launch_description():
    pkg_share = FindPackageShare(package="clucky_description").find(
        "clucky_description"
    )
    default_model_path = os.path.join(
        pkg_share, "src", "description", "testbed_description.urdf"
    )
    default_rviz_config_path = os.path.join(pkg_share, "rviz", "config.rviz")

    # ======================================================
    # Description
    # ======================================================
