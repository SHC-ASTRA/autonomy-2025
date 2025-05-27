#==========================================================
# Imports
#==========================================================
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

#==========================================================
# Launch Description
#==========================================================
def generate_launch_description():
    pkg_share = FindPackageShare(package='clucky_description')