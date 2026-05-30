from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory("clucky_description")
    urdf_path = os.path.join(pkg_dir, "urdf", "clucky.urdf")
    nav2_params_path = os.path.join(pkg_dir, "config", "nav2_params_real.yaml")

    # Launch robot_state_publisher with your URDF
    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": False}],
        arguments=[urdf_path],
    )

    # Launch RTAB-Map in SLAM modeShould
    rtabmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("rtabmap_launch"),
                    "launch",
                    "rtabmap.launch.py",
                )
            ]
        ),
        launch_arguments={
            "use_sim_time": "false",
            "frame_id": "camera_link",
            "rgb_topic": "/camera/color/image_raw",
            "depth_topic": "/camera/depth/image_raw",
            "camera_info_topic": "/camera/color/camera_info",
            "subscribe_rgb": "true",
            "subscribe_depth": "true",
            "visual_odometry": "true",
            "approx_sync": "true",
        }.items(),
    )

    # Launch Nav2
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("nav2_bringup"),
                    "launch",
                    "bringup_launch.py",
                )
            ]
        ),
        launch_arguments={
            "use_sim_time": "false",
            "params_file": nav2_params_path,
        }.items(),
    )

    return LaunchDescription([robot_state_pub, rtabmap, nav2])
