from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            # Declare launch-time Arugments
            DeclareLaunchArgument("mission_type", default_value="0"),
            DeclareLaunchArgument("lat", default_value="0.00"),
            DeclareLaunchArgument("longi", default_value="0.00"),
            DeclareLaunchArgument("radius", default_value="0.0"),
            DeclareLaunchArgument("period", default_value="0.8"),
            # Action Server Node
            Node(
                package="actions_cpp",
                executable="navigate_rover_server",
                name="auto_server",
                output="screen",
            ),
            # Goal Send Node
            Node(
                package="actions_cpp",
                executable="goal",
                name="auto_client",
                output="screen",
                parameters=[
                    {
                        "mission_type": LaunchConfiguration("mission_type"),
                        "lat": LaunchConfiguration("lat"),
                        "longi": LaunchConfiguration("longi"),
                        "radius": LaunchConfiguration("radius"),
                        "period": LaunchConfiguration("period"),
                    }
                ],
            ),
        ]
    )
