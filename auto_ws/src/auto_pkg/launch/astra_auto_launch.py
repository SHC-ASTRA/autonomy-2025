import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='auto_pkg',
            executable='astra_auto_server',
            name='auto_server'
        ),
        launch_ros.actions.Node(
            package='auto_pkg',
            executable='auto_client_startup',
            name='auto_client_start'
        )
    ])