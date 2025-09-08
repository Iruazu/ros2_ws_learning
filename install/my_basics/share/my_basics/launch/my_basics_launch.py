import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='my_basics',
            executable='talker',
            name='talker_node'
        ),
        launch_ros.actions.Node(
            package='my_basics',
            executable='listener',
            name='listener_node'
        )
    ])