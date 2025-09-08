import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # TurtleBot3のGazeboシミュレーションのLaunchファイルへのパスを取得
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')

    # あなたの自作パッケージのshareディレクトリへのパスを取得
    pkg_my_tb3_controller = get_package_share_directory('my_tb3_controller')

    # use_sim_timeパラメータをTrueに設定
    use_sim_time = True

    # TurtleBot3のワールドを起動するための設定
    start_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_turtlebot3_gazebo, 'launch', 'turtlebot3_world.launch.py')
        ),
        # TurtleBot3のLaunchファイルにもuse_sim_timeを渡す
        launch_arguments={'use_sim_time': str(use_sim_time).lower()}.items(),
    )

    # あなたの障害物回避ノードを起動するための設定
    start_obstacle_avoider_node = Node(
        package='my_tb3_controller',
        executable='obstacle_avoider', # setup.pyで設定した名前
        name='obstacle_avoider_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}] # あなたのノードにもuse_sim_timeを設定
    )

    # 上記の２つの設定をLaunchDescriptionにまとめて返す
    return LaunchDescription([
        start_world,
        start_obstacle_avoider_node
    ])