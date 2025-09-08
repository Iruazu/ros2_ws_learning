import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # use_sim_timeパラメータをブール値で定義
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # TurtleBot3 Gazeboパッケージのパスを取得
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    
    # ワールドファイルのパスを指定
    world = os.path.join(pkg_turtlebot3_gazebo, 'worlds', 'turtlebot3_world.world')

    # Gazeboサーバー(物理計算)を起動
    gzserver_cmd = ExecuteProcess(
        cmd=['gzserver', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world],
        output='screen'
    )

    # Gazeboクライアント(GUI)を起動
    gzclient_cmd = ExecuteProcess(
        cmd=['gzclient'],
        output='screen'
    )

    # Robot State Publisherを起動
    urdf_file_name = 'turtlebot3_burger.urdf'
    urdf = os.path.join(
        get_package_share_directory('turtlebot3_description'),
        'urdf',
        urdf_file_name)
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[urdf]
    )

    # ロボットをワールドに出現させる (Spawn)
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'burger',
            '-x', '-2.0',
            '-y', '-0.5',
            '-z', '0.01'
        ],
        output='screen'
    )

    # あなたの障害物回避ノードを起動
    obstacle_avoider_node = Node(
        package='my_tb3_controller',
        executable='obstacle_avoider',
        name='obstacle_avoider_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # LaunchDescriptionに全てのプロセスとノードを登録
    return LaunchDescription([
        gzserver_cmd,
        gzclient_cmd,
        robot_state_publisher_node,
        spawn_entity_node,
        obstacle_avoider_node
    ])