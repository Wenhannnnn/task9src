from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop_twist_keyboard',
            output='screen',
            emulate_tty=True,
        ),
    ])

    pkg_name = 'turtlebot3_world'
    sdf_path = os.path.join(get_package_share_directory(pkg_name), 'worlds', 'your_map_file.sdf')

    return LaunchDescription([
        # 启动map_server节点来发布地图
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            namespace='',
            output='screen',
            parameters=[{'use_sim_time': True, 'yaml_filename': sdf_path}],  # 设置地图文件路径
        ),
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            namespace='',
            output='screen',
            parameters=[{'use_sim_time': True, 'yaml_filename': sdf_path}],  # 设置地图文件路径
        ),
    ])
if __name__ == '__main__':
    generate_launch_description()
