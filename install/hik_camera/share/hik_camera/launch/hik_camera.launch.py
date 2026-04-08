import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 配置文件路径
    config_file = os.path.join(
        get_package_share_directory('hik_camera'),
        'config',
        'hik_camera_config.yaml'
    )

    return LaunchDescription([
        Node(
            package='hik_camera',
            executable='hik_camera_node',
            name='hik_dual_camera',
            output='screen',
            emulate_tty=True,
            parameters=[
                config_file
            ],
        )
    ])

