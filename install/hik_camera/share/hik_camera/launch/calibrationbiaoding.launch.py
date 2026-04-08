import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, ExecuteProcess
from launch.substitutions import LaunchConfiguration, FindExecutable
from launch_ros.actions import Node


def generate_launch_description():
    # 使用原有启动文件的参数配置
    params_file = os.path.join(
        get_package_share_directory('hik_camera'), 'config', 'camera_params.yaml')
    
    camera_info_url = 'package://hik_camera/config/camera_info.yaml'
    
    # 标定参数
    board_size = LaunchConfiguration('board_size')
    square_size = LaunchConfiguration('square_size')
    image_topic = LaunchConfiguration('image_topic')
    camera_ns = LaunchConfiguration('camera_ns')

    return LaunchDescription([
        # 原有参数声明
        DeclareLaunchArgument(name='params_file',
                             default_value=params_file),
        DeclareLaunchArgument(name='camera_info_url',
                             default_value=camera_info_url),
        DeclareLaunchArgument(name='use_sensor_data_qos',
                             default_value='false'),
        
        # 标定参数声明
        DeclareLaunchArgument(
            'board_size',
            default_value='6x9',
            description='棋盘格内角点数量，如 8x6, 9x6 等'
        ),
        DeclareLaunchArgument(
            'square_size', 
            default_value='0.024',
            description='每个方格的实际尺寸（米）'
        ),
        DeclareLaunchArgument(
            'image_topic',
            default_value='/image_raw',
            description='图像话题名称'
        ),
        DeclareLaunchArgument(
            'camera_ns',
            default_value='/hik_camera', 
            description='相机命名空间'
        ),

        # 原有相机节点
        Node(
            package='hik_camera',
            executable='hik_camera_node',
            output='screen',
            emulate_tty=True,
            parameters=[LaunchConfiguration('params_file'), {
                'camera_info_url': LaunchConfiguration('camera_info_url'),
                'use_sensor_data_qos': LaunchConfiguration('use_sensor_data_qos'),
            }],
        ),
        
        # 延迟启动标定器，确保相机已初始化
        TimerAction(
            period=5.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        FindExecutable(name='ros2'),
                        'run', 'camera_calibration', 'cameracalibrator',
                        '--size', board_size,
                        '--square', square_size,
                        '--no-service-check',
                        'image:=', image_topic,
                        'camera:=', camera_ns
                    ],
                    output='screen',
                    shell=False
                )
            ]
        )
    ])
