import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
from lk1209_navigation.nav_profiles import profile_file


def select_lidar(context):
    source = LaunchConfiguration('params_file').perform(context)
    mode = LaunchConfiguration('lidar_mode').perform(context)
    config = os.path.join(get_package_share_directory('robot_navigation'), 'config')
    return [SetLaunchConfiguration('params_file', profile_file(source, 'classic', config, lidar_mode=mode))]


def generate_launch_description():

    # Nav2 官方功能包路径
    nav2_dir = get_package_share_directory('nav2_bringup')

    # LK1209 完整 Nav2 参数文件
    params_file = os.path.join(
        get_package_share_directory('robot_navigation'),
        'config',
        'nav2_params.yaml'
    )

    return LaunchDescription([

        # 建图模式不需要预先加载地图
        # 但 ROS2 Humble Bringup 要求声明 map 参数
        DeclareLaunchArgument(
            'map',
            default_value=''
        ),

        # 仿真时间
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true'
        ),

        # Nav2 参数文件
        DeclareLaunchArgument(
            'params_file',
            default_value=params_file
        ),

        DeclareLaunchArgument('lidar_mode', default_value='2d', choices=['2d', '3d']),
        OpaqueFunction(function=select_lidar),

        # 启动 Nav2 官方 Bringup
        Node(package='obstacle_detection', executable='lidar_detector',
             parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    nav2_dir,
                    'launch',
                    'bringup_launch.py'
                )
            ),

            launch_arguments={

                'slam': 'True',

                'map': LaunchConfiguration('map'),

                'use_sim_time': LaunchConfiguration('use_sim_time'),

                'params_file': LaunchConfiguration('params_file'),

                'autostart': 'True',

                'use_composition': 'False'

            }.items()
        )
    ])

