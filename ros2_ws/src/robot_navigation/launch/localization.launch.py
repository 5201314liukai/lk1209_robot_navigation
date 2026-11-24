import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    nav2_dir = get_package_share_directory('nav2_bringup')

    params_file = os.path.join(
        get_package_share_directory('robot_navigation'),
        'config',
        'nav2_params.yaml'
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            'map',
            description='Path to the saved map YAML file'
        ),

        DeclareLaunchArgument(
            'params_file',
            default_value=params_file
        ),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    nav2_dir,
                    'launch',
                    'localization_launch.py'
                )
            ),

            launch_arguments={
                'map': LaunchConfiguration('map'),
                'params_file': LaunchConfiguration('params_file'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'autostart': 'True',
                'use_composition': 'False'
            }.items()
        )
    ])

