from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def start_dispatcher(context):
    parameters = [{'use_sim_time': True}]
    if LaunchConfiguration('fleet').perform(context) == 'true':
        share = Path(get_package_share_directory('robot_navigation'))
        parameters.append(str(share / 'config' / 'fleet_delivery.yaml'))
    return [Node(package='robot_navigation', executable='delivery_dispatcher',
                 parameters=parameters, output='screen')]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('fleet', default_value='false', choices=['true', 'false']),
        OpaqueFunction(function=start_dispatcher),
    ])
