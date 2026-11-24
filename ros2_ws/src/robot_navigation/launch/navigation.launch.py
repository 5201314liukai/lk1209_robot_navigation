import atexit
import hashlib
import os
from pathlib import Path
from tempfile import NamedTemporaryFile

import yaml

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, LogInfo,
    OpaqueFunction, SetLaunchConfiguration,
)
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
from lk1209_navigation.nav_profiles import NAV_PROFILES, profile_file


# This map was recorded at the house spawn, facing map +X.
HOUSE_YAML_SHA256 = '9900bc1a851a2fda7930838c414f478274dea45a31e787e7be698d73e2516e54'
HOUSE_PGM_SHA256 = 'c1e29326788e353d57feb81899257b411f23d893ef0d1f79026a481a94877bb7'


def file_sha256(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(65536), b''):
            digest.update(block)
    return digest.hexdigest()


def select_profile(context):
    source = LaunchConfiguration('params_file').perform(context)
    profile = LaunchConfiguration('nav_profile').perform(context)
    config_dir = Path(get_package_share_directory('robot_navigation')) / 'config'
    filename = profile_file(source, profile, config_dir,
                            lidar_mode=LaunchConfiguration('lidar_mode').perform(context))
    return [SetLaunchConfiguration('params_file', filename)]


def configure_house_pose(context):
    if LaunchConfiguration('world').perform(context) != 'turtlebot3_house':
        return []

    map_file = Path(LaunchConfiguration('map').perform(context))
    # Resolve the image relative to the YAML passed to map_server.
    image_file = map_file.with_name('turtlebot3_house_test.pgm')
    try:
        verified = (map_file.resolve().name == 'turtlebot3_house_test.yaml'
                    and file_sha256(map_file) == HOUSE_YAML_SHA256
                    and file_sha256(image_file) == HOUSE_PGM_SHA256)
    except OSError:
        verified = False
    if not verified:
        return [LogInfo(msg='Unverified house map. Keeping the supplied AMCL configuration.')]

    source = Path(LaunchConfiguration('params_file').perform(context))
    with source.open() as stream:
        params = yaml.safe_load(stream)
    amcl = params.setdefault('amcl', {}).setdefault('ros__parameters', {})
    # Replace either dotted or nested parameter spelling from the input YAML.
    pose = {coordinate: 0.0 for coordinate in ('x', 'y', 'z', 'yaw')}
    for coordinate in pose:
        amcl.pop(f'initial_pose.{coordinate}', None)
    amcl.update(set_initial_pose=True, initial_pose=pose)

    with NamedTemporaryFile(mode='w', prefix='lk1209_house_amcl_', suffix='.yaml',
                            delete=False) as output:
        atexit.register(Path(output.name).unlink, missing_ok=True)
        yaml.safe_dump(params, output, sort_keys=False)
    return [
        SetLaunchConfiguration('params_file', output.name),
        LogInfo(msg='Verified house map. AMCL will start at map (0, 0), yaw 0.'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            str(Path(get_package_share_directory('robot_navigation')) /
                'launch' / 'delivery.launch.py'))),
    ]


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
            'world',
            default_value='',
            description='Gazebo world selected by the simulation launcher'
        ),

        DeclareLaunchArgument(
            'params_file',
            default_value=params_file
        ),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true'
        ),

        DeclareLaunchArgument('nav_profile', default_value='classic',
                              choices=NAV_PROFILES),
        DeclareLaunchArgument('lidar_mode', default_value='2d', choices=['2d', '3d']),
        OpaqueFunction(function=select_profile),
        OpaqueFunction(function=configure_house_pose),
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
                'slam': 'False',
                'map': LaunchConfiguration('map'),
                'params_file': LaunchConfiguration('params_file'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'autostart': 'True',
                'use_composition': 'False'
            }.items()
        )
    ])

