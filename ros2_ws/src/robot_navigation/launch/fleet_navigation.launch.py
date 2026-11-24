"""Two independently localized Nav2 stacks in a shared TurtleBot3 House world."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, ExecuteProcess, GroupAction,
    IncludeLaunchDescription, OpaqueFunction,
)
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from lk1209_navigation.nav_profiles import NAV_PROFILES, profile_file


def start(context):
    run = Path(LaunchConfiguration('runtime_dir').perform(context))
    root = Path(LaunchConfiguration('project_root').perform(context))
    share = Path(get_package_share_directory('robot_navigation'))
    profile = LaunchConfiguration('nav_profile').perform(context)
    nav2 = Path(get_package_share_directory('nav2_bringup')) / 'launch/bringup_launch.py'
    tf = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    # Each row describes one Gazebo transport endpoint and its ROS endpoint.
    wires = [
        ('scan', 'scan', 'sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'),
        ('odometry', 'odom', 'nav_msgs/msg/Odometry[gz.msgs.Odometry'),
        ('tf', 'tf', 'tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'),
        ('joint_states', 'joint_states', 'sensor_msgs/msg/JointState[gz.msgs.Model'),
        ('cmd_vel', 'sim/cmd_vel_safe', 'geometry_msgs/msg/Twist]gz.msgs.Twist'),
    ]
    topics = ['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock']
    remaps = []
    actions = []
    for ns in ('robot1', 'robot2'):
        for gz_name, ros_name, spec in wires:
            topic = f'/model/{ns}/{gz_name}'
            topics.append(f'{topic}@{spec}')
            remaps.append((topic, f'/{ns}/{ros_name}'))
        safety = str(run / f'{ns}_safety.yaml')
        nav_params = profile_file(run / f'{ns}_nav.yaml', profile, share / 'config', ns)
        guard = ['--ros-args', '--params-file', safety, '-r', f'__ns:=/{ns}']
        for source, dest in tf + [('/scan', 'scan'), ('/cmd_vel', 'cmd_vel'),
                                  ('/lk1209/sim/cmd_vel_checked', 'sim/cmd_vel_checked')]:
            guard.extend(['-r', f'{source}:=/{ns}/{dest}'])
        actions.extend([
            Node(package='obstacle_detection', executable='lidar_detector',
                 namespace=ns, parameters=[{'use_sim_time': True}]),
            Node(package='robot_state_publisher', executable='robot_state_publisher',
                 namespace=ns, remappings=tf, output='screen',
                 parameters=[{'use_sim_time': True, 'robot_description': ParameterValue(
                     (run / f'{ns}.urdf').read_text(), value_type=str)}]),
            ExecuteProcess(cmd=['/usr/bin/python3', str(root / 'simulation/sim_cmd_vel_guard.py'),
                                str(share / 'config/nav2_params.yaml'), *guard],
                           name=f'{ns}_velocity_guard', output='screen',
                           on_exit=[EmitEvent(event=Shutdown(reason=f'{ns} velocity guard exited'))]),
            Node(package='nav2_collision_monitor', executable='collision_monitor',
                 namespace=ns, name='collision_monitor', remappings=tf,
                 parameters=[safety], output='screen',
                 on_exit=[EmitEvent(event=Shutdown(reason=f'{ns} collision monitor exited'))]),
            Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                 namespace=ns, name='lifecycle_manager_collision', output='screen',
                 parameters=[{'use_sim_time': True, 'autostart': True,
                              'node_names': ['collision_monitor']}]),
            GroupAction([IncludeLaunchDescription(PythonLaunchDescriptionSource(str(nav2)),
                launch_arguments={'namespace': ns, 'use_namespace': 'True', 'slam': 'False',
                                  'map': str(share / 'maps/turtlebot3_house_test.yaml'),
                                  'params_file': nav_params, 'autostart': 'True',
                                  'use_sim_time': 'True', 'use_composition': 'True'}.items())]),
        ])
    bridge = Node(package='ros_gz_bridge', executable='parameter_bridge',
                  name='fleet_bridge', arguments=topics, remappings=remaps, output='screen',
                  on_exit=[EmitEvent(event=Shutdown(reason='fleet bridge exited'))])
    delivery = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share / 'launch/delivery.launch.py')),
        launch_arguments={'fleet': 'true'}.items())
    return [bridge, *actions, delivery]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('project_root'), DeclareLaunchArgument('runtime_dir'),
        DeclareLaunchArgument('nav_profile', default_value='classic',
                              choices=NAV_PROFILES),
        OpaqueFunction(function=start),
    ])
