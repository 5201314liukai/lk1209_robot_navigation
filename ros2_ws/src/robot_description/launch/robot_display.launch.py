import atexit
from pathlib import Path
from tempfile import NamedTemporaryFile

import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def lidar_rviz_config(source):
    """Keep the navigation panels and controls, with the 3D scan visible on launch."""
    config = yaml.safe_load(Path(source).read_text(encoding='utf-8'))
    manager = config['Visualization Manager']
    display = {
        'Class': 'rviz_default_plugins/PointCloud2', 'Position Transformer': 'XYZ',
        'Style': 'Points', 'Size (Pixels)': 3, 'Decay Time': 0,
    }
    topic = {'Reliability Policy': 'Best Effort', 'Durability Policy': 'Volatile',
             'History Policy': 'Keep Last', 'Depth': 2}
    manager['Displays'][0:0] = [
        {**display, 'Name': '3D LiDAR (height)', 'Enabled': True, 'Value': True,
         'Topic': {**topic, 'Value': '/lidar/points'},
         'Color Transformer': 'AxisColor', 'Axis': 'Z', 'Use Fixed Frame': True},
        {**display, 'Name': 'Collision-height obstacles', 'Enabled': False, 'Value': False,
         'Topic': {**topic, 'Value': '/lidar/obstacles'}, 'Size (Pixels)': 6,
         'Color Transformer': 'FlatColor', 'Color': '255; 94; 45'},
    ]
    manager['Views']['Current'] = {
        'Class': 'rviz_default_plugins/Orbit', 'Name': 'Current View',
        'Target Frame': 'base_footprint', 'Distance': 5.0, 'Pitch': 0.55, 'Yaw': 2.45,
        'Focal Point': {'X': 0.0, 'Y': 0.0, 'Z': 0.15},
    }
    with NamedTemporaryFile(mode='w', encoding='utf-8', suffix='.rviz', delete=False) as out:
        atexit.register(Path(out.name).unlink, missing_ok=True)
        yaml.safe_dump(config, out, sort_keys=False, allow_unicode=True)
        return out.name


def start_rviz(context, config):
    path = config.perform(context)
    if LaunchConfiguration('lidar_mode').perform(context) == '3d':
        path = lidar_rviz_config(path)
    return [Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
                 arguments=['-d', path],
                 parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}])]


def generate_launch_description():

    # ==========================================
    # 1. 机器人 Xacro 模型路径
    # ==========================================

    xacro_file = PathJoinSubstitution([
        FindPackageShare('robot_description'),
        'urdf',
        'robot.urdf.xacro'
    ])

    robot_description = Command([
        'xacro ',
        xacro_file, ' lidar_mode:=', LaunchConfiguration('lidar_mode')
    ])

    # ==========================================
    # 2. RViz2 配置文件路径
    # ==========================================

    rviz_config = PathJoinSubstitution([
        FindPackageShare('robot_navigation'),
        'config',
        'rviz_nav2.rviz'
    ])

    # ==========================================
    # 3. 仿真时间参数
    # ==========================================

    use_sim_time = LaunchConfiguration('use_sim_time')

    # ==========================================
    # 4. Robot State Publisher
    # ==========================================

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }]
    )

    # ==========================================
    # 5. Joint State Publisher
    # ==========================================

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )

    # ==========================================
    # 6. RViz2
    # ==========================================

    rviz_node = OpaqueFunction(function=start_rviz, args=[rviz_config],
                               condition=IfCondition(LaunchConfiguration('rviz')))

    # ==========================================
    # 7. 返回 Launch Description
    # ==========================================

    return LaunchDescription([
        DeclareLaunchArgument('lidar_mode', default_value='2d', choices=['2d', '3d']),
        DeclareLaunchArgument('rviz', default_value='true'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true'
        ),

        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node

    ])

