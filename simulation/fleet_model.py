"""Prepare two instances of the installed LK1209 model and spawn them in one house."""
import argparse
from pathlib import Path
import subprocess
import time
import xml.etree.ElementTree as ET

import yaml


# Poses are in the saved house map. The map origin corresponds to world (-2, 0.45).
HOME = {'robot1': (0.0, 0.0), 'robot2': (5.10, 2.25)}
WORLD_OFFSET = (-2.0, 0.45)


def robot_model(xml, name):
    model = ET.fromstring(xml)
    model.set('name', name)
    prefix = f'/model/{name}'
    drive = model.find(".//plugin[@name='gz::sim::systems::DiffDrive']")
    sensor = model.find(".//sensor[@name='lidar_sensor']")
    if drive is None or sensor is None:
        raise ValueError('LK1209 model requires a DiffDrive plugin and lidar_sensor')
    for node, tag, value in [(drive, 'topic', 'cmd_vel'), (drive, 'odom_topic', 'odometry'),
                             (drive, 'tf_topic', 'tf'), (sensor, 'topic', 'scan')]:
        child = node.find(tag)
        if child is None:
            child = ET.SubElement(node, tag)
        child.text = f'{prefix}/{value}'

    plugin = ET.SubElement(ET.SubElement(model, 'gazebo'), 'plugin', {
        'filename': 'gz-sim-joint-state-publisher-system',
        'name': 'gz::sim::systems::JointStatePublisher'})
    ET.SubElement(plugin, 'topic').text = f'{prefix}/joint_states'
    ET.SubElement(plugin, 'update_rate').text = '20'
    for joint in ('left_wheel_joint', 'right_wheel_joint'):
        ET.SubElement(plugin, 'joint_name').text = joint
    return ET.tostring(model, encoding='unicode')


def namespace_topics(config, name):
    """Rewrite only the singleton robot topic names used by the source YAML."""
    mapping = {
        '/scan': 'scan', '/odom': 'odom', '/obstacle_points': 'obstacle_points',
        '/lk1209/sim/cmd_vel_checked': 'sim/cmd_vel_checked',
        '/lk1209/sim/cmd_vel_safe': 'sim/cmd_vel_safe',
        '/local_costmap/published_footprint': 'local_costmap/published_footprint',
    }
    if isinstance(config, dict):
        return {key: namespace_topics(value, name) for key, value in config.items()}
    if isinstance(config, list):
        return [namespace_topics(value, name) for value in config]
    if isinstance(config, str) and config in mapping:
        return f'/{name}/{mapping[config]}'
    return config


def prepare(run, config, model_path):
    xml = subprocess.check_output(['xacro', str(model_path)], text=True)
    nav = yaml.safe_load((config / 'nav2_params.yaml').read_text())
    safety = yaml.safe_load((config / 'sim_collision_monitor.yaml').read_text())
    for name, (x, y) in HOME.items():
        (run / f'{name}.urdf').write_text(robot_model(xml, name))
        params = namespace_topics(nav, name)
        amcl = params['amcl']['ros__parameters']
        amcl.update(set_initial_pose=True, initial_pose=dict(x=x, y=y, z=0.0, yaw=0.0))
        params['global_costmap']['global_costmap']['ros__parameters']['static_layer']['map_topic'] = f'/{name}/map'
        (run / f'{name}_nav.yaml').write_text(yaml.safe_dump(params, sort_keys=False))
        (run / f'{name}_safety.yaml').write_text(
            yaml.safe_dump({name: namespace_topics(safety, name)}, sort_keys=False))


def spawn(run):
    from gz.msgs10.boolean_pb2 import Boolean
    from gz.msgs10.empty_pb2 import Empty
    from gz.msgs10.entity_factory_pb2 import EntityFactory
    from gz.msgs10.scene_pb2 import Scene
    from gz.transport13 import Node

    transport = Node()
    for name, (x, y) in HOME.items():
        req = EntityFactory(name=name, sdf=(run / f'{name}.urdf').read_text())
        req.pose.position.x = x + WORLD_OFFSET[0]
        req.pose.position.y = y + WORLD_OFFSET[1]
        req.pose.position.z = 0.10
        req.pose.orientation.w = 1.0
        ok, msg = transport.request('/world/default/create', req, EntityFactory, Boolean, 12000)
        if not ok or not msg.data:
            raise RuntimeError(f'Gazebo rejected {name}')
    deadline = time.monotonic() + 30
    while time.monotonic() < deadline:
        ok, scene = transport.request('/world/default/scene/info', Empty(), Empty, Scene, 1500)
        if ok and set(HOME) <= {model.name for model in scene.model}:
            print('Gazebo spawned robot1 and robot2 in the same house.', flush=True)
            return
        time.sleep(.25)
    raise RuntimeError('Gazebo scene does not contain both robot models')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('config', type=Path)
    parser.add_argument('model', type=Path)
    args = parser.parse_args()
    prepare(args.run, args.config, args.model)
    spawn(args.run)
