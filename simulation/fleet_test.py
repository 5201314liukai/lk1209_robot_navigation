"""Regression tests for isolating the two robot models and safety paths."""
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import yaml

from fleet_model import namespace_topics, robot_model, HOME


ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / 'ros2_ws/src/robot_navigation/config'


class FleetIsolation(unittest.TestCase):
    def test_models_have_disjoint_drive_scan_and_transform_topics(self):
        # The substitutions in the project xacro do not affect these ROS/Gazebo tags.
        source = (ROOT / 'ros2_ws/src/robot_description/urdf/robot.urdf.xacro').read_text(encoding='utf-8')
        endpoints = []
        for robot in HOME:
            model = ET.fromstring(robot_model(source, robot))
            drive = model.find(".//plugin[@name='gz::sim::systems::DiffDrive']")
            points = {drive.findtext(k) for k in ('topic', 'odom_topic', 'tf_topic')}
            points.add(model.findtext('.//sensor/topic'))
            self.assertTrue(all(p.startswith('/model/' + robot + '/') for p in points))
            endpoints.append(points)
        self.assertTrue(endpoints[0].isdisjoint(endpoints[1]))

    def test_sensor_guard_and_monitor_follow_the_same_robot(self):
        source = yaml.safe_load((CONFIG / 'sim_collision_monitor.yaml').read_text(encoding='utf-8'))
        for robot in HOME:
            monitor = namespace_topics(source, robot)['collision_monitor']['ros__parameters']
            self.assertEqual(monitor['scan']['topic'], f'/{robot}/scan')
            self.assertEqual(monitor['cmd_vel_in_topic'], f'/{robot}/sim/cmd_vel_checked')
            self.assertEqual(monitor['cmd_vel_out_topic'], f'/{robot}/sim/cmd_vel_safe')
            self.assertEqual(monitor['FootprintApproach']['footprint_topic'],
                             f'/{robot}/local_costmap/published_footprint')


if __name__ == '__main__':
    unittest.main()
