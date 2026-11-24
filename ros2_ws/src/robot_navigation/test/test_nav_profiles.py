"""Verify the algorithm selections against the actual navigation YAML."""
import copy
from pathlib import Path
import sys
import unittest

import yaml

PKG = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG / 'launch'))


class ProfileTest(unittest.TestCase):
    def setUp(self):
        self.assertTrue((PKG / 'launch/nav_profiles.py').is_file(), 'missing profile selection')
        from nav_profiles import apply_profile
        self.apply = apply_profile
        self.config = PKG / 'config'
        self.base = yaml.safe_load((self.config / 'nav2_params.yaml').read_text())

    def test_classic_returns_original_configuration(self):
        self.assertIs(self.apply(self.base, 'classic', self.config), self.base)

    def test_lk_astar_replaces_only_the_global_planner(self):
        original = copy.deepcopy(self.base)
        for ns in ('', 'robot1', 'robot2'):
            with self.subTest(namespace=ns):
                params = self.apply(self.base, 'lk_astar', self.config, ns)
                planner = params['planner_server']['ros__parameters']['GridBased']
                self.assertEqual(planner['plugin'], 'lk1209_astar_planner/FourNeighborAStar')
                self.assertNotIn('use_astar', planner)
                self.assertEqual(params['controller_server'], original['controller_server'])
                for key in ('local_costmap', 'global_costmap', 'slam_toolbox'):
                    self.assertEqual(params[key], original[key])
        self.assertEqual(self.base, original)

    def test_3d_profiles_mark_body_height_points_and_clear_using_full_rays(self):
        from nav_profiles import profile_file
        for profile in ('classic', 'smac_rpp', 'lk_astar'):
            with self.subTest(profile=profile):
                source = self.config / 'nav2_params.yaml'
                params = yaml.safe_load(Path(profile_file(
                    source, profile, self.config, lidar_mode='3d')).read_text())
                for key in ('local_costmap', 'global_costmap'):
                    costmap = params[key][key]['ros__parameters']
                    self.assertNotIn('obstacle_layer', costmap['plugins'])
                    self.assertIn('voxel_layer', costmap['plugins'])
                    layer = costmap['voxel_layer']
                    self.assertEqual(layer['obstacles']['topic'], '/lidar/obstacles')
                    self.assertEqual(layer['clearing_rays']['topic'], '/lidar/clearing')
                    self.assertTrue(layer['obstacles']['marking'])
                    self.assertFalse(layer['clearing_rays']['marking'])
                    self.assertTrue(layer['clearing_rays']['clearing'])
                self.assertEqual(params['amcl']['ros__parameters']['scan_topic'], '/scan')

    def test_smac_rpp_sensors_and_limits(self):
        original = copy.deepcopy(self.base)
        for ns in ('', 'robot1', 'robot2'):
            params = self.apply(self.base, 'smac_rpp', self.config, ns)
            controller = params['controller_server']['ros__parameters']['FollowPath']
            planner = params['planner_server']['ros__parameters']['GridBased']
            self.assertEqual(planner['plugin'], 'nav2_smac_planner/SmacPlanner2D')
            self.assertEqual(controller['plugin'], 'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController')
            self.assertTrue(controller['use_collision_detection'])
            self.assertNotIn('critics', controller)
            self.assertEqual(controller['desired_linear_vel'], params['velocity_smoother']['ros__parameters']['max_velocity'][0])
            for key in ('local_costmap', 'global_costmap'):
                layer = params[key][key]['ros__parameters']['obstacle_layer']
                self.assertIn('scan', layer['observation_sources'].split())
                self.assertEqual(layer['scan'], self.base[key][key]['ros__parameters']['obstacle_layer']['scan'])
                self.assertIn('features', layer['observation_sources'].split())
                cloud = layer['features']
                self.assertTrue(cloud['marking'])
                self.assertFalse(cloud['clearing'])
                self.assertEqual(cloud['topic'], f'/{ns}/obstacle_points' if ns else '/obstacle_points')
            self.assertEqual(params['slam_toolbox'], self.base['slam_toolbox'])
        self.assertEqual(self.base, original)

    def test_bad_selection_raises(self):
        with self.assertRaises(ValueError):
            self.apply(self.base, 'invalid', self.config)


if __name__ == '__main__':
    unittest.main()
