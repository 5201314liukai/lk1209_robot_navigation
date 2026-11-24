"""Apply a robot-specific Nav2 algorithm selection before lifecycle startup."""
import atexit
import copy
from pathlib import Path
from tempfile import NamedTemporaryFile

import yaml


NAV_PROFILES = ('classic', 'smac_rpp', 'lk_astar')


def apply_profile(base, profile, config_dir, namespace=''):
    if profile == 'classic':
        return base
    if profile not in NAV_PROFILES:
        raise ValueError(f"Unknown navigation profile {profile!r}; choose from {', '.join(NAV_PROFILES)}")
    params = copy.deepcopy(base)
    if profile == 'lk_astar':
        params['planner_server']['ros__parameters']['GridBased'] = {
            'plugin': 'lk1209_astar_planner/FourNeighborAStar',
        }
        return params

    with (Path(config_dir) / 'smac_rpp.yaml').open() as stream:
        settings = yaml.safe_load(stream)
    params['planner_server']['ros__parameters']['GridBased'] = settings['GridBased']
    rpp = settings['FollowPath']
    limits = params['velocity_smoother']['ros__parameters']
    inflation = params['local_costmap']['local_costmap']['ros__parameters']['inflation_layer']
    rpp.update(desired_linear_vel=limits['max_velocity'][0],
               rotate_to_heading_angular_vel=limits['max_velocity'][2],
               max_angular_accel=limits['max_accel'][2],
               cost_scaling_dist=inflation['inflation_radius'],
               inflation_cost_scaling_factor=inflation['cost_scaling_factor'])
    params['controller_server']['ros__parameters']['FollowPath'] = rpp
    # Costmap nodes live below /local_costmap and /global_costmap. Resolve the
    # cloud at the owning robot's root rather than relative to a costmap node.
    robot = namespace.strip('/')
    topic = f'/{robot}/obstacle_points' if robot else '/obstacle_points'
    for key in ('local_costmap', 'global_costmap'):
        layer = params[key][key]['ros__parameters']['obstacle_layer']
        sources = layer['observation_sources'].split()
        if 'features' not in sources:
            sources.append('features')
        layer['observation_sources'] = ' '.join(sources)
        scan = layer['scan']
        layer['features'] = dict(settings['features'], topic=topic,
                                 obstacle_min_range=scan['obstacle_min_range'],
                                 obstacle_max_range=scan['obstacle_max_range'])
    return params


def profile_file(source, profile, config_dir, namespace='', lidar_mode='2d'):
    if profile == 'classic' and lidar_mode == '2d':
        return str(source)
    with Path(source).open() as stream:
        params = apply_profile(yaml.safe_load(stream), profile, config_dir, namespace)
    if lidar_mode == '3d':
        with (Path(config_dir) / 'lidar_voxel_layer.yaml').open() as stream:
            voxel_layer = yaml.safe_load(stream)
        for key in ('local_costmap', 'global_costmap'):
            costmap = params[key][key]['ros__parameters']
            index = costmap['plugins'].index('obstacle_layer')
            costmap['plugins'][index] = 'voxel_layer'
            del costmap['obstacle_layer']
            costmap['voxel_layer'] = copy.deepcopy(voxel_layer)
    with NamedTemporaryFile(mode='w', prefix='lk1209_nav_', suffix='.yaml', delete=False) as out:
        atexit.register(Path(out.name).unlink, missing_ok=True)
        yaml.safe_dump(params, out, sort_keys=False)
        return out.name
