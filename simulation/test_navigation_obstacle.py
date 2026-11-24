#!/usr/bin/env python3
"""Real fixed-box regression, run as part of the fleet delivery order."""
import csv
import math
import os
from pathlib import Path
import subprocess
import sys
import time
import uuid


BOX_X, BOX_Y = -.90, .37
BOX_HX, BOX_HY = .16, .17
ROBOT_HX, ROBOT_HY = .15, .135  # Including wheels; not the padded planning footprint.


def separation(x, y, yaw):
    """Largest separating-axis gap, a conservative signed planar distance in meters."""
    dx, dy = x-BOX_X, y-BOX_Y
    c, s = math.cos(yaw), math.sin(yaw)
    ac, ass = abs(c), abs(s)
    return max(abs(dx)-BOX_HX-ROBOT_HX*ac-ROBOT_HY*ass,
               abs(dy)-BOX_HY-ROBOT_HX*ass-ROBOT_HY*ac,
               abs(dx*c+dy*s)-ROBOT_HX-BOX_HX*ac-BOX_HY*ass,
               abs(-dx*s+dy*c)-ROBOT_HY-BOX_HX*ass-BOX_HY*ac)


def route_evidence(trail):
    from test_fleet_delivery import DOOR_X, HOME
    gaps = [separation(x, y, a) for _, x, y, a in trail]
    if len(gaps) < 2:
        raise RuntimeError('insufficient Gazebo poses to evaluate clearance')
    if min(gaps) <= 0:
        raise RuntimeError(f'robot/box envelopes intersected; signed gap {min(gaps):.4f} m')
    door = next((t for t, x, _, _ in trail if x > DOOR_X), None)
    if door is None:
        raise RuntimeError('robot never reached the east side of the doorway')
    home_x, home_y = HOME['robot1']
    # The unobstructed geometric reference is the direct line from home to the
    # doorway center, not a claim about the route a planner took on another run.
    direct_y = home_y + (BOX_X-home_x) * (.403-home_y)/(DOOR_X-home_x)
    if abs(direct_y - BOX_Y) >= BOX_HY:
        raise RuntimeError('box placement does not block the geometric reference')
    passages = {}
    for (t,x,y,_), (u,a,b,_) in zip(trail, trail[1:]):
        if (x < BOX_X) == (a < BOX_X):
            continue
        if u-t > .5 or math.hypot(a-x,b-y) > .14:
            raise RuntimeError('Gazebo pose gap too large at obstacle passage')
        stage = 'outbound' if a > x and u < door else 'returning' if a < x and t > door else None
        if stage is not None and stage not in passages:
            crossing_y = y + (BOX_X-x)*(b-y)/(a-x)
            if abs(crossing_y-BOX_Y) <= BOX_HY:
                raise RuntimeError('robot center crossed the box')
            passages[stage] = {'world_y_m': round(crossing_y, 4),
                               'offset_from_direct_line_m': round(crossing_y-direct_y, 4)}
    if len(passages) != 2:
        raise RuntimeError(f'missing physical outbound/return box passage: {passages}')
    return {'min_sampled_clearance_lower_bound_m': round(min(gaps), 5),
            'pose_samples': len(gaps), 'direct_line_y_m': round(direct_y, 4),
            'box_passages': passages}


class StaticObstacle:
    def __init__(self, test, world):
        from nav_msgs.msg import OccupancyGrid, Odometry
        from rclpy.qos import qos_profile_sensor_data

        self.test, self.world = test, world
        self.name = 'lk1209_acceptance_box_' + uuid.uuid4().hex
        self.creation_requested = False
        self.marked = False
        self.odom = None
        self.baseline = {'local': [], 'global': []}
        self.with_box = {'local': [], 'global': []}
        self.phase = self.baseline
        self.plugins = {}
        self.trace = None
        self.node = test.node
        self.node.create_subscription(Odometry, '/robot1/odom', self.on_odom, qos_profile_sensor_data)
        for layer in ['local', 'global']:
            self.node.create_subscription(OccupancyGrid, f'/robot1/{layer}_costmap/costmap',
                lambda msg, key=layer: self.on_grid(key, msg), qos_profile_sensor_data)

    def on_odom(self, msg):
        p = msg.pose.pose
        q = p.orientation
        self.odom = (p.position.x, p.position.y,
                     math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))

    def on_grid(self, layer, msg):
        if self.marked:
            return
        from test_fleet_delivery import HOME
        if layer == 'global' and msg.header.frame_id == 'map':
            cx, cy, yaw = BOX_X-HOME['robot1'][0], BOX_Y-HOME['robot1'][1], 0.
        elif layer == 'local' and msg.header.frame_id == 'odom' and self.odom:
            poses = self.test.poses.trail('robot1')
            if not poses:
                return
            _, x, y, a = poses[-1]
            ox, oy, oa = self.odom
            yaw = oa-a
            dx, dy = BOX_X-x, BOX_Y-y
            cx = ox + math.cos(yaw)*dx - math.sin(yaw)*dy
            cy = oy + math.sin(yaw)*dx + math.cos(yaw)*dy
        else:
            return
        # Before sending any motion command, count lethal cells in the known
        # box rectangle plus two grid cells for scan/map registration.
        info = msg.info
        q = info.origin.orientation
        if abs(q.z) > 1e-5 or abs(q.x)+abs(q.y) > 1e-5:
            raise RuntimeError('this house test requires axis-aligned costmap grids')
        r = info.resolution
        pad = 2*r
        half = math.hypot(BOX_HX+pad, BOX_HY+pad)
        gx, gy = info.origin.position.x, info.origin.position.y
        hits = 0
        for j in range(max(0, math.floor((cy-half-gy)/r)), min(info.height, math.ceil((cy+half-gy)/r))):
            for i in range(max(0, math.floor((cx-half-gx)/r)), min(info.width, math.ceil((cx+half-gx)/r))):
                if msg.data[j*info.width+i] != 100:
                    continue
                dx, dy = gx+(i+.5)*r-cx, gy+(j+.5)*r-cy
                if (abs(math.cos(yaw)*dx+math.sin(yaw)*dy) <= BOX_HX+pad and
                        abs(-math.sin(yaw)*dx+math.cos(yaw)*dy) <= BOX_HY+pad):
                    hits += 1
        t = msg.header.stamp.sec*1_000_000_000 + msg.header.stamp.nanosec
        samples = self.phase[layer]
        if not samples or samples[-1][0] != t:
            samples.append((t, hits))

    def service(self, name, req, response):
        # The Gazebo pose subscriber holds Python callbacks in this process.
        # A separate interpreter can block on Gazebo services without starving them.
        try:
            reply = subprocess.run([sys.executable, str(Path(__file__).resolve()), '--gz-request',
                                    self.world, name], input=req.SerializeToString(),
                                   capture_output=True, timeout=13, check=True)
        except subprocess.TimeoutExpired as exc:
            raise RuntimeError(f'Gazebo {name} service worker timed out') from exc
        except subprocess.CalledProcessError as exc:
            raise RuntimeError(f'Gazebo {name}: {exc.stderr.decode(errors="replace").strip()}') from exc
        return response.FromString(reply.stdout)

    def models(self):
        from gz.msgs10.empty_pb2 import Empty
        from gz.msgs10.scene_pb2 import Scene
        for attempt in range(2):
            try:
                return {m.name: m for m in self.service('scene/info', Empty(), Scene).model}
            except RuntimeError:
                if attempt == 1:
                    raise

    def await_model(self, present, seconds):
        deadline = time.monotonic() + seconds
        last = ''
        while time.monotonic() < deadline:
            try:
                if (self.name in self.models()) == present:
                    return
            except RuntimeError as exc:
                last = str(exc)
            time.sleep(.15)
        raise RuntimeError(f'Cannot confirm Gazebo model {self.name} present={present}. {last}')

    def check_plugins(self):
        from rcl_interfaces.srv import GetParameters
        for server, param, expected in [('planner_server', 'GridBased.plugin', 'SmacPlanner2D'),
                                        ('controller_server', 'FollowPath.plugin', 'RegulatedPurePursuitController')]:
            client = self.node.create_client(GetParameters, f'/robot1/{server}/get_parameters')
            try:
                if not client.wait_for_service(timeout_sec=8):
                    raise RuntimeError(f'{server} parameter service unavailable')
                f = client.call_async(GetParameters.Request(names=[param]))
                self.test.wait(f.done, f'{server} live plugin parameter', 12)
                value = f.result().values[0].string_value
                if value.split('/')[-1].split('::')[-1] != expected:
                    raise RuntimeError(f'{server} loaded {value!r}; expected {expected}')
                self.plugins[server] = value
            finally:
                self.node.destroy_client(client)
        def consumers():
            return {i.node_namespace.rstrip('/')+'/'+i.node_name for i in
                    self.node.get_subscriptions_info_by_topic('/robot1/obstacle_points')}
        expected = {f'/robot1/{layer}_costmap/{layer}_costmap' for layer in ('local', 'global')}
        self.test.wait(lambda: expected <= consumers(), 'both costmaps subscribing to /robot1/obstacle_points', 25)
        self.test.wait(lambda: self.node.count_publishers('/robot1/obstacle_points') == 1,
                       'one live robot1 obstacle point publisher', 12)

    def prepare(self):
        from gz.msgs10.boolean_pb2 import Boolean
        from gz.msgs10.entity_factory_pb2 import EntityFactory
        self.check_plugins()
        self.test.wait(lambda: all(len(samples) >= 2 for samples in self.baseline.values()),
                       'two unobstructed costmaps per layer', 50)
        models = self.models()
        if ('lk1209_test_box' in models or any(k.startswith('lk1209_acceptance_box_') for k in models)):
            raise RuntimeError('the house already has a test box; refusing to alter that fixture')
        for robot in ('robot1', 'robot2'):
            if robot in models:
                p = models[robot].pose.position
                # The enclosing circle of either car fits in a radius of 0.203 m.
                if math.hypot(max(abs(p.x-BOX_X)-BOX_HX, 0), max(abs(p.y-BOX_Y)-BOX_HY, 0)) < .24:
                    raise RuntimeError(f'{robot} overlaps the planned spawn area')
        box = '<geometry><box><size>0.32 0.34 0.40</size></box></geometry>'
        sdf = (f'<sdf version="1.8"><model name="{self.name}"><static>true</static>'
               f'<link name="box"><collision name="box">{box}</collision>'
               f'<visual name="box">{box}</visual></link></model></sdf>')
        req = EntityFactory(name=self.name, sdf=sdf)
        req.pose.position.x, req.pose.position.y, req.pose.position.z = BOX_X, BOX_Y, .2
        req.pose.orientation.w = 1.0
        self.creation_requested = True
        try:
            response = self.service('create', req, Boolean)
        except RuntimeError as exc:
            print(f'{exc}; checking the original fixture name in the scene.', file=sys.stderr)
            response = None
        if response is not None and not response.data:
            raise RuntimeError('Gazebo rejected the test box')
        # The server may have created the entity before an acknowledgement was
        # lost. Do not resend create, which could leave a second obstacle.
        self.await_model(True, 20)
        self.verify_box_pose()
        self.phase = self.with_box
        def increased():
            return all(len(s) >= 2 and all(n >= max(b for _, b in self.baseline[k])+2 for _, n in s[-2:])
                       for k, s in self.with_box.items())
        self.test.wait(increased, 'two fresh frames with new lethal box cells in both costmaps', 60)
        self.marked = True
        print('Fixed box visible in both live costmaps, starting the delivery.', flush=True)

    def verify_box_pose(self):
        pose = self.models()[self.name].pose
        p, q = pose.position, pose.orientation
        if (math.dist((p.x, p.y, p.z), (BOX_X, BOX_Y, .2)) > .001 or
                math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z) > .001):
            raise RuntimeError(f'Gazebo box pose differs from fixture: {pose}')

    def save(self, trail):
        logs = Path(__file__).resolve().parent / 'logs' / 'fleet'
        logs.mkdir(parents=True, exist_ok=True)
        self.trace = logs / f'{self.name}_poses.csv'
        with self.trace.open('w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(('sim_s','world_x','world_y','yaw','box_gap_lower_bound_m'))
            for t,x,y,a in trail:
                writer.writerow((t,x,y,a,separation(x,y,a)))

    def verify_route(self, trail):
        self.save(trail)
        self.verify_box_pose()
        return dict(route_evidence(trail), trajectory_csv=str(self.trace),
                    live_plugins=self.plugins, box_world_xy=[BOX_X,BOX_Y],
                    baseline_peak_lethal_cells={k:max(n for _,n in s) for k,s in self.baseline.items()},
                    box_lethal_cells={k:[n for _,n in s[-2:]] for k,s in self.with_box.items()})

    def cleanup(self):
        from gz.msgs10.boolean_pb2 import Boolean
        from gz.msgs10.entity_pb2 import Entity
        if not self.creation_requested:
            return
        try:
            if self.trace is None:
                self.save(self.test.poses.trail('robot1'))
        finally:
            models = self.models()
            if self.name in models:
                ent = Entity(id=models[self.name].id, name=self.name, type=Entity.MODEL)
                for attempt in range(2):
                    try:
                        self.service('remove', ent, Boolean)
                    except RuntimeError as exc:
                        print(f'{exc}; checking whether fixture was removed.', file=sys.stderr)
                    try:
                        self.await_model(False, 10)
                        return
                    except RuntimeError:
                        if attempt == 1:
                            raise
                        model = self.models().get(self.name)
                        if model is None:
                            return
                        if model.id != ent.id:
                            raise RuntimeError('Fixture entity ID changed; refusing to remove another model')


def service_worker(world, operation):
    # Keep stdout a binary protobuf channel even if Gazebo writes diagnostics.
    with os.fdopen(os.dup(sys.stdout.fileno()), 'wb') as output:
        os.dup2(sys.stderr.fileno(), sys.stdout.fileno())
        from gz.msgs10.boolean_pb2 import Boolean
        from gz.msgs10.empty_pb2 import Empty
        from gz.msgs10.entity_factory_pb2 import EntityFactory
        from gz.msgs10.entity_pb2 import Entity
        from gz.msgs10.scene_pb2 import Scene
        from gz.transport13 import Node
        types = {'scene/info': (Empty, Scene), 'create': (EntityFactory, Boolean),
                 'remove': (Entity, Boolean)}
        request, response = types[operation]
        node = Node()
        ok, reply = node.request(f'/world/{world}/{operation}', request.FromString(sys.stdin.buffer.read()),
                                 request, response, 6000)
        if not ok:
            raise RuntimeError('No response within six seconds')
        output.write(reply.SerializeToString())


if __name__ == '__main__':
    if len(sys.argv) == 4 and sys.argv[1] == '--gz-request':
        service_worker(sys.argv[2], sys.argv[3])
    else:
        from test_fleet_delivery import main
        main(['--case', 'fleet', '--static-obstacle', *sys.argv[1:]])
