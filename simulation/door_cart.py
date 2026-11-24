#!/usr/bin/env python3
"""Move one Gazebo test cart across the west approach to the east-room door.

Run before submitting an outbound delivery. The owner waits for <outdir>/ready
and creates <outdir>/finish after the order. Pose/event CSVs use simulation
time. Both a normal finish and a catchable termination clean up this fixture.
"""
import argparse
import csv
import math
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time
import uuid


CART_X = 1.72
START_Y, LANE_Y, PARK_Y = .725, .403, .050
TRIGGER_X = CART_X - .75
HX, HY = .19, .12
LANE_HALF_WIDTH = .20
SPEED = .32
POSE_HEADER = ['sim_s', 'r1_x', 'r1_y', 'r1_yaw', 'r2_x', 'r2_y', 'r2_yaw',
               'cart_x', 'cart_y', 'cart_yaw']


def cart_sdf(name, topic):
    box = '<geometry><box><size>0.38 0.24 0.42</size></box></geometry>'
    # A velocity-controlled carriage moves on a horizontal guide. Its collision
    # geometry can contact the robot; the guide keeps it off the floor.
    return f'''<sdf version="1.8"><model name="{name}"><link name="cart">
    <gravity>false</gravity><inertial><mass>2</mass><inertia>
    <ixx>0.046</ixx><iyy>0.058</iyy><izz>0.034</izz></inertia></inertial>
    <collision name="body"><pose>0 0 .212 0 0 0</pose>{box}</collision>
    <visual name="body_visual"><pose>0 0 .212 0 0 0</pose>{box}
    <material><ambient>0.12 0.52 0.72 1</ambient><diffuse>0.12 0.52 0.72 1</diffuse></material></visual>
    </link><plugin filename="gz-sim-velocity-control-system" name="gz::sim::systems::VelocityControl">
    <topic>{topic}</topic></plugin></model></sdf>'''


def envelope_top(y, yaw):
    return y + HX * abs(math.sin(yaw)) + HY * abs(math.cos(yaw))


def pose_tuple(p):
    q = p.orientation
    return (p.position.x, p.position.y,
            math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z)))


class Reentry:
    """Observe robot1 independently and drive only the cart's return trip."""

    def __init__(self, cart):
        from gz.msgs10.laserscan_pb2 import LaserScan
        from gz.msgs10.odometry_pb2 import Odometry
        import yaml

        config = Path(__file__).resolve().parents[1] / 'ros2_ws/src/robot_navigation/config/sim_collision_monitor.yaml'
        guard = yaml.safe_load(config.read_text())['sim_cmd_vel_guard']['ros__parameters']
        self.length = guard['stop_length']
        self.half_width = guard['stop_half_width']
        self.odoms = self.scans = ()
        self.last_odom = self.stop_start = self.forward_start = None
        self.first_stop = self.second_stop = False
        self.clear_time = self.return_time = None
        if not cart.gz.subscribe(Odometry, '/model/robot1/odometry', self.read_odom):
            raise RuntimeError('Gazebo robot1 odometry subscription failed')
        if not cart.gz.subscribe(LaserScan, '/model/robot1/scan', self.read_scan):
            raise RuntimeError('Gazebo robot1 scan subscription failed')

    def read_odom(self, msg):
        stamp = msg.header.stamp
        self.odoms = (*self.odoms[-7:], (stamp.sec + stamp.nsec*1e-9, msg.twist.linear.x,
                                        msg.twist.angular.z))

    def read_scan(self, msg):
        # The LK1209 laser is directly above base_footprint with zero yaw.
        usable = count = 0
        for i, r in enumerate(msg.ranges):
            if r == math.inf:
                usable += 1
            elif math.isfinite(r) and msg.range_min <= r <= msg.range_max:
                usable += 1
                a = msg.angle_min + i*msg.angle_step
                if 0 <= r*math.cos(a) < self.length and abs(r*math.sin(a)) < self.half_width:
                    count += 1
        stamp = msg.header.stamp
        self.scans = (*self.scans[-2:], (stamp.sec + stamp.nsec*1e-9, bool(usable) and count < 3))

    def observe_motion(self, t, trigger):
        sample = next((item for item in reversed(self.odoms) if item[0] <= t), None)
        if sample is None:
            return
        stamp, v, w = sample
        if stamp < trigger or stamp == self.last_odom or not 0 <= t-stamp < .15:
            return
        if self.last_odom is None or stamp-self.last_odom > .15:
            self.stop_start = self.forward_start = None
        self.last_odom = stamp
        if abs(v) <= .005 and abs(w) <= .035:
            if self.stop_start is None:
                self.stop_start = stamp
            if stamp-self.stop_start >= .30:
                if self.return_time is None:
                    self.first_stop = True
                elif self.stop_start >= self.return_time:
                    self.second_stop = True
        else:
            self.stop_start = None
        if self.first_stop and self.clear_time is not None and stamp >= self.clear_time and v > .015:
            if self.forward_start is None:
                self.forward_start = stamp
        else:
            self.forward_start = None

    def can_return(self, t):
        scan = next((item for item in reversed(self.scans) if item[0] <= t), None)
        return (self.forward_start is not None and self.last_odom - self.forward_start >= .20
                and 0 <= t-self.last_odom < .15
                and scan is not None and 0 <= t-scan[0] < .15 and scan[1])


class Cart:
    def __init__(self, world, name):
        from gz.msgs10.pose_v_pb2 import Pose_V
        from gz.msgs10.twist_pb2 import Twist
        from gz.transport13 import Node

        self.world, self.name = world, name
        self.topic = '/model/' + name + '/velocity'
        self.gz = Node()
        self.lock = threading.Lock()
        self.row = None
        self.created = False
        self.twist = Twist
        self.pub = self.gz.advertise(self.topic, Twist)
        if not self.gz.subscribe(Pose_V, '/world/' + world + '/pose/info', self.sample):
            raise RuntimeError('Gazebo pose subscription failed')

    def sample(self, msg):
        names = ('robot1', 'robot2', self.name)
        poses = {p.name: pose_tuple(p) for p in msg.pose if p.name in names}
        if all(k in poses for k in names):
            t = msg.header.stamp.sec + msg.header.stamp.nsec*1e-9
            with self.lock:
                self.row = (t, *poses['robot1'], *poses['robot2'], *poses[self.name])

    def request(self, op, request, replytype):
        helper = Path(__file__).with_name('test_navigation_obstacle.py')
        result = subprocess.run([sys.executable, str(helper), '--gz-request', self.world, op],
                                input=request.SerializeToString(), capture_output=True, timeout=14)
        if result.returncode:
            raise RuntimeError(result.stderr.decode(errors='replace')[-500:])
        return replytype.FromString(result.stdout)

    def models(self):
        from gz.msgs10.empty_pb2 import Empty
        from gz.msgs10.scene_pb2 import Scene
        for attempt in range(2):
            try:
                scene = self.request('scene/info', Empty(), Scene)
                return {m.name: m for m in scene.model}
            except (RuntimeError, subprocess.TimeoutExpired):
                if attempt:
                    raise

    def spawn(self):
        from gz.msgs10.boolean_pb2 import Boolean
        from gz.msgs10.entity_factory_pb2 import EntityFactory
        models = self.models()
        if self.name in models or not {'robot1', 'robot2'} <= models.keys():
            raise RuntimeError('Expected both delivery robots and a fresh fixture name')
        for model in models.values():
            if model.name.startswith('lk1209_door_cart_'):
                raise RuntimeError('A prior test cart is still in the world: ' + model.name)
        req = EntityFactory(name=self.name, sdf=cart_sdf(self.name, self.topic))
        req.pose.position.x, req.pose.position.y = CART_X, START_Y
        req.pose.orientation.w = 1.
        self.created = True
        try:
            ack = self.request('create', req, Boolean)
        except (RuntimeError, subprocess.TimeoutExpired) as e:
            print(f'Checking unique fixture after lost create response: {e}', flush=True)
        else:
            if not ack.data:
                raise RuntimeError('Gazebo rejected the cart')
        until = time.monotonic() + 25.
        while time.monotonic() < until:
            # Not all Transport 13 Python bindings expose publisher discovery.
            connected = getattr(self.pub, 'has_connections', None)
            if self.row is not None and (connected is None or connected()):
                return
            time.sleep(.04)
        raise RuntimeError('No common robot/cart poses or velocity subscriber')

    def speed(self, value):
        v = self.twist()
        v.linear.y = value
        if not self.pub.publish(v):
            raise RuntimeError('Cart speed publication failed')

    def cleanup(self):
        from gz.msgs10.boolean_pb2 import Boolean
        from gz.msgs10.entity_pb2 import Entity
        if not self.created:
            return
        try:
            self.speed(0.)
        finally:
            model = self.models().get(self.name)
            if model is not None:
                entity = Entity(name=self.name, id=model.id, type=Entity.MODEL)
                for _ in range(3):
                    try:
                        self.request('remove', entity, Boolean)
                    except (RuntimeError, subprocess.TimeoutExpired):
                        pass
                    remaining = self.models().get(self.name)
                    if remaining is None:
                        return
                    if remaining.id != entity.id:
                        raise RuntimeError('Fixture entity ID changed during cleanup')
                    time.sleep(.3)
                raise RuntimeError('Gazebo still reports ' + self.name)


def capture(cart, out, reentry=None):
    with (out/'poses.csv').open('x', buffering=1, newline='') as pf, \
            (out/'events.csv').open('x', buffering=1, newline='') as ef:
        poses, events = csv.writer(pf), csv.writer(ef)
        poses.writerow(POSE_HEADER)
        events.writerow(['sim_s','event', *POSE_HEADER[1:]])
        stage, last, hold = 'arming', -1., None
        trigger = None
        motion_deadline = None
        stable_y, stable_since = None, None
        last_data = time.monotonic()
        wall_limit = last_data + 1800
        while not (out/'finish').exists():
            now = time.monotonic()
            if now > wall_limit or now-last_data > 20:
                raise TimeoutError('Cart test timed out waiting for live poses or order finish')
            with cart.lock:
                row = cart.row
            if row is None or row[0] - last < .019:
                time.sleep(.008)
                continue
            t, rx, _, _, _, _, _, cx, cy, yaw = row
            poses.writerow(row)
            last, last_data = t, now
            if reentry is not None and trigger is not None:
                reentry.observe_motion(t, trigger)
            if motion_deadline is not None and t > motion_deadline:
                raise RuntimeError('Cart did not complete physical motion in stage ' + stage)

            def log(label):
                events.writerow([t, label, *row[1:]])
                print(label, f't={t:.3f} robot1_x={rx:.3f} cart=({cx:.3f}, {cy:.3f})', flush=True)

            if stage == 'arming':
                if math.hypot(cx-CART_X, cy-START_Y) > .01 or rx >= TRIGGER_X:
                    raise RuntimeError('Unusable spawn or first robot past trigger')
                log('ready')
                (out/'ready').write_text(cart.name + '\n')
                stage = 'waiting'
            elif stage == 'waiting' and rx >= TRIGGER_X:
                cart.speed(-SPEED)
                log('trigger')
                trigger = t
                stage = 'crossing'
                motion_deadline = t + 4.
            elif stage == 'crossing' and cy <= LANE_Y:
                cart.speed(0.)
                stable_y, stable_since = cy, t
                log('brake')
                stage = 'settling'
            elif stage == 'settling':
                if abs(cy-stable_y) > .001:
                    stable_y, stable_since = cy, t
                elif t - stable_since >= .15:
                    if abs(cy-LANE_Y) > .025:
                        raise RuntimeError('Cart failed to stop on the requested lane')
                    log('occupy')
                    stage, hold = 'holding', t
                    motion_deadline = None
            elif stage == 'holding':
                if abs(cy-stable_y) > .005:
                    raise RuntimeError('Cart moved during its measured hold')
                if t-hold >= 3.10:
                    cart.speed(-SPEED)
                    log('depart')
                    stage = 'clearing'
                    motion_deadline = t + 4.
            elif stage == 'clearing' and envelope_top(cy, yaw) <= LANE_Y-LANE_HALF_WIDTH:
                log('lane_clear')
                if reentry is not None:
                    reentry.clear_time = t
                stage = 'exiting'
            elif stage == 'exiting' and cy <= PARK_Y:
                cart.speed(0.)
                log('park')
                stage = 'await_restart' if reentry is not None else 'parked'
                motion_deadline = t + 35. if reentry is not None else None
            elif stage == 'await_restart':
                heading = row[3]
                nose_x = rx + .15*abs(math.cos(heading)) + .135*abs(math.sin(heading))
                if nose_x >= cx - HX:
                    raise RuntimeError('Robot1 reached cart crossing before a clear, moving re-entry trigger')
                if reentry.can_return(t):
                    cart.speed(SPEED)
                    log('reentry_trigger')
                    reentry.return_time = t
                    stage, motion_deadline = 'return_to_center', t + 4.
            elif stage == 'return_to_center' and cy >= LANE_Y:
                cart.speed(0.)
                stage, stable_y, stable_since = 'settle_again', cy, t
            elif stage == 'settle_again':
                if abs(cy-stable_y) > .001:
                    stable_y, stable_since = cy, t
                elif t-stable_since >= .15:
                    if abs(cy-LANE_Y) > .025:
                        raise RuntimeError('Returning cart did not stop at lane center')
                    log('reentry_occupy')
                    stage, hold, motion_deadline = 'hold_again', t, None
            elif stage == 'hold_again':
                if abs(cy-stable_y) > .005:
                    raise RuntimeError('Returning cart moved during its hold')
                # Always allow the order to finish; the assessor judges braking independently.
                if (t-hold >= 3.10 and reentry.second_stop) or t-hold >= 12.:
                    cart.speed(SPEED)
                    log('reentry_depart')
                    stage, motion_deadline = 'exit_north', t + 4.
            elif stage == 'exit_north' and -envelope_top(-cy, yaw) >= LANE_Y+LANE_HALF_WIDTH:
                log('reentry_clear')
                cart.speed(0.)
                stage, stable_y, stable_since = 'park_north', cy, t
            elif stage == 'park_north':
                if abs(cy-stable_y) > .001:
                    stable_y, stable_since = cy, t
                elif t-stable_since >= .15:
                    log('reentry_park')
                    stage, motion_deadline = 'parked', None
        if stage != 'parked':
            raise RuntimeError('Order finished before cart parked; last stage ' + stage)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('outdir', type=Path)
    parser.add_argument('--world', default='default')
    parser.add_argument('--reentry', action='store_true', help='Return across the lane after robot1 restarts')
    args = parser.parse_args()
    out = args.outdir
    out.mkdir(parents=True, exist_ok=True)
    if any((out / f).exists() for f in ('finish','ready','done','poses.csv','events.csv','error')):
        raise RuntimeError('Use a fresh output directory for each trial')
    def interrupted(*_):
        raise KeyboardInterrupt()
    for sig in (signal.SIGTERM, signal.SIGINT, signal.SIGHUP):
        signal.signal(sig, interrupted)
    cart = Cart(args.world, 'lk1209_door_cart_' + uuid.uuid4().hex)
    try:
        try:
            cart.spawn()
            reentry = Reentry(cart) if args.reentry else None
            capture(cart, out, reentry)
        finally:
            cart.cleanup()
    except BaseException as exc:
        (out/'error').write_text(repr(exc) + '\n')
        raise
    (out/'done').write_text('Fixture removed\n')


if __name__ == '__main__':
    main()
