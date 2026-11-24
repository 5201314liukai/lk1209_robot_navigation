#!/usr/bin/env python3
"""Measure real 3D laser, flat scan and local costmap responses to height fixtures."""
import argparse
from collections import deque
import json
import math
from pathlib import Path
import signal
import time
import uuid

import numpy as np

from test_fleet_delivery import GazeboPoses
from test_navigation_obstacle import StaticObstacle


DISTANCE, DEPTH, WIDTH = .82, .12, .42
CASES = [('low_block', 0., .09), ('low_bar', .115, .16), ('raised_bar', .285, .335)]


def xyz(msg):
    fields = {field.name: field for field in msg.fields}
    endian = '>' if msg.is_bigendian else '<'
    def field_view(name):
        field = fields[name]
        if field.datatype != 7 or field.count != 1:
            raise ValueError('Expected FLOAT32 position field ' + name)
        return np.ndarray((msg.height, msg.width), dtype=endian+'f4', buffer=msg.data,
                          offset=field.offset, strides=(msg.row_step, msg.point_step)).ravel()
    if msg.width * msg.height == 0:
        return np.empty((3, 0))
    return np.array([field_view(k) for k in ('x', 'y', 'z')])


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def ring_geometry(msg, points):
    """Read ring IDs even for beams that saw no surface within the 8 m range."""
    f = next((field for field in msg.fields if field.name=='ring'),None)
    types = {2:'u1',4:'u2',6:'u4'}
    if f is None or f.count != 1 or f.datatype not in types:
        raise ValueError('Organized Gazebo cloud missing unsigned ring IDs')
    dtype=('>' if msg.is_bigendian else '<')+types[f.datatype]
    ids=np.ndarray((msg.height,msg.width),dtype=dtype,buffer=msg.data,
                   offset=f.offset,strides=(msg.row_step,msg.point_step))
    finite=np.all(np.isfinite(points),axis=0)
    observed_ids=ids.ravel()[finite]
    finite_rings=[int(i) for i in np.unique(observed_ids)]
    expected_rings=list(range(16))
    calibrated=False
    max_error=None
    if len(finite_rings):
        x,y,z=points[:,finite]
        angles=np.degrees(np.arctan2(z,np.hypot(x,y)))
        max_error=float(np.max(abs(angles-(-20.+2.*observed_ids))))
        calibrated=max_error<=.20
    organized=(msg.height==16 and msg.width==360 and
               np.all(ids==np.arange(16)[:,None]))
    return dict(rows=int(msg.height),columns=int(msg.width),ring_ids=[int(i) for i in np.unique(ids)],
                finite_rings=finite_rings,empty_rings=sorted(set(expected_rings)-set(finite_rings)),
                distinct_elevations=len(finite_rings),max_elevation_error_deg=max_error,
                calibrated_16_rings=bool(organized and calibrated))


class Fixture(StaticObstacle):
    """Reuse the existing isolated Gazebo service client and scene queries."""
    def __init__(self, world, kind):
        self.world = world
        self.name = 'lk1209_lidar3d_' + kind + '_' + uuid.uuid4().hex
        self.requested = False

    def spawn(self, center, heading, bottom, top):
        from gz.msgs10.boolean_pb2 import Boolean
        from gz.msgs10.entity_factory_pb2 import EntityFactory
        geometry = f'<geometry><box><size>{DEPTH} {WIDTH} {top-bottom}</size></box></geometry>'
        sdf = (f'<sdf version="1.8"><model name="{self.name}"><static>true</static><link name="bar">'
               f'<collision name="collision">{geometry}</collision>'
               f'<visual name="visual">{geometry}</visual></link></model></sdf>')
        req = EntityFactory(name=self.name, sdf=sdf)
        req.pose.position.x, req.pose.position.y = center
        req.pose.position.z = (bottom+top)/2
        req.pose.orientation.w, req.pose.orientation.z = math.cos(heading/2), math.sin(heading/2)
        self.requested = True
        try:
            response = self.service('create', req, Boolean)
        except RuntimeError:
            response = None  # Verify world state if the reply was lost.
        if response is not None and not response.data:
            raise RuntimeError('Gazebo rejected fixture creation')
        self.await_model(True, 15)
        p = self.models()[self.name].pose.position
        if math.dist((p.x,p.y,p.z), (*center,(bottom+top)/2)) > .005:
            raise RuntimeError('Unexpected Gazebo fixture pose')

    def remove(self):
        from gz.msgs10.boolean_pb2 import Boolean
        from gz.msgs10.entity_pb2 import Entity
        if not self.requested:
            return
        for _ in range(2):
            model = self.models().get(self.name)
            if model is None:
                return
            req = Entity(id=model.id, name=self.name, type=Entity.MODEL)
            try:
                self.service('remove', req, Boolean)
            except RuntimeError:
                pass
        self.await_model(False, 15)


class Probe:
    def __init__(self, node, world):
        from nav_msgs.msg import Odometry, OccupancyGrid
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import LaserScan, PointCloud2
        from tf2_ros import Buffer, TransformListener
        self.node = node
        self.poses = GazeboPoses(world, ['lk1209'])
        self.buf = Buffer()
        self.listener = TransformListener(self.buf, node)
        self.latest = {}
        self.streams = {'raw':('/lidar/points',PointCloud2), 'obstacles':('/lidar/obstacles',PointCloud2),
                        'scan':('/scan',LaserScan), 'grid':('/local_costmap/costmap',OccupancyGrid),
                        'odom':('/odom',Odometry)}
        for key,(topic,typ) in self.streams.items():
            node.create_subscription(typ, topic, lambda m,k=key: self.latest.update({k:m}), qos_profile_sensor_data)

    def prepare(self):
        import rclpy
        from rclpy.time import Time
        deadline = time.monotonic()+45.
        while time.monotonic()<deadline:
            rclpy.spin_once(self.node, timeout_sec=.15)
            if (len(self.latest)==len(self.streams) and len(self.poses.trail('lk1209')) >= 3
                    and self.buf.can_transform('base_footprint','lidar_link',Time())):
                break
        if len(self.latest)!=len(self.streams):
            raise RuntimeError('Missing live streams: '+str(self.streams.keys()-self.latest.keys()))
        tf = self.buf.lookup_transform('base_footprint','lidar_link',Time()).transform
        if math.hypot(tf.translation.x,tf.translation.y)>.005 or abs(tf.rotation.w)<.9999:
            raise RuntimeError('Probe expects level, horizontally centered LK1209 lidar')
        self.sensor_height = tf.translation.z
        _,rx,ry,heading = self.poses.trail('lk1209')[-1]
        self.start = (rx,ry,heading)
        self.center = (rx+DISTANCE*math.cos(heading), ry+DISTANCE*math.sin(heading))
        if self.node.count_publishers('/scan') != 1:
            raise RuntimeError('Expected exactly one /scan publisher')
        for key in ('raw','obstacles','scan'):
            if self.latest[key].header.frame_id != 'lidar_link':
                raise ValueError(f'{key} has unexpected frame {self.latest[key].header.frame_id}')

    def rates(self):
        """Sample live arrivals while continuously spinning, before fixture operations."""
        import rclpy
        observed = {k:[] for k in ('raw','obstacles','scan')}
        deadline = time.monotonic()+60.
        while time.monotonic()<deadline:
            rclpy.spin_once(self.node,timeout_sec=.1)
            for k,v in observed.items():
                t = stamp(self.latest[k])
                if not v or t>v[-1][0]:
                    v.append((t,time.monotonic()))
            if all(len(v)>=20 for v in observed.values()):
                break
        result = {}
        for k,v in observed.items():
            if len(v)<3:
                raise RuntimeError('Missing distinct frame timestamps for '+k)
            # The first timestamp may be left over from the readiness probe.
            samples=v[1:]
            sim_duration=samples[-1][0]-samples[0][0]
            wall_duration=samples[-1][1]-samples[0][1]
            result[k]={'frames':len(samples),'simulation_hz':round((len(samples)-1)/sim_duration,3),
                       'wall_hz':round((len(samples)-1)/wall_duration,3)}
        return result

    def grid_cells(self, grid):
        odom = self.latest['odom']
        if grid.header.frame_id!='odom' or odom.header.frame_id!='odom':
            raise RuntimeError('Expected local costmap and pose expressed in odom')
        p = odom.pose.pose
        a = yaw(p.orientation)
        cx,cy = p.position.x+DISTANCE*math.cos(a), p.position.y+DISTANCE*math.sin(a)
        info = grid.info
        if abs(yaw(info.origin.orientation))>.001:
            raise RuntimeError('Expected axis-aligned grid')
        yy,xx = np.indices((info.height,info.width))
        dx = info.origin.position.x+(xx+.5)*info.resolution-cx
        dy = info.origin.position.y+(yy+.5)*info.resolution-cy
        # Sampling margin includes the cell intersecting the box's near surface.
        region = ((abs(dx*math.cos(a)+dy*math.sin(a))<DEPTH/2+info.resolution)
                  & (abs(-dx*math.sin(a)+dy*math.cos(a))<WIDTH/2+info.resolution))
        cells = np.asarray(grid.data).reshape(info.height,info.width)[region]
        if not len(cells) or np.any(cells<0):
            raise RuntimeError('Probe lies outside known local costmap coverage')
        return {'lethal_cells':int(np.sum(cells==100)), 'region_cells':int(len(cells)),
                'peak_cost':int(np.max(cells))}

    def measure(self, key, msg, bottom, top):
        if key=='grid':
            return self.grid_cells(msg)
        if key=='scan':
            a = msg.angle_min+np.arange(len(msg.ranges))*msg.angle_increment
            d = np.array(msg.ranges)
            d = np.where(np.isfinite(d) & (d>=msg.range_min) & (d<=msg.range_max),d,np.nan)
            return {'fixture_hits':int(np.sum((abs(d*np.cos(a)-DISTANCE)<DEPTH/2+.025)
                         & (abs(d*np.sin(a))<WIDTH/2+.02))), 'beams':len(d),
                    'finite_hits':int(np.sum(np.isfinite(d)))}
        points = xyz(msg)
        x,y,z = points[:,np.all(np.isfinite(points), axis=0)]
        z_world = z + self.sensor_height
        in_xy = (abs(x-DISTANCE)<DEPTH/2+.025) & (abs(y)<WIDTH/2+.02)
        # Exclude the floor stripe even when measuring a floor-mounted box.
        hits = np.sum(in_xy & (z_world>=max(.026,bottom-.018)) & (z_world<=top+.018))
        obs = {'fixture_hits':int(hits), 'finite_points':int(len(x))}
        if key=='raw':
            obs.update(ring_geometry(msg, points))
        return obs

    def phase(self, name, bottom, top, present, timeout):
        import rclpy
        world_time = self.poses.trail('lk1209')[-1][0]
        begin = max(world_time, *(stamp(m) for m in self.latest.values())) + .20
        stages = ('raw','obstacles','scan','grid')
        recent = {k:deque(maxlen=3 if k!='grid' else 2) for k in stages}
        seen = {}
        expected_mark = present and top <= .22
        deadline = time.monotonic()+timeout
        while time.monotonic()<deadline:
            rclpy.spin_once(self.node,timeout_sec=.15)
            _,wx,wy,wa = self.poses.trail('lk1209')[-1]
            dyaw = wa-self.start[2]
            if math.dist((wx,wy),self.start[:2])>.015 or abs(math.atan2(math.sin(dyaw),math.cos(dyaw)))>.018:
                raise RuntimeError('Robot moved during stationary test')
            for k in stages:
                msg=self.latest[k]
                t=stamp(msg)
                if t<begin or t==seen.get(k):
                    continue
                seen[k]=t
                row=self.measure(k,msg,bottom,top)
                if k=='raw':
                    good = (row['calibrated_16_rings'] and
                            (row['fixture_hits']>=3 if present else row['fixture_hits']==0))
                elif k=='obstacles':
                    good = row['fixture_hits']>=2 if expected_mark else row['fixture_hits']==0
                elif k=='scan':
                    good = row['beams']>=350 and row['finite_hits']>10 and row['fixture_hits']==0
                else:
                    good = row['lethal_cells']>=2 if expected_mark else row['lethal_cells']==0
                row.update(stamp_s=round(t,6), passed=bool(good))
                recent[k].append(row)
            if all(len(v)==v.maxlen and all(row['passed'] for row in v) for v in recent.values()):
                break
        report = {k:list(v) for k,v in recent.items()}
        passed = all(len(v)==v.maxlen and all(row['passed'] for row in v) for v in recent.values())
        print(name, 'PASS' if passed else 'FAIL', {k:v[-1] if v else {} for k,v in report.items()},flush=True)
        return dict(passed=passed, streams=report)


def main():
    import rclpy
    from rclpy.parameter import Parameter
    from rclpy.signals import SignalHandlerOptions
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('result',type=Path)
    parser.add_argument('--world',default='default')
    parser.add_argument('--timeout',type=float,default=65.,help='Wall seconds for each phase')
    args=parser.parse_args()
    if args.result.exists():
        parser.error('Choose a new result filename')
    args.result.parent.mkdir(parents=True,exist_ok=True)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    def interrupt(*_):
        raise KeyboardInterrupt()
    for sig in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP):
        signal.signal(sig,interrupt)
    node=rclpy.create_node('lk1209_cloud_acceptance',parameter_overrides=[Parameter('use_sim_time',value=True)])
    report={'passed':False,'phases':{},'stationary':True}
    try:
        probe=Probe(node,args.world)
        probe.prepare()
        report.update(robot_world_pose=probe.start,fixture_world_xy=probe.center,lidar_height_m=probe.sensor_height)
        report['rate_sample']=probe.rates()
        def phase(name,bottom,top,present):
            result=probe.phase(name,bottom,top,present,args.timeout)
            report['phases'][name]=result
            if not result['passed']:
                raise RuntimeError('Live measurements failed in '+name)
        phase('clear_before',0.,.335,False)
        for label,bottom,top in CASES:
            fixture=Fixture(args.world,label)
            try:
                fixture.spawn(probe.center,probe.start[2],bottom,top)
                phase(label,bottom,top,True)
            finally:
                fixture.remove()
            phase('clear_after_'+label,bottom,top,False)
        report['passed']=True
    except BaseException as exc:
        report['error']=repr(exc)
        raise
    finally:
        args.result.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
        node.destroy_node()
        rclpy.try_shutdown()


if __name__=='__main__':
    main()
