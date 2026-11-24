#!/usr/bin/env python3
"""Record robot1's real ROS inputs/commands and match them to the cart's Gazebo poses."""
import argparse
import bisect
import csv
import gzip
import json
import math
import signal
import statistics
from collections import defaultdict
from pathlib import Path


def yaw(q):
    return math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z))


def record(directory):
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import OccupancyGrid, Odometry
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan

    directory.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = Node('door_cart_observer', parameter_overrides=[Parameter('use_sim_time', value=True)])
    stopped = False

    def stop(*_):
        nonlocal stopped
        stopped = True
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    def now():
        return node.get_clock().now().nanoseconds * 1e-9
    def stamp(h):
        return h.stamp.sec + h.stamp.nanosec * 1e-9

    with gzip.open(directory / 'door_ros.jsonl.gz', 'xt', compresslevel=1) as stream:
        def emit(kind, *values):
            stream.write(json.dumps([kind, *values], separators=(',', ':'))+'\n')
        def scan(m):
            # Preserve which individual rays can clear space without a hit.
            ranges = [r if math.isfinite(r) else ('+Inf' if r == math.inf else None)
                      for r in m.ranges]
            emit('scan', stamp(m.header), now(), m.header.frame_id, m.angle_min, m.angle_increment,
                 m.range_min, m.range_max, ranges, sum(r == math.inf for r in m.ranges))
        def grid(m):
            info = m.info
            emit('grid', stamp(m.header), now(), m.header.frame_id, info.resolution, info.width,
                 info.origin.position.x, info.origin.position.y, yaw(info.origin.orientation),
                 [i for i, cell in enumerate(m.data) if cell == 100])
        def odom(m):
            p = m.pose.pose
            emit('odom', stamp(m.header), now(), p.position.x, p.position.y, yaw(p.orientation),
                 m.twist.twist.linear.x, m.twist.twist.angular.z)
        for typ, topic, callback in [(LaserScan, 'scan', scan), (OccupancyGrid, 'local_costmap/costmap', grid),
                                     (Odometry, 'odom', odom)]:
            node.create_subscription(typ, '/robot1/'+topic, callback, qos_profile_sensor_data)
        for topic in ('cmd_vel_nav', 'cmd_vel', 'sim/cmd_vel_checked', 'sim/cmd_vel_safe'):
            node.create_subscription(Twist, '/robot1/'+topic,
                lambda m, name=topic: emit(name, now(), m.linear.x, m.angular.z), qos_profile_sensor_data)
        print('Recording robot1 scan, local costmap, odometry and velocity chain', flush=True)
        try:
            while rclpy.ok() and not stopped:
                rclpy.spin_once(node, timeout_sec=0.2)
        finally:
            node.destroy_node()
            rclpy.try_shutdown()


def facing(x, y, heading):
    """World vector expressed in a frame rotated by heading."""
    c, s = math.cos(heading), math.sin(heading)
    return c*x+s*y, -s*x+c*y


HX, HY = 0.19, 0.12


def ray_entry(ox, oy, dx, dy):
    near, far = 0.0, math.inf
    for origin, direction, half in ((ox, dx, HX), (oy, dy, HY)):
        if abs(direction) < 1e-10:
            if abs(origin) > half:
                return None
        else:
            a,b = sorted(((-half-origin)/direction,(half-origin)/direction))
            near,far=max(near,a),min(far,b)
            if far < near:
                return None
    return near


def matching_scan(scan, pose):
    _, t, recv, frame, a0, da, lo, hi, ranges, _ = scan
    if frame != 'lidar_link':
        raise ValueError('Unexpected scan frame: '+frame)
    # The LK1209 lidar joint is directly above base_link with zero horizontal offset and yaw.
    rx,ry,ra,cx,cy,ca = pose
    x,y = facing(rx-cx,ry-cy,ca)
    expected, hits = 0,0
    for i,r in enumerate(ranges):
        a = ra-ca+a0+i*da
        distance = ray_entry(x,y,math.cos(a),math.sin(a))
        if distance is not None and lo <= distance <= hi:
            expected += 1
            if isinstance(r,(int,float)) and lo <= r <= hi and abs(r-distance) <= 0.035:
                hits += 1
    return expected, hits


def grid_counts(grid, pose, odom, old):
    _,t,recv,frame,res,width,gx,gy,grid_yaw,cells = grid
    if frame != 'odom':
        raise ValueError('Unexpected local costmap frame: '+frame)
    rx,ry,ra,cx,cy,ca = pose
    _,ot,ort,ox,oy,oa,*_ = odom
    cur_hits, old_hits = 0, 0
    for i in cells:
        u,v = facing(((i%width)+.5)*res, ((i//width)+.5)*res,-grid_yaw)
        dx,dy = facing(gx+u-ox,gy+v-oy,oa-ra)
        wx,wy = rx+dx,ry+dy
        px,py = facing(wx-cx,wy-cy,ca)
        if abs(px)<=HX+.075 and abs(py)<=HY+.075 and min(abs(abs(px)-HX),abs(abs(py)-HY))<=.075:
            cur_hits+=1
        if old:
            px,py=facing(wx-old[0],wy-old[1],old[2])
            if abs(px)<HX and abs(py)<HY:
                old_hits+=1
    return cur_hits,old_hits


def closest(times, items, t):
    i=bisect.bisect_left(times,t)
    if not times:
        return None
    j=min((j for j in (i-1,i) if 0<=j<len(times)),key=lambda j:abs(times[j]-t))
    return items[j] if abs(times[j]-t)<=.08 else None


def bounds(values):
    if not values:
        return {'samples':0}
    return dict(samples=len(values),min=round(min(values),4),median=round(statistics.median(values),4),
                max=round(max(values),4))


def assess(directory):
    def read_csv(name):
        with (directory/name).open() as f:
            return list(csv.DictReader(f))
    poses=read_csv('poses.csv')
    times=[float(q['sim_s']) for q in poses]
    geometry=[tuple(float(q[k]) for k in ('r1_x','r1_y','r1_yaw','cart_x','cart_y','cart_yaw')) for q in poses]
    events={q['event']:float(q['sim_s']) for q in read_csv('events.csv')}
    if 'trigger' not in events or not times:
        raise ValueError('No measured crossing trigger or Gazebo model poses')
    trace=defaultdict(list)
    last_clear = events.get('reentry_clear', events.get('lane_clear', times[-1]))
    with gzip.open(directory/'door_ros.jsonl.gz','rt') as f:
        for line in f:
            row=json.loads(line)
            if events['trigger']-2.1 <= row[1] <= last_clear+20:
                trace[row[0]].append(row)
    odoms=sorted(trace['odom'],key=lambda q:q[1]); od_times=[q[1] for q in odoms]
    old=closest(times,geometry,events['occupy']) if 'occupy' in events else None
    series=[]
    for key in ('scan','grid'):
        for q in trace[key]:
            t=q[1]
            p=closest(times,geometry,t)
            if p is None:continue
            if key=='scan':
                expected,hits=matching_scan(q,p)
                series.append((t,key,hits,expected,None,q[2]-t))
            else:
                o=closest(od_times,odoms,t)
                if o is None:continue
                hits,former=grid_counts(q,p,o,old[3:] if old else None)
                series.append((t,key,hits,None,former if old else None,q[2]-t))
    with (directory/'sensor_match.csv').open('w',newline='') as f:
        w=csv.writer(f);w.writerow(['sim_s','source','matched_hits','geometric_beams','old_cart_cells','receive_age_sim_s'])
        w.writerows(sorted(series))
    phases={}
    edges=[('pre',events['trigger']-2),('enter',events['trigger']),('held',events.get('occupy')),
           ('depart',events.get('depart')),('after',events.get('lane_clear'))]
    if 'reentry_trigger' in events:
        edges.extend((phase, events.get(phase)) for phase in
                     ('reentry_trigger', 'reentry_occupy', 'reentry_depart', 'reentry_clear'))
    edges=[(k,t) for k,t in edges if t is not None]
    for i,(name,a) in enumerate(edges):
        b=edges[i+1][1] if i+1<len(edges) else times[-1]
        if name in ('after', 'reentry_clear'):
            b=min(b, a+3)
        stats={}
        for key in ('scan','grid'):
            matches=[v for v in series if v[1]==key and a<=v[0]<b]
            stats[key]=dict(raw_frames=sum(a<=q[1]<b for q in trace[key]), matched_pose_frames=len(matches),
                            frames_with_cart_hits=sum(row[2]>0 for row in matches),
                            peak_hits=max((row[2] for row in matches),default=0),
                            receive_age_sim_s=bounds([row[5] for row in matches]))
            if key=='scan':
                stats[key]['frames_with_geometric_rays']=sum(row[3]>0 for row in matches)
        for key in ('cmd_vel_nav','cmd_vel','sim/cmd_vel_checked','sim/cmd_vel_safe','odom'):
            vi=6 if key=='odom' else 2
            stats[key+'_mps']=bounds([r[vi] for r in trace[key] if a<=r[1]<b])
        phases[name]=stats
    report=dict(phases=phases,limits={'scan_surface_tolerance_m':0.035,'grid_surface_tolerance_m':0.075,
                     'max_nearest_pose_offset_sim_s':0.08,'grid_value':100},
                complete_phases=all(k in events for k in ('trigger','occupy','depart','lane_clear')))
    if 'lane_clear' in events and old:
        clear=events['lane_clear']
        next_entry=events.get('reentry_trigger', math.inf)
        samples=[q for q in series if q[1]=='grid' and clear<=q[0]<=clear+3
                 and q[0]<next_entry and q[4] is not None]
        report['former_cart_area_first_3s_after_clear']={
            'grid_samples':len(samples),'samples_with_lethal_cells':sum(q[4]>0 for q in samples),
            'max_lethal_cell_count':max((q[4] for q in samples),default=None)}
        # Distinguish resuming at a crawl from recovering sustained progress.
        stages = ('cmd_vel_nav', 'cmd_vel', 'sim/cmd_vel_checked', 'sim/cmd_vel_safe', 'odom')
        report['after_clear_velocity_bins'] = []
        for start_s in range(0, 20, 2):
            begin = clear + start_s
            if begin >= next_entry:
                break
            end = min(begin + 2, next_entry)
            row = {'start_s': start_s, 'end_s': round(end-clear, 4)}
            for topic in stages:
                values = [r[6 if topic == 'odom' else 2] for r in trace[topic]
                          if begin <= r[1] < end]
                row[topic] = round(statistics.median(values), 4) if values else None
            report['after_clear_velocity_bins'].append(row)
    start=events['trigger']
    end=events.get('lane_clear',times[-1])
    inputs=sorted(trace['cmd_vel'],key=lambda r:r[1]); cmd_times=[r[1] for r in inputs]
    pairs=[]
    for q in trace['sim/cmd_vel_safe']:
        if start<=q[1]<=end:
            raw=closest(cmd_times,inputs,q[1])
            if raw and raw[2]>.04:
                pairs.append((q[1],raw[2],q[2]))
    slower=[q for q in pairs if q[2]<.65*q[1]]
    report['velocity_chain_encounter']={
        'paired_forward_command_samples':len(pairs),'samples_below_65pct_input':len(slower),
        'first_slow_output_sim_s_after_trigger':round(slower[0][0]-start,3) if slower else None,
        'max_forward_reduction_mps':round(max((v-u for t,v,u in pairs),default=0),3) if pairs else None}
    (directory/'sensor_summary.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode',choices=['record','assess'])
    parser.add_argument('directory',type=Path)
    args=parser.parse_args()
    (record if args.mode=='record' else assess)(args.directory)
