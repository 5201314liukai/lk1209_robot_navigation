#!/usr/bin/env python3
"""Verify two live obstacle extractors against their own timestamped Gazebo scans."""
import argparse
import json
import math
import statistics
import time


def stamp(header):
    return header.stamp.sec * 1_000_000_000 + header.stamp.nanosec


def measure(scan, segments, tolerance):
    if not scan.header.frame_id or scan.header.frame_id != segments.header.frame_id:
        raise ValueError(f'frame mismatch {scan.header.frame_id!r} / {segments.header.frame_id!r}')
    if stamp(scan.header) != stamp(segments.header):
        raise ValueError('scan and segments have different timestamps')
    if len(segments.starts) != len(segments.ends):
        raise ValueError('unequal start and end array lengths')
    points = []
    for i, radius in enumerate(scan.ranges):
        if math.isfinite(radius) and scan.range_min <= radius <= scan.range_max:
            angle = scan.angle_min + i * scan.angle_increment
            points.append((radius * math.cos(angle), radius * math.sin(angle)))
    if not points or not segments.starts:
        raise ValueError('expected measured surfaces and at least one segment in the house')

    worst = 0.0
    for i, (a, b) in enumerate(zip(segments.starts, segments.ends)):
        for p in (a, b):
            if not all(math.isfinite(v) for v in (p.x, p.y, p.z)) or abs(p.z) > 1e-5:
                raise ValueError(f'segment {i} has a nonfinite or nonplanar endpoint')
            error = min(math.hypot(x - p.x, y - p.y) for x, y in points)
            if error > tolerance:
                raise ValueError(f'segment {i} endpoint ({p.x:.3f}, {p.y:.3f}) is '
                                 f'{error:.3f} m from its nearest measured return')
            worst = max(error, worst)
        dx, dy = b.x - a.x, b.y - a.y
        length = math.hypot(dx, dy)
        if length < 1e-6:
            raise ValueError(f'segment {i} is degenerate')
        # A longer line connecting two unrelated nearby surfaces can have two
        # plausible endpoints. Demand at least one return inside its middle 80%.
        if length > 2 * tolerance and not any(
            abs((x-a.x)*dy - (y-a.y)*dx) <= tolerance * length and
            0.1*length**2 <= (x-a.x)*dx + (y-a.y)*dy <= 0.9*length**2
            for x, y in points
        ):
            raise ValueError(f'segment {i} lacks interior scan support over {length:.3f} m')
    return len(segments.starts), worst


class RobotAudit:
    def __init__(self, node, name, required, tolerance):
        from lk1209_interfaces.msg import ObstacleSegments
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import LaserScan

        self.node = node
        self.name = name
        self.required = required
        self.tolerance = tolerance
        self.pending = {'scan': {}, 'obstacle_segments': {}}
        self.seen = set()
        self.counts = []
        self.worst = 0.0
        self.lags = []
        for topic, typ in [('scan', LaserScan), ('obstacle_segments', ObstacleSegments)]:
            node.create_subscription(typ, f'/{name}/{topic}',
                                     lambda msg, key=topic: self.receive(key, msg),
                                     qos_profile_sensor_data)

    def receive(self, topic, msg):
        key = stamp(msg.header)
        if len(self.seen) >= self.required or key in self.seen:
            return
        now = self.node.get_clock().now().nanoseconds
        if key <= 0 or now <= 0:
            return
        cache = self.pending[topic]
        cache[key] = (msg, now)
        if len(cache) > 128:
            cache.pop(next(iter(cache)))
        scans, outputs = self.pending['scan'], self.pending['obstacle_segments']
        if key not in scans or key not in outputs:
            return
        scan, _ = scans.pop(key)
        segments, receive_time = outputs.pop(key)
        try:
            count, worst = measure(scan, segments, self.tolerance)
        except ValueError as error:
            raise ValueError(f'{self.name} stamp {key}: {error}') from error
        self.seen.add(key)
        self.counts.append(count)
        self.worst = max(self.worst, worst)
        self.lags.append((receive_time - key) / 1e9)

    def report(self):
        result = {'matched_stamps': len(self.seen), 'segments': sum(self.counts),
                  'max_endpoint_error_m': round(self.worst, 4)}
        if self.lags:
            result['sim_receive_age_s'] = {
                'min': round(min(self.lags), 4), 'median': round(statistics.median(self.lags), 4),
                'max': round(max(self.lags), 4),
            }
        return result

    def check_publisher(self):
        ns = '/' + self.name
        pubs = self.node.get_publishers_info_by_topic(ns + '/obstacle_segments')
        if len(pubs) != 1 or pubs[0].node_namespace != ns:
            raise ValueError(f'{ns}/obstacle_segments needs exactly one publisher in {ns}; '
                             f'got {[(p.node_namespace, p.node_name) for p in pubs]}')
        publisher = (ns, pubs[0].node_name)
        for robot, expected in [('robot1', self.name == 'robot1'), ('robot2', self.name == 'robot2')]:
            subs = self.node.get_subscriptions_info_by_topic(f'/{robot}/scan')
            present = any((s.node_namespace, s.node_name) == publisher for s in subs)
            if present != expected:
                raise ValueError(f'{publisher} has unexpected scan subscription state on /{robot}/scan')
        return f'{ns}/{publisher[1]}'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--frames', type=int, default=12, help='unique matching stamps per car (>=10)')
    parser.add_argument('--timeout', type=float, default=90.0, help='wall seconds')
    parser.add_argument('--tolerance', type=float, default=.08,
                        help='max endpoint / measured return distance in meters (default 0.08)')
    args = parser.parse_args()
    if args.frames < 10 or not all(math.isfinite(v) and v > 0 for v in (args.timeout, args.tolerance)):
        parser.error('frames must be at least 10; timeout and tolerance must be finite and positive')

    import rclpy
    from rclpy.parameter import Parameter

    rclpy.init()
    node = rclpy.create_node('fleet_lidar_acceptance',
                             parameter_overrides=[Parameter('use_sim_time', value=True)])
    cars = [RobotAudit(node, name, args.frames, args.tolerance) for name in ('robot1', 'robot2')]
    result = {'passed': False, 'tolerance_m': args.tolerance}
    try:
        deadline = time.monotonic() + args.timeout
        while any(len(c.seen) < args.frames for c in cars):
            if time.monotonic() >= deadline:
                raise TimeoutError('timed out waiting for matching scan/segment stamps')
            rclpy.spin_once(node, timeout_sec=0.2)
        result['publishers'] = {c.name: c.check_publisher() for c in cars}
        result['passed'] = True
    except (ValueError, TimeoutError) as error:
        result['error'] = str(error)
    finally:
        result['robots'] = {c.name: c.report() for c in cars}
        node.destroy_node()
        rclpy.shutdown()
        print(json.dumps(result, allow_nan=False), flush=True)
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
