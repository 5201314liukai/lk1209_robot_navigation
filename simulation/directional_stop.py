"""Laser-based directional braking for the LK1209 differential-drive simulator."""
import math

from geometry_msgs.msg import Twist
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import Buffer, TransformException, TransformListener


class DirectionalStop:
    """Yield to entering obstacles, then bound the command sent for trajectory checking."""

    def __init__(self, node):
        self.node = node
        self.length = node.declare_parameter("stop_length", 0.42).value
        self.half_width = node.declare_parameter("stop_half_width", 0.20).value
        self.turn_radius = node.declare_parameter("turn_clear_radius", 0.225).value
        self.yield_ns = int(node.declare_parameter("yield_seconds", 4.0).value * 1e9)
        self.timeout_ns = int(node.declare_parameter("scan_timeout", 0.4).value * 1e9)
        self.creep_speed = node.declare_parameter("creep_speed", 0.08).value
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, node)
        self.stamp = None
        self.blocked_since = [None, None]
        self.turn_clear = False
        node.create_subscription(LaserScan, "/scan", self.update, qos_profile_sensor_data)

    def fresh(self, now):
        return self.stamp is not None and 0 <= now - self.stamp <= self.timeout_ns

    def update(self, scan):
        stamp = Time.from_msg(scan.header.stamp)
        if self.stamp is not None and stamp.nanoseconds == self.stamp:
            return
        try:
            tf = self.buffer.lookup_transform("base_footprint", scan.header.frame_id, stamp).transform
        except TransformException:
            self.stamp = None
            self.blocked_since = [None, None]
            return
        q = tf.rotation
        xx, xy = 1-2*(q.y*q.y+q.z*q.z), 2*(q.x*q.y-q.z*q.w)
        yx, yy = 2*(q.x*q.y+q.z*q.w), 1-2*(q.x*q.x+q.z*q.z)
        counts = [0, 0]
        usable = 0
        radial_clear = True
        for i, r in enumerate(scan.ranges):
            if r == math.inf:
                usable += 1
                continue
            if not math.isfinite(r) or not scan.range_min <= r <= scan.range_max:
                radial_clear = False
                continue
            usable += 1
            theta = scan.angle_min + i*scan.angle_increment
            sx, sy = r*math.cos(theta), r*math.sin(theta)
            x = tf.translation.x + xx*sx + xy*sy
            y = tf.translation.y + yx*sx + yy*sy
            if abs(x) < self.length and abs(y) < self.half_width:
                counts[int(x < 0)] += 1
            if x*x + y*y <= self.turn_radius**2:
                radial_clear = False

        t = stamp.nanoseconds
        if not self.fresh(t):
            self.blocked_since = [None, None]
        if not usable:
            self.stamp = None
            self.blocked_since = [None, None]
            return
        self.stamp = t
        self.turn_clear = radial_clear and abs(scan.angle_increment)*len(scan.ranges) >= 2*math.pi-0.04
        for i,count in enumerate(counts):
            if count < 3:
                self.blocked_since[i] = None
            elif self.blocked_since[i] is None:
                self.blocked_since[i] = t

    def filter(self, command):
        now = self.node.get_clock().now().nanoseconds
        if not self.fresh(now):
            return Twist()
        if command.linear.x == 0.0 and command.angular.z != 0.0 and not self.turn_clear:
            return Twist()
        if command.linear.x != 0.0:
            since = self.blocked_since[int(command.linear.x < 0)]
        else:
            # Turning in place sweeps both ends of the robot.
            active = [t for t in self.blocked_since if t is not None]
            since = min(active) if active else None
        if since is None:
            return command
        output = Twist()
        if now - since < self.yield_ns or not self.turn_clear:
            return output
        # Retain Nav2's requested curvature. The downstream footprint approach
        # monitor checks this slow trajectory for collision with the actual scan.
        scale = min(1.0, self.creep_speed / abs(command.linear.x)) if command.linear.x else 1.0
        output.linear.x = command.linear.x * scale
        output.angular.z = command.angular.z * scale
        return output

