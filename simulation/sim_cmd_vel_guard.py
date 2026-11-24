"""Validate ROS velocity commands before forwarding them to the simulation bridge."""

import argparse
import math

import rclpy
import yaml
from directional_stop import DirectionalStop
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.utilities import remove_ros_args


def load_velocity_limits(path):
    with open(path, encoding="utf-8") as stream:
        params = yaml.safe_load(stream)["velocity_smoother"]["ros__parameters"]

    lower = params["min_velocity"]
    upper = params["max_velocity"]
    for values in (lower, upper):
        if len(values) != 3 or not all(
            type(value) in (int, float) and math.isfinite(value) for value in values
        ):
            raise ValueError("Velocity limits must contain three finite numbers (x, y, yaw)")
    if not all(lo <= 0 <= hi for lo, hi in zip(lower, upper)):
        raise ValueError("Each velocity range must include zero")
    return (lower[0], upper[0]), (lower[2], upper[2])


class SimCmdVelGuard(Node):
    def __init__(self, limits):
        super().__init__("sim_cmd_vel_guard")
        self.linear_range, self.yaw_range = limits
        self.output = self.create_publisher(Twist, "/lk1209/sim/cmd_vel_checked", 1)
        enabled = self.declare_parameter("directional_stop_enabled", False).value
        self.stop_zone = DirectionalStop(self) if enabled else None
        self.forwarding_motion = False
        if self.stop_zone:
            self.create_timer(0.1, self.check_scan)
        self.create_subscription(Twist, "/cmd_vel", self.forward_command, 1)
        self.get_logger().info(
            f"Checking linear.x in {self.linear_range} m/s and angular.z in {self.yaw_range} rad/s"
        )

    def check_scan(self):
        now = self.get_clock().now().nanoseconds
        if self.forwarding_motion and not self.stop_zone.fresh(now):
            self.output.publish(Twist())
            self.forwarding_motion = False

    def forward_command(self, command):
        linear, angular = command.linear, command.angular
        finite = all(math.isfinite(value) for value in (
            linear.x, linear.y, linear.z, angular.x, angular.y, angular.z
        ))
        if not finite:
            reason = "non-finite Twist component"
        elif not self.linear_range[0] <= linear.x <= self.linear_range[1]:
            reason = f"linear.x={linear.x} outside {self.linear_range} m/s"
        elif not self.yaw_range[0] <= angular.z <= self.yaw_range[1]:
            reason = f"angular.z={angular.z} outside {self.yaw_range} rad/s"
        else:
            if self.stop_zone:
                command = self.stop_zone.filter(command)
                self.forwarding_motion = bool(command.linear.x or command.angular.z)
            self.output.publish(command)
            return

        self.forwarding_motion = False
        self.output.publish(Twist())
        self.get_logger().warning(f"Rejected /cmd_vel, sent stop: {reason}", throttle_duration_sec=1.0)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("params_file", help="Navigation YAML containing velocity_smoother limits")
    args = parser.parse_args(remove_ros_args()[1:])
    limits = load_velocity_limits(args.params_file)
    rclpy.init()
    node = SimCmdVelGuard(limits)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
