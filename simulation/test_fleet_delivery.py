#!/usr/bin/env python3
"""Exercise the house delivery action against a live Gazebo Harmonic world."""
import argparse
import json
import math
import signal
import sys
import threading
import time
from dataclasses import dataclass, field


# TurtleBot3 House Wall_91. Its 0.90 m door is centered at world y=0.403.
DOOR_X = 2.29994
HOME = {"lk1209": (-2.0, 0.45), "robot1": (-2.0, 0.45), "robot2": (3.10, 2.70)}
EAST = (3.15, 1.20)


def require(ok, message):
    if not ok:
        raise RuntimeError(message)


def door_crossings(samples):
    crossings = []
    for (t, x, y, _), (t2, x2, y2, _) in zip(samples, samples[1:]):
        if (x < DOOR_X) == (x2 < DOOR_X):
            continue
        if t2 - t > 0.5 or math.hypot(x2 - x, y2 - y) > 0.14:
            continue
        y_at_door = y + (y2 - y) * (DOOR_X - x) / (x2 - x)
        if abs(y_at_door - 0.403) < 0.45:
            crossings.append(1 if x2 > x else -1)
    return crossings


def is_still(samples):
    if not samples or samples[-1][0] - samples[0][0] < 1.0:
        return False
    _, x, y, yaw = samples[0]
    for before, point in zip(samples, samples[1:]):
        t, px, py, heading = point
        dyaw = math.atan2(math.sin(heading - yaw), math.cos(heading - yaw))
        if t - before[0] > 0.4 or math.hypot(px - x, py - y) > 0.015 or abs(dyaw) > 0.05:
            return False
    return True


class GazeboPoses:
    def __init__(self, world, names):
        from gz.msgs10.pose_v_pb2 import Pose_V
        from gz.transport13 import Node

        self.samples = {name: [] for name in names}
        self.mapping = {("lk1209_robot" if r == "lk1209" else r): r for r in names}
        self.lock = threading.Lock()
        self.node = Node()
        topic = f"/world/{world}/pose/info"
        require(self.node.subscribe(Pose_V, topic, self.update), f"Cannot subscribe to Gazebo {topic}")

    def update(self, message):
        stamp = message.header.stamp.sec + message.header.stamp.nsec * 1e-9
        with self.lock:
            for p in message.pose:
                robot = self.mapping.get(p.name)
                if not robot:
                    continue
                trail = self.samples[robot]
                if trail and stamp - trail[-1][0] < 0.05:
                    continue
                q = p.orientation
                yaw = math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))
                trail.append((stamp, p.position.x, p.position.y, yaw))

    def trail(self, robot):
        with self.lock:
            return list(self.samples[robot])


@dataclass
class Order:
    goal_id: object
    handle: object = None
    completed: object = None
    stages: list = field(default_factory=list)
    robot: str = ""
    distances: list = field(default_factory=list)


class Acceptance:
    def __init__(self, args):
        import rclpy
        from action_msgs.msg import GoalStatusArray
        from action_msgs.srv import CancelGoal
        from lk1209_interfaces.action import DeliveryTask
        from rclpy.action import ActionClient
        from rclpy.qos import DurabilityPolicy, QoSProfile

        self.ros = rclpy
        self.node = rclpy.create_node("delivery_acceptance")
        self.action_type = DeliveryTask
        self.cancel_type = CancelGoal
        self.action = ActionClient(self.node, DeliveryTask, "/delivery_task")
        self.cancel_client = self.node.create_client(CancelGoal, "/delivery_task/_action/cancel_goal")
        self.orders = []
        self.names = ["robot1", "robot2"] if args.case == "fleet" else [args.robot]
        self.poses = GazeboPoses(args.world, self.names)
        self.active = {}
        self.obstacle = None
        self.ever_active = set()
        self.overlap_since = None
        self.max_overlap_s = 0.0
        self.deadline = time.monotonic() + args.timeout
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for name in self.names:
            prefix = "" if name == "lk1209" else "/" + name
            self.node.create_subscription(
                GoalStatusArray, prefix + "/navigate_to_pose/_action/status",
                lambda msg, robot=name: self.nav_status(robot, msg), qos)

    def nav_status(self, name, msg):
        self.active[name] = any(1 <= s.status <= 3 for s in msg.status_list)
        if self.active[name]:
            self.ever_active.add(name)
        self.track_overlap()

    def track_overlap(self):
        now = time.monotonic()
        if self.overlap_since is not None:
            self.max_overlap_s = max(self.max_overlap_s, now - self.overlap_since)
        if sum(self.active.values()) > 1:
            if self.overlap_since is None:
                self.overlap_since = now
        else:
            self.overlap_since = None

    def wait(self, condition, description, seconds=None):
        until = self.deadline
        if seconds is not None:
            until = min(until, time.monotonic() + seconds)
        while not condition():
            if time.monotonic() >= until:
                raise RuntimeError("Timeout waiting for " + description)
            self.ros.spin_once(self.node, timeout_sec=0.12)
            self.track_overlap()

    def prepare(self):
        require(self.action.wait_for_server(timeout_sec=25), "/delivery_task unavailable")
        self.wait(lambda: all(len(self.poses.trail(n)) >= 3 for n in self.names),
                  "live Gazebo model poses", 30)
        # An unused NavigateToPose server can have no latched status sample yet.
        for name in self.names:
            prefix = "" if name == "lk1209" else "/" + name
            topic = prefix + "/navigate_to_pose/_action/status"
            self.wait(lambda: self.node.count_publishers(topic) > 0,
                      "DDS discovery of " + topic, 25)
            require(self.node.count_publishers(topic) == 1, "Expected one action status publisher on " + topic)
            require(not self.active.get(name, False), f"{name} already has a navigation goal")
            start = self.poses.trail(name)[-1][1:3]
            require(math.dist(start, HOME[name]) < 0.35, f"{name} must start at its home; observed {start}")

    def submit(self, robot, station):
        import uuid
        from unique_identifier_msgs.msg import UUID

        order = Order(UUID(uuid=list(uuid.uuid4().bytes)))
        self.orders.append(order)
        goal = self.action_type.Goal(station=station, robot_id=robot)

        def feedback(msg):
            f = msg.feedback
            if f.robot_id:
                require(not order.robot or order.robot == f.robot_id, "Robot changed on an active order")
                order.robot = f.robot_id
            if not order.stages or f.stage != order.stages[-1]:
                order.stages.append(f.stage)
                print(f"{f.robot_id}: {f.stage}", flush=True)
            if f.stage in ("OUTBOUND", "RETURNING") and math.isfinite(f.distance_remaining):
                order.distances.append(f.distance_remaining)

        sent = self.action.send_goal_async(goal, goal_uuid=order.goal_id, feedback_callback=feedback)
        self.wait(sent.done, "delivery goal acknowledgement", 20)
        order.handle = sent.result()
        require(order.handle.accepted, f"Rejected delivery to {station} for {robot or 'auto'}")
        order.completed = order.handle.get_result_async()
        return order

    def terminal(self, order, status):
        self.wait(order.completed.done, "delivery result")
        response = order.completed.result()
        require(response.status == status and response.result.success == (status == 4),
                f"Delivery status {response.status}: {response.result.message}")
        require(response.result.robot_id == order.robot and order.robot, "Incorrect robot in result")

    def cancel(self, order):
        f = order.handle.cancel_goal_async()
        self.wait(f.done, "delivery cancel acknowledgement", 20)
        require(any(bytes(g.goal_id.uuid) == bytes(order.goal_id.uuid)
                    for g in f.result().goals_canceling), "Cancellation not acknowledged for our goal")
        self.terminal(order, 5)

    def cancel_after_motion(self, order):
        def moving():
            if order.completed.done():
                raise RuntimeError("Order finished before cancellation could be exercised")
            if not order.robot or not self.active.get(order.robot, False):
                return False
            path = self.poses.trail(order.robot)
            return math.dist(path[-1][1:3], path[0][1:3]) >= 0.18

        self.wait(moving, "an accepted Nav2 goal and 18 cm of Gazebo movement")
        self.cancel(order)
        self.wait(lambda: self.active.get(order.robot) is False, "Nav2 terminal state", 20)
        start_sim = self.poses.trail(order.robot)[-1][0]

        def settled():
            trail = [p for p in self.poses.trail(order.robot) if p[0] >= start_sim]
            if not trail:
                return False
            return is_still([p for p in trail if p[0] >= trail[-1][0] - 1.25])

        self.wait(settled, "one simulated second of stationary Gazebo position and yaw", 45)

    def run(self, case):
        self.prepare()
        if self.obstacle is not None:
            self.obstacle.prepare()
        robot = self.names[0]
        dispatch_wall = time.monotonic()
        dispatch_sim = self.poses.trail(robot)[-1][0]
        first = self.submit(robot, "east_room")
        report = {"case": case, "source": "Gazebo Harmonic model poses and live ROS 2 actions"}
        if case == "cancel":
            self.cancel_after_motion(first)
        else:
            second = None
            if case == "fleet":
                self.wait(lambda: "OUTBOUND" in first.stages, "first car starting its trip", 45)
                second = self.submit("", "west_room")
                self.wait(lambda: second.robot and "QUEUED" in second.stages, "second car queued", 45)
                require(second.robot != robot, "Auto dispatcher did not reserve the other car")
            self.terminal(first, 4)
            trail = self.poses.trail(robot)
            crossings = door_crossings(trail)
            stages = [s for s in first.stages if s in ("OUTBOUND", "DWELL", "RETURNING")]
            require(stages == ["OUTBOUND", "DWELL", "RETURNING"], f"Missing or unordered stages: {stages}")
            require(any(d > 0.1 for d in first.distances), "No remaining-distance feedback")
            require(crossings == [1, -1], f"Incorrect physical door crossings: {crossings}")
            report.update(door_crossings=crossings,
                          east_error_m=min(math.dist(p[1:3], EAST) for p in trail),
                          home_error_m=math.dist(trail[-1][1:3], HOME[robot]))
            require(max(report["east_error_m"], report["home_error_m"]) < 0.35, "Gazebo station error exceeds 35 cm")
            report["first_order_wall_s"] = round(time.monotonic()-dispatch_wall, 2)
            report["first_order_sim_s"] = round(trail[-1][0]-dispatch_sim, 2)
            report["east_arrival_sim_s"] = round(next(p[0] for p in trail
                if p[0] >= dispatch_sim and math.dist(p[1:3], EAST) < .35)-dispatch_sim, 2)
            if self.obstacle is not None:
                report["static_obstacle"] = self.obstacle.verify_route(trail)
            if second:
                # Restrict the witness interval to the first car's last point away from home;
                # action results and the second car's feedback arrive via separate DDS writers.
                away = max(p[0] for p in trail if math.dist(p[1:3], HOME[robot]) > 0.35)
                waiting = [p for p in self.poses.trail(second.robot) if p[0] <= away]
                require(len(waiting) > 2, "Missing queued-car samples")
                report["queued_max_displacement_m"] = max(math.dist(p[1:3], waiting[0][1:3]) for p in waiting)
                require(report["queued_max_displacement_m"] < 0.06, "Queued robot moved during the first trip")
                self.cancel_after_motion(second)
        self.wait(lambda: all(self.active.get(n) is False for n in self.names), "idle Nav2 actions", 20)
        require(self.ever_active == set(self.names), "One or more robots never had a live Nav2 goal")
        # Namespaces have independent DDS writers. A short receive-order inversion
        # at handoff is possible; the queued-car Gazebo trajectory checks it independently.
        report["max_apparent_nav_goal_overlap_wall_s"] = round(self.max_overlap_s, 3)
        require(self.max_overlap_s < 0.75, "Observed sustained simultaneous Nav2 goals in the shared route")
        report["orders"] = [{"robot": o.robot, "stages": o.stages, "status": o.completed.result().status}
                            for o in self.orders]
        print(json.dumps(report, indent=2), flush=True)

    def cleanup(self):
        # Scope all cancellations by UUID, including a submitted goal whose ack was lost.
        for order in self.orders:
            if order.completed is not None and order.completed.done():
                continue
            if order.handle is not None and not order.handle.accepted:
                continue
            req = self.cancel_type.Request()
            req.goal_info.goal_id = order.goal_id
            reply = self.cancel_client.call_async(req)
            self.ros.spin_until_future_complete(self.node, reply, timeout_sec=8)
            if not reply.done():
                print(f"Cancellation unconfirmed for {bytes(order.goal_id.uuid).hex()}", file=sys.stderr)
            if order.completed is not None:
                self.ros.spin_until_future_complete(self.node, order.completed, timeout_sec=12)
                if not order.completed.done():
                    print("Delivery did not reach a terminal state during cleanup", file=sys.stderr)
        self.node.destroy_node()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--case", choices=("single", "cancel", "fleet"), required=True)
    parser.add_argument("--robot", choices=("lk1209", "robot1"), default="lk1209",
                        help="Select the root or first namespaced car for a single-car test")
    parser.add_argument("--world", default="default")
    parser.add_argument("--timeout", type=float, default=1250, help="Maximum total wall seconds")
    parser.add_argument("--static-obstacle", action="store_true",
                        help="Validate the robot1 round trip with a real box and check both costmaps")
    args = parser.parse_args(argv)
    if args.static_obstacle and not (args.case == "fleet" or (args.case == "single" and args.robot == "robot1")):
        parser.error("--static-obstacle requires a robot1 round trip")
    import rclpy
    from rclpy.signals import SignalHandlerOptions

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    test = Acceptance(args)
    def interrupt(signum, frame):
        raise KeyboardInterrupt

    # A terminated SSH/background test must still cancel its specific goal IDs
    # and remove the single Gazebo fixture it owns.
    signals = (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)
    previous = {s: signal.signal(s, interrupt) for s in signals}
    try:
        if args.static_obstacle:
            from test_navigation_obstacle import StaticObstacle
            test.obstacle = StaticObstacle(test, args.world)
        test.run(args.case)
    finally:
        for s in signals:
            signal.signal(s, signal.SIG_IGN)
        try:
            test.cleanup()
        finally:
            try:
                if test.obstacle is not None:
                    test.obstacle.cleanup()
            finally:
                rclpy.shutdown()
                for s, handler in previous.items():
                    signal.signal(s, handler)


if __name__ == "__main__":
    main()
