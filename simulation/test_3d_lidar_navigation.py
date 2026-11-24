"""Drive around two low fixtures and under a raised bar, then return home."""
import argparse
import json
import math
from pathlib import Path
import time

import rclpy
from action_msgs.msg import GoalStatusArray
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile

from test_3d_lidar import CASES, DEPTH, WIDTH, Fixture
from test_fleet_delivery import GazeboPoses, HOME, is_still, require


BAR_X, BAR_Y = -0.90, 0.45
BAR_HX, BAR_HY = DEPTH/2, WIDTH/2
BODY_HX, BODY_HY = 0.15, 0.135


def gap(x, y, yaw):
    """Separating-axis lower bound on planar distance to a horizontal bar."""
    dx, dy = x-BAR_X, y-BAR_Y
    c, s = math.cos(yaw), math.sin(yaw)
    return max(abs(dx)-BAR_HX-BODY_HX*abs(c)-BODY_HY*abs(s),
               abs(dy)-BAR_HY-BODY_HX*abs(s)-BODY_HY*abs(c),
               abs(dx*c+dy*s)-BODY_HX-BAR_HX*abs(c)-BAR_HY*abs(s),
               abs(dy*c-dx*s)-BODY_HY-BAR_HX*abs(s)-BAR_HY*abs(c))


def wait(node, pred, seconds, label):
    end = time.monotonic()+seconds
    while not pred():
        if time.monotonic() >= end:
            raise RuntimeError('Timeout waiting for ' + label)
        rclpy.spin_once(node, timeout_sec=.1)


def trial(node, nav, poses, name, heights, goal_x, record):
    fixture = Fixture('default', 'navigation_' + name) if heights else None
    handle = None
    result = None
    try:
        if fixture is not None:
            bottom, top = heights
            fixture.spawn((BAR_X, BAR_Y), 0., bottom, top)
            record.update(bottom_m=bottom, top_m=top)
        start = poses.trail('lk1209')[-1][0]
        wait(node, lambda: poses.trail('lk1209')[-1][0] > start+2., 75, 'sensor updates')
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x = goal_x
        goal.pose.pose.orientation.w = 1.
        sent = nav.send_goal_async(goal)
        wait(node, sent.done, 12, 'goal acceptance')
        handle = sent.result()
        require(handle.accepted, 'Goal rejected')
        result = handle.get_result_async()
        wait(node, result.done, 550, 'navigation result')
        status = result.result().status
        trail = [p for p in poses.trail('lk1209') if p[0] >= start]
        require(len(trail) >= 2, 'Missing Gazebo motion samples')
        crossings = []
        directions = []
        for (t,x,y,_), (u,xx,yy,_) in zip(trail, trail[1:]):
            if (x < BAR_X) != (xx < BAR_X):
                require(u-t < .25 and abs(xx-x) < .12, 'Discontinuous passage')
                crossings.append(y+(yy-y)*(BAR_X-x)/(xx-x))
                directions.append(1 if xx>x else -1)
        home_x, home_y = HOME['lk1209']
        error = math.dist(trail[-1][1:3], (home_x+goal_x, home_y))
        required_direction = 1 if goal_x>0 else -1
        geometry_pass = required_direction in directions
        if fixture is not None:
            minimum = min(gap(x,y,a) for _,x,y,a in trail)
            record.update(min_planar_gap_lower_bound_m=minimum, bar_center_y_m=BAR_Y)
            if name == 'raised_bar':
                geometry_pass = any(direction==required_direction and abs(y-BAR_Y)<BAR_HY
                                    for direction,y in zip(directions,crossings))
            else:
                geometry_pass = geometry_pass and minimum>0
        record.update(passed=status==4 and geometry_pass and error<.35, action_status=status,
                      world_crossing_y_m=crossings, world_crossing_directions=directions,
                      samples=len(trail), goal_world_error_m=error,
                      actual_final_world_xy_m=trail[-1][1:3])
    finally:
        try:
            if handle and handle.accepted and (result is None or not result.done()):
                cancel = handle.cancel_goal_async()
                wait(node, cancel.done, 10, 'goal cancellation')
                if result is not None:
                    wait(node, result.done, 15, 'canceled goal result')
        finally:
            if fixture is not None:
                fixture.remove()


def save(out, report):
    temp = out.with_suffix(out.suffix+'.tmp')
    temp.write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    temp.replace(out)


def main():
    args = argparse.ArgumentParser(description=__doc__)
    args.add_argument('output', type=Path)
    out = args.parse_args().output
    out.parent.mkdir(parents=True, exist_ok=True)
    try:
        out.touch(exist_ok=False)
    except FileExistsError:
        args.error('Use a fresh result file')
    rclpy.init()
    node = rclpy.create_node('lidar_height_navigation_check')
    active = set()
    def update_status(msg):
        active.clear()
        active.update(bytes(g.goal_info.goal_id.uuid) for g in msg.status_list if g.status in (1,2,3))
    status_sub = node.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status',
        update_status, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    report = {'passed':False}
    try:
        poses = GazeboPoses('default', ['lk1209'])
        nav = ActionClient(node, NavigateToPose, '/navigate_to_pose')
        require(nav.wait_for_server(timeout_sec=15), 'Navigation unavailable')
        wait(node, lambda: node.count_publishers('/navigate_to_pose/_action/status') and
             is_still(poses.trail('lk1209')[-23:]), 45, 'stationary robot and navigation status publisher')
        require(not active, 'Navigation is busy')
        require(math.dist(poses.trail('lk1209')[-1][1:3], (-2.,.45)) < .25, 'Start at house home')
        heights = {name:(bottom,top) for name,bottom,top in CASES}
        for name,goal_x in (('low_bar',2.20),('raised_bar',0.),('low_block',2.20),('return_home',0.)):
            phase = {'passed': False, 'goal_map_x_m': goal_x}
            report[name] = phase
            save(out, report)
            trial(node, nav, poses, name, heights.get(name), goal_x, phase)
            save(out, report)
            print(name, phase, flush=True)
            require(phase['passed'], 'Failed '+name+' navigation')
        report['passed'] = True
    except Exception as exc:
        report['error'] = str(exc)
        raise
    finally:
        save(out, report)
        node.destroy_subscription(status_sub)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
