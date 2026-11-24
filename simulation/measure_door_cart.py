#!/usr/bin/env python3
"""Measure the door crossing using synchronized Gazebo world poses (SI units)."""
import argparse
import bisect
import csv
import json
import math
import statistics
from pathlib import Path
from typing import NamedTuple


class Sample(NamedTuple):
    t: float
    rx: float
    ry: float
    ra: float
    qx: float
    qy: float
    qa: float
    cx: float
    cy: float
    ca: float


FIELDS = 'sim_s r1_x r1_y r1_yaw r2_x r2_y r2_yaw cart_x cart_y cart_yaw'.split()
BODY = (0.15, 0.135)  # rectangle enclosing chassis and wheels
CART = (0.19, 0.12)
LANE_Y = 0.403
HALF_LANE = 0.20
MAX_SAMPLE_GAP = 0.20


def signed_gap(p):
    """Euclidean distance between two oriented rectangles; negative SAT depth on overlap."""
    dx, dy = p.rx - p.cx, p.ry - p.cy
    cr, sr = math.cos(p.ra), math.sin(p.ra)
    cc, sc = math.cos(p.ca), math.sin(p.ca)
    c, s = math.cos(p.ra - p.ca), math.sin(p.ra - p.ca)
    rx, ry = dx*cc + dy*sc, -dx*sc + dy*cc
    cx, cy = -dx*cr - dy*sr, dx*sr - dy*cr
    a, b = BODY
    u, v = CART
    gap = max(abs(rx) - u - a*abs(c) - b*abs(s),
              abs(ry) - v - a*abs(s) - b*abs(c),
              abs(cx) - a - u*abs(c) - v*abs(s),
              abs(cy) - b - u*abs(s) - v*abs(c))
    if gap <= 0:
        return gap

    def closest(px, py, half_x, half_y, sine, other_x, other_y):
        distances = []
        for x in (-half_x, half_x):
            for y in (-half_y, half_y):
                qx = px + c*x - sine*y
                qy = py + sine*x + c*y
                distances.append(math.hypot(max(abs(qx)-other_x, 0.0),
                                            max(abs(qy)-other_y, 0.0)))
        return min(distances)

    return min(closest(rx, ry, a, b, s, u, v), closest(cx, cy, u, v, -s, a, b))


def measure(run_dir, queue_start=None, queue_end=None, order_end=None):
    path = Path(run_dir)
    with (path / 'poses.csv').open() as f:
        poses = [Sample(*(float(row[k]) for k in FIELDS)) for row in csv.DictReader(f)]
    with (path / 'events.csv').open() as f:
        events = {row['event']: float(row['sim_s']) for row in csv.DictReader(f)}
    if len(poses) < 2 or not all(math.isfinite(x) for p in poses for x in p):
        raise ValueError('Insufficient finite, synchronized pose samples')
    ts = [p.t for p in poses]
    steps = [b-a for a, b in zip(ts, ts[1:])]
    if min(steps) <= 0 or not all(map(math.isfinite, events.values())):
        raise ValueError('Invalid Gazebo timestamps')
    issues = []
    gaps = [signed_gap(p) for p in poses]
    i = min(range(len(poses)), key=gaps.__getitem__)
    result = dict(samples=len(poses), recorded_sim_s=[ts[0], ts[-1]],
                  events_sim_s=events, max_sample_interval_s=max(steps),
                  min_sampled_envelope_gap_m=gaps[i], closest_sim_s=ts[i],
                  overlap_samples=sum(v <= 0 for v in gaps), issues=issues)
    if max(steps) > MAX_SAMPLE_GAP:
        issues.append('Pose recording has gaps longer than 0.20 simulation seconds')
    if gaps[i] <= 0:
        issues.append('Sampled vehicle and cart envelopes touch or overlap')

    def covers(a, b):
        return a <= b and a >= ts[0] - MAX_SAMPLE_GAP and b <= ts[-1] + MAX_SAMPLE_GAP

    def coord(t, field):
        index = bisect.bisect_left(ts, t)
        if index == 0:
            return poses[0][field]
        if index == len(ts):
            return poses[-1][field]
        f = (t-ts[index-1]) / (ts[index]-ts[index-1])
        return poses[index-1][field]*(1-f) + poses[index][field]*f

    if order_end is not None:
        result['recording_covers_order_end'] = covers(order_end, order_end)
    if queue_start is None or queue_end is None or not covers(queue_start, queue_end):
        issues.append('Complete second-robot queue interval unavailable')
    else:
        qx, qy = coord(queue_start, 4), coord(queue_start, 5)
        qs = [(p.qx, p.qy) for p in poses if queue_start <= p.t <= queue_end]
        qs.append((coord(queue_end, 4), coord(queue_end, 5)))
        result['queue_sim_s'] = [queue_start, queue_end]
        result['queued_robot2_max_displacement_m'] = max(math.hypot(x-qx, y-qy) for x, y in qs)
        if result['queued_robot2_max_displacement_m'] >= .06:
            issues.append('Queued robot moved at least 6 cm')

    missing = [k for k in ('trigger', 'occupy', 'depart', 'lane_clear', 'park') if k not in events]
    if missing:
        issues.append('Missing scenario events: ' + ', '.join(missing))
        result['metrics_passed'] = False
        return result
    trigger, hold, depart, clear, park = (events[k] for k in ('trigger','occupy','depart','lane_clear','park'))
    if not (trigger < hold < depart < clear <= park and covers(trigger, park)):
        issues.append('Missing or misordered crossing coverage')
        result['metrics_passed'] = False
        return result
    def period(a, b):
        return poses[bisect.bisect_left(ts, a):bisect.bisect_right(ts, b)]
    held = period(hold, depart)
    if not held:
        raise ValueError('Missing cart poses in hold interval')
    center = (coord(hold, 7), coord(hold, 8))
    hold_motion = max(math.hypot(p.cx-center[0], p.cy-center[1]) for p in held)
    anchor = min(poses, key=lambda p: abs(p.t - clear))
    extent_y = CART[0]*abs(math.sin(anchor.ca)) + CART[1]*abs(math.cos(anchor.ca))
    clear_margin = LANE_Y - HALF_LANE - (anchor.cy + extent_y)

    def moving_speed(a, b):
        # Measure the central half of the traveled interval, excluding stop transients.
        y0, y1 = coord(a, 8), coord(b, 8)
        motion = [p for p in period(a,b) if y1+.25*(y0-y1) <= p.cy <= y1+.75*(y0-y1)]
        if len(motion) < 2:
            return None
        return (motion[0].cy - motion[-1].cy)/(motion[-1].t - motion[0].t)

    approach, departure = moving_speed(trigger,hold), moving_speed(depart,park)
    crossing = period(trigger, clear)
    closest_crossing = min(crossing, key=signed_gap)
    result.update(min_crossing_envelope_gap_m=signed_gap(closest_crossing),
                  closest_crossing_sim_s=closest_crossing.t)
    result.update(hold_sim_s=depart-hold, hold_sample_span_s=held[-1].t-held[0].t,
                  hold_max_displacement_m=hold_motion, hold_center_world_xy_m=center,
                  measured_cart_southbound_speeds_mps=[approach,departure],
                  lane_clear_cart_north_edge_m=anchor.cy+extent_y,
                  lane_south_edge_m=LANE_Y-HALF_LANE, clear_margin_m=clear_margin)
    if depart-hold < 3.0-1e-6 or hold_motion>.02 or abs(center[1]-LANE_Y)>.04:
        issues.append('Three-second stationary hold at the lane center not demonstrated')
    if clear_margin < -.005:
        issues.append('Cart had not cleared the lane at reported timestamp')
    if any(v is None or abs(v-.32)>.04 for v in (approach, departure)):
        issues.append('Actual cart speed could not be verified near 0.32 m/s on both legs')

    def speeds(t, dt=.1):
        vx = (coord(t+dt, 1)-coord(t, 1))/dt
        vy = (coord(t+dt, 2)-coord(t, 2))/dt
        return vx, math.hypot(vx, vy)

    def windows(start, end, interval):
        for i in range(max(0, 1+math.floor((end-start)/interval))):
            yield start + i*interval

    def typical(start, end):
        vals = [speeds(t)[1] for t in windows(max(ts[0],start), min(ts[-1],end)-.1, .1)]
        return statistics.median(vals) if vals else None

    def advancing(t):
        return t>=ts[0] and t+.5<=ts[-1] and all(speeds(t+i*.1)[0]>.04 for i in range(5))

    until = min(ts[-1], order_end if order_end is not None else ts[-1])
    first = next((t for t in windows(clear,until-.5,.05) if advancing(t)), None)
    at_clear = advancing(clear-.25) and advancing(clear)
    slow = [speeds(t,.3)[1] for t in windows(trigger, clear-.3, .05)]
    stopped_for = longest_stop = 0.0
    for t in windows(trigger, clear-.1, .1):
        stopped_for = stopped_for + .1 if speeds(t)[1] <= .025 else 0.0
        longest_stop = max(longest_stop, stopped_for)
    passed_while_blocked = any(p.rx>p.cx and p.t>=hold for p in period(trigger,clear))
    result.update(robot1_median_speed_last_second_before_trigger_mps=typical(trigger-1,trigger),
                  robot1_min_speed_over_300ms_during_encounter_mps=min(slow,default=None),
                  robot1_longest_low_speed_sim_s=round(longest_stop,3),
                  low_speed_threshold_mps=.025,
                  robot1_trigger_xy_m=[coord(trigger,1),coord(trigger,2)],
                  robot1_clear_xy_m=[coord(clear,1),coord(clear,2)],
                  robot1_median_speed_first_second_after_clear_mps=typical(clear,clear+1),
                  sustained_forward_at_clear=at_clear,
                  reached_cart_x_during_blockage=passed_while_blocked,
                  resume_after_clear_sim_s=None if first is None else first-clear,
                  resume_definition='eastward speed > 0.04 m/s in each of five consecutive 0.10 s intervals')
    if first is None and not passed_while_blocked:
        issues.append('No sustained eastward progress observed after lane cleared')
    result['metrics_passed'] = not issues
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run_dir', type=Path)
    parser.add_argument('--queue-start', type=float)
    parser.add_argument('--queue-end', type=float)
    parser.add_argument('--order-end', type=float)
    args = parser.parse_args()
    report = measure(args.run_dir, args.queue_start, args.queue_end, args.order_end)
    print(json.dumps(report, indent=2, allow_nan=False))
    raise SystemExit(0 if report['metrics_passed'] else 1)
