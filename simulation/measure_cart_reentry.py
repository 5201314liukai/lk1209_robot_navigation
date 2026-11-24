"""Replay a second cart incursion using raw laser, velocity and Gazebo records."""
import csv
import gzip
import json
import math
from collections import defaultdict
from pathlib import Path

from measure_door_cart import CART, FIELDS, HALF_LANE, LANE_Y, Sample, signed_gap
from observe_door_cart import closest, facing, ray_entry


def first_interval(rows, start, end, condition, duration):
    """Return onset and confirmation of a sustained state across fresh odometry."""
    onset = previous = None
    for row in rows:
        t = row[1]
        if not start <= t <= end:
            continue
        if not condition(row):
            onset = None
        else:
            if onset is None or t - previous > .12:
                onset = t
            if t - onset >= duration - 1e-8:
                return onset, t
        previous = t
    return None


def guard_hits(scan, p, length, half_width):
    if scan[3] != 'lidar_link':
        raise ValueError('Unexpected scan frame ' + scan[3])
    a0, da, lo, hi, ranges = scan[4:9]
    ox, oy = facing(p.rx - p.cx, p.ry - p.cy, p.ca)
    total = cart = 0
    for i, r in enumerate(ranges):
        if not isinstance(r, (int, float)) or not lo <= r <= hi:
            continue
        a = a0 + da * i
        # In the LK1209 URDF the lidar has zero planar offset and yaw.
        if 0 <= r * math.cos(a) < length and abs(r * math.sin(a)) < half_width:
            total += 1
            theta = a + p.ra - p.ca
            expected = ray_entry(ox, oy, math.cos(theta), math.sin(theta))
            if expected is not None and abs(r - expected) <= .035:
                cart += 1
    return total, cart


def guard_dimensions(directory):
    # Caller may archive the two live ROS parameters for reproducible replay.
    saved = directory / 'guard_geometry.json'
    if saved.is_file():
        params = json.loads(saved.read_text())
    else:
        from yaml import safe_load
        relative = Path('ros2_ws/src/robot_navigation/config/sim_collision_monitor.yaml')
        candidates = [Path(__file__).resolve().parents[1] / relative, Path.cwd() / relative]
        config = next((p for p in candidates if p.is_file()), None)
        if config is None:
            raise ValueError('No guard_geometry.json or project collision monitor configuration')
        params = safe_load(config.read_text())['sim_cmd_vel_guard']['ros__parameters']
    return float(params['stop_length']), float(params['stop_half_width'])


def stopped(row):
    return abs(row[6]) <= .005 and abs(row[7]) <= .035


def associate_brake(frames, reversal, command_time):
    """Relate a zero command to a fresh episode of observed cart returns."""
    if command_time is None:
        return 'no zero command', None
    prior = [f for f in frames if f[0] <= command_time]
    if not prior or command_time - prior[-1][0] > .16:
        return 'scan unavailable at zero command', None
    if prior[-1][2] >= 3:
        i = len(prior)-1
        while i > 0 and prior[i-1][2] >= 3 and prior[i][0]-prior[i-1][0] <= .16:
            i -= 1
        if i == 0 or prior[i][0] < reversal or prior[i][0]-prior[i-1][0] > .16:
            return 'start of guard occupancy unobserved', None
        return 'cart in guard at zero command', prior[i][0]
    following = next((f for f in frames if f[0] > command_time), None)
    if (following and following[2] >= 3 and following[0]-prior[-1][0] <= .16
            and 0 < prior[-1][2] < 3 and prior[-1][0] >= reversal):
        return 'early braking with cart returns in guard before threshold', following[0]
    return 'zero command unassociated with observed cart guard entry', None


def measure_reentry(directory):
    directory = Path(directory)
    checks = {}
    report = {'checks': checks}
    def finish():
        report['issues'] = [key for key, passed in checks.items() if not passed]
        report['passed'] = not report['issues']
        return report

    with (directory / 'events.csv').open() as f:
        events = {r['event']: float(r['sim_s']) for r in csv.DictReader(f)}
    names = ('trigger', 'lane_clear', 'park', 'reentry_trigger', 'reentry_occupy',
             'reentry_depart', 'reentry_clear', 'reentry_park')
    checks['recorded_reentry_sequence'] = (all(n in events for n in names)
        and all(events[a] < events[b] for a, b in zip(names, names[1:])))
    if not checks['recorded_reentry_sequence']:
        report['events_sim_s'] = events
        return finish()
    start, occupied, depart, clear, park = [events[n] for n in names[3:]]
    with (directory / 'poses.csv').open() as f:
        poses = [Sample(*(float(r[k]) for k in FIELDS)) for r in csv.DictReader(f)]
    times = [p.t for p in poses]
    phase = [p for p in poses if start <= p.t <= park]
    if len(phase) < 2:
        checks['second_physical_encounter_recorded'] = False
        return finish()
    checks['continuous_pose_record'] = all(0 < b-a <= .20 for a,b in zip(times, times[1:]))
    anchor = closest(times, poses, clear)
    center = closest(times, poses, occupied)
    checks['cart_reached_lane_center'] = center is not None and abs(center.cy-LANE_Y) <= .04
    if anchor is not None:
        south_edge = anchor.cy - CART[0]*abs(math.sin(anchor.ca)) - CART[1]*abs(math.cos(anchor.ca))
        report['cart_south_edge_at_final_clear_m'] = south_edge
    checks['cart_cleared_north_edge'] = anchor is not None and south_edge >= LANE_Y + HALF_LANE - .005
    speeds = []
    for a,b in ((start,occupied),(depart,park)):
        leg = [p for p in phase if a <= p.t <= b]
        if len(leg) >= 2:
            dy = leg[-1].cy - leg[0].cy
            leg = [p for p in leg if .25*dy <= p.cy-leg[0].cy <= .75*dy]
        speeds.append((leg[-1].cy-leg[0].cy)/(leg[-1].t-leg[0].t) if len(leg) >= 2 else None)
    report['actual_northbound_speeds_mps'] = speeds
    checks['northbound_cart_speed'] = all(v is not None and abs(v-.32) < .04 for v in speeds)
    nearest = min(phase, key=signed_gap)
    report.update(min_reentry_envelope_gap_m=signed_gap(nearest), closest_reentry_sim_s=nearest.t)
    checks['positive_reentry_clearance'] = report['min_reentry_envelope_gap_m'] > 0
    checks['robot_yielded_before_crossing'] = all(p.rx < p.cx for p in phase if p.t <= clear)

    trace = defaultdict(list)
    topics = ('odom', 'sim/cmd_vel_safe', 'cmd_vel', 'sim/cmd_vel_checked', 'scan')
    with gzip.open(directory / 'door_ros.jsonl.gz', 'rt') as f:
        for line in f:
            row = json.loads(line)
            if row[0] in topics and events['trigger']-.2 <= row[1] <= times[-1]:
                if row[0] != 'scan' or start-.35 <= row[1] <= clear+.15:
                    trace[row[0]].append(row)
    for rows in trace.values():
        rows.sort(key=lambda r: r[1])
    odom = trace['odom']
    first_stop = first_interval(odom, events['trigger'], start, stopped, .30)
    moving = (first_interval(odom, max(events['lane_clear'],first_stop[1]), start+.03,
                             lambda r: r[6] > .015, .20) if first_stop else None)
    at_start = closest([r[1] for r in odom], odom, start)
    report['initial_stop_interval_sim_s'] = first_stop
    report['verified_restart_interval_sim_s'] = moving
    report['reentry_initial_forward_speed_mps'] = at_start[6] if at_start else None
    checks['cart_returned_after_real_restart'] = bool(moving and at_start and at_start[6] > .015)

    length, width = guard_dimensions(directory)
    frames = []
    for scan in trace['scan']:
        p = closest(times, poses, scan[1])
        if p is not None:
            total, cart = guard_hits(scan, p, length, width)
            frames.append((scan[1], total, cart))
    before = [f for f in frames if start-.16 <= f[0] <= start]
    checks['guard_clear_at_reentry_trigger'] = bool(before and before[-1][1] < 3)
    hazard = next((f for f in frames if start <= f[0] <= clear and f[2] >= 3), None)
    checks['returning_cart_observed_in_directional_guard'] = hazard is not None
    report['directional_guard'] = dict(length_m=length,half_width_m=width,
        pretrigger_scan=before[-1] if before else None,
        entry_scan=hazard, frame_fields=['sim_s','total_in_guard','matching_cart_in_guard'])
    if hazard and before:
        hazard_window = [f for f in frames if before[-1][0] <= f[0] <= hazard[0]]
        checks['laser_coverage_to_guard_entry'] = all(b[0]-a[0] <= .16 for a,b in zip(hazard_window,hazard_window[1:]))

    stop = first_interval(odom, start, clear, stopped, .30)
    if stop and hazard and stop[0] < hazard[0]:
        # Earlier braking qualifies only if the vehicle stayed stopped through
        # entry. A brief unrelated stop before the hazard is a different event.
        earlier = [r for r in odom if stop[0] <= r[1] <= hazard[0]]
        if not all(stopped(r) for r in earlier) or hazard[0] - earlier[-1][1] > .12:
            stop = first_interval(odom, hazard[0], clear, stopped, .30)
    safe = trace['sim/cmd_vel_safe']
    zero_command = None
    # A brake command may precede a scan threshold crossing. Keep its signed
    # offset, and require it belongs to the motion-to-stop transition.
    if stop:
        moving_commands = [r for r in safe if start-.2 <= r[1] <= stop[0]+.04 and r[2] > .005]
        if moving_commands:
            zero_command = next((r for r in safe if max(start,moving_commands[-1][1]) <= r[1] <= stop[0]+.12
                                 and abs(r[2]) < 1e-6 and abs(r[3]) < 1e-6), None)
    checks['second_zero_base_command'] = zero_command is not None
    checks['second_sustained_odom_stop'] = stop is not None
    stop_t = stop[0] if stop else None
    cmd_t = zero_command[1] if zero_command else None
    association, entry_t = associate_brake(frames, start, cmd_t)
    if entry_t is not None and stop_t is not None and stop_t < entry_t:
        between = [r for r in odom if stop_t <= r[1] <= entry_t]
        if (not between or not all(stopped(r) for r in between)
                or any(b[1]-a[1] > .12 for a,b in zip(between,between[1:]))
                or entry_t-between[-1][1] > .12):
            association, entry_t = 'early stop not sustained through guard entry', None
    checks['brake_associated_with_reentering_cart'] = entry_t is not None and stop is not None
    report['brake_association'] = dict(observation=association, associated_guard_entry_sim_s=entry_t)
    report.update(reentry_trigger_sim_s=start, final_clear_sim_s=clear, stop_command_sim_s=cmd_t,
                  odometry_stop_interval_sim_s=stop, stop_threshold_mps=.005,
                  stop_yaw_threshold_radps=.035, stop_confirmation_s=.30)
    if stop:
        report['max_abs_yaw_rate_during_stop_confirmation_radps'] = max(
            abs(r[7]) for r in odom if stop[0] <= r[1] <= stop[1])
    for name,t in [('zero_command', cmd_t), ('odometry_stop', stop_t)]:
        report[name+'_after_cart_reversal_s'] = t-start if t is not None else None
        report[name+'_relative_to_guard_entry_s'] = (
            t-entry_t if t is not None and entry_t is not None and checks['guard_clear_at_reentry_trigger'] else None)
    if stop and entry_t is not None:
        report['odometry_stop_relative_to_entry'] = ('already stopped at first in-zone scan'
            if stop[0] < entry_t else 'stopped at or after first in-zone scan')
    after = first_interval(odom, clear, times[-1], lambda r: r[6] > .015, .20)
    report['post_clear_forward_interval_sim_s'] = after
    checks['eastward_trip_continued_after_final_clear'] = bool(after and anchor and
        any(p.rx > anchor.rx+.30 and p.rx > p.cx for p in poses if after[0] <= p.t))
    return finish()
