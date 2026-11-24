#!/usr/bin/env python3
"""Run one east-room delivery with a crossing cart and a queued second order."""
import argparse
from contextlib import ExitStack
import json
import math
from pathlib import Path
import signal
import subprocess
import sys
from types import SimpleNamespace

from test_fleet_delivery import Acceptance, HOME, door_crossings, require


def run_order(test, directory, recorder, cart, result):
    def sim_time():
        return test.poses.trail('robot1')[-1][0]

    def fixture_ready():
        require(cart.poll() is None and recorder.poll() is None, 'Cart or observer exited')
        return (directory / 'ready').exists()

    test.wait(fixture_ready, 'cart in Gazebo', 70)
    one = test.submit('robot1', 'east_room')
    test.wait(lambda: 'OUTBOUND' in one.stages or one.completed.done(), 'robot1 outbound', 45)
    require('OUTBOUND' in one.stages, 'First order did not start')
    two = test.submit('robot2', 'west_room')
    test.wait(lambda: 'QUEUED' in two.stages or two.completed.done(), 'robot2 queued', 25)
    require('QUEUED' in two.stages, 'Second order did not queue')
    result['queue_start'] = sim_time()
    try:
        def returned_through_door():
            fixture_ready()
            return (one.completed.done() or
                    ('RETURNING' in one.stages and door_crossings(test.poses.trail('robot1')) == [1, -1]))

        test.wait(returned_through_door, 'return crossing or terminal order')
        result['queue_end'] = sim_time()
        # Withdraw the witness order before robot1 releases the shared route.
        # Only the first car is scheduled to move in this test.
        if not two.completed.done():
            test.cancel(two)
        test.terminal(one, 4)
        trail = test.poses.trail('robot1')
        result.update(door_crossings=door_crossings(trail),
                      home_error_m=math.dist(trail[-1][1:3], HOME['robot1']))
        require(result['door_crossings'] == [1, -1], 'Missing a physical door crossing')
        require(result['home_error_m'] < .35, 'Robot1 did not return home')
        require(two.stages == ['QUEUED'] and 'robot2' not in test.ever_active,
                'Witness order left the queue')
        require(two.completed.result().status == 5, 'Witness order cancellation unconfirmed')
    finally:
        result.setdefault('queue_end', sim_time())
        result['order_end'] = sim_time()
        result['orders'] = [
            dict(robot=o.robot, stages=o.stages, status=o.completed.result().status
                 if o.completed.done() else None,
                 message=o.completed.result().result.message if o.completed.done() else None)
            for o in (one, two)]


def assess_run(out):
    """Recompute the verdict from the original order, Gazebo and ROS records."""
    from measure_door_cart import measure
    from observe_door_cart import assess

    checks = {}
    errors = []
    inputs = (OSError, EOFError, ValueError, KeyError, IndexError, TypeError)
    order = {}
    try:
        order = json.loads((out / 'order.json').read_text())
        if order.get('scenario', 'crossing') not in ('crossing', 'reentry'):
            raise ValueError('Unknown recorded cart scenario')
        jobs = {q['robot']: q for q in order.get('orders', [])}
        checks['order_and_fixture_completed'] = bool(
            order.get('passed') and not order.get('error')
            and jobs.get('robot1', {}).get('status') == 4
            and jobs.get('robot2', {}).get('status') == 5
            and jobs['robot2'].get('stages') == ['QUEUED']
            and order.get('door_crossings') == [1, -1]
            and order.get('home_error_m', math.inf) < .35)
        if order.get('error'):
            errors.append(order['error'])
    except inputs as exc:
        errors.append(f'Order evidence: {exc}')
    try:
        motion = measure(out, order.get('queue_start'), order.get('queue_end'), order.get('order_end'))
        (out / 'motion.json').write_text(json.dumps(motion, indent=2, allow_nan=False) + '\n')
        checks['motion_metrics'] = motion['metrics_passed']
        checks['recorded_through_order_end'] = motion.get('recording_covers_order_end') is True
        checks['yielded_while_blocked'] = motion.get('reached_cart_x_during_blockage') is False
        delay = motion.get('resume_after_clear_sim_s')
        checks['sustained_progress_after_clear'] = delay is not None and math.isfinite(delay) and delay >= 0
        errors.extend(motion['issues'])
    except inputs as exc:
        errors.append(f'Gazebo evidence: {exc}')
    try:
        assess(out)
        sensors = json.loads((out / 'sensor_summary.json').read_text())
        held = sensors['phases']['held']
        checks['sensor_phase_coverage'] = sensors['complete_phases']
        checks['held_cart_scan_detected'] = held['scan']['frames_with_cart_hits'] > 0
        checks['held_cart_costmap_marked'] = held['grid']['frames_with_cart_hits'] > 0
        checks['held_odometry_recorded'] = held['odom_mps']['samples'] > 0
    except inputs as exc:
        errors.append(f'ROS evidence: {exc}')
    if order.get('scenario') == 'reentry':
        from measure_cart_reentry import measure_reentry
        try:
            reentry = measure_reentry(out)
            (out / 'reentry.json').write_text(json.dumps(reentry, indent=2, allow_nan=False) + '\n')
            checks['second_encounter'] = reentry['passed']
            errors.extend(reentry['issues'])
        except inputs as exc:
            errors.append(f'Reentry evidence: {exc}')
    failed = [key for key, passed in checks.items() if not passed]
    verdict = dict(scenario=order.get('scenario', 'crossing'),
                   passed=not errors and not failed, checks=checks, failed_checks=failed, errors=errors)
    (out / 'acceptance.json').write_text(json.dumps(verdict, indent=2) + '\n')
    print(json.dumps(verdict), flush=True)
    return verdict['passed']


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path, help='Fresh output directory, or a recorded run for replay')
    parser.add_argument('--assess-only', action='store_true', help='Recompute acceptance without ROS or motion')
    parser.add_argument('--reenter', action='store_true',
                        help='Return the cart across the lane once robot1 resumes forward motion')
    args = parser.parse_args()
    out = args.directory.resolve()
    if args.assess_only:
        if not out.is_dir():
            parser.error('Recorded directory does not exist')
        if args.reenter:
            try:
                recorded = json.loads((out / 'order.json').read_text())
            except (OSError, ValueError) as exc:
                parser.error(f'Cannot identify recorded scenario: {exc}')
            if recorded.get('scenario') != 'reentry':
                parser.error('This recording does not contain a reentry order')
        raise SystemExit(0 if assess_run(out) else 1)
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    out.mkdir(parents=True, exist_ok=True)
    if any((out / n).exists() for n in ('order.json', 'ready', 'finish', 'poses.csv')):
        parser.error('Use a fresh output directory')
    scripts = Path(__file__).resolve().parent
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    test = Acceptance(SimpleNamespace(case='fleet', world='default', timeout=1250))
    def interrupt(*_):
        raise KeyboardInterrupt()
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, interrupt)
    result = {'scope': 'delivery order and fixture completion', 'passed': False,
              'scenario': 'reentry' if args.reenter else 'crossing'}
    try:
        test.prepare()
        with ExitStack() as stack:
            def worker(name, *cmd):
                log = stack.enter_context((out / (name + '.log')).open('w'))
                return subprocess.Popen([sys.executable, '-u', str(scripts / (name + '.py')),
                                         *cmd], stdout=log, stderr=subprocess.STDOUT)
            recorder = worker('observe_door_cart', 'record', str(out))
            cart = None
            try:
                cart_args = ['--reentry'] if args.reenter else []
                cart = worker('door_cart', str(out), *cart_args)
                run_order(test, out, recorder, cart, result)
            finally:
                try:
                    test.cleanup()
                finally:
                    (out / 'finish').touch()
                    try:
                        if cart is not None:
                            cart.wait(timeout=80)
                    finally:
                        recorder.terminate()
                        recorder.wait(timeout=20)
            require(cart.returncode == 0 and (out / 'done').exists(), 'Cart completion not confirmed')
            require(recorder.returncode == 0, 'Sensor recorder failed')
        result['passed'] = True
    except BaseException as exc:
        result['error'] = str(exc) or type(exc).__name__
        raise
    finally:
        (out / 'order.json').write_text(json.dumps(result, indent=2) + '\n')
        rclpy.try_shutdown()
        passed = assess_run(out)
        if result['passed']:
            require(passed, 'Dynamic crossing acceptance failed; see acceptance.json')


if __name__ == '__main__':
    main()
