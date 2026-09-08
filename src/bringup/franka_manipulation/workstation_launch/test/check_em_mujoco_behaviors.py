"""Isolated Franka MuJoCo / example 16 behavior probe. Never launches real hardware."""

import argparse
import json
import os
import re
import signal
import subprocess
import sys
import time
from pathlib import Path

import rclpy
from execution_manager_interfaces.action import LeasedFollowJointTrajectory
from execution_manager_interfaces.srv import SetSourceActivation
from std_msgs.msg import String
from execution_manager_interfaces.msg import (
    AuthorityStatus,
    LeasedJointReference,
    SourceLifecycleStatus,
)
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState, Joy
from trajectory_msgs.msg import JointTrajectoryPoint

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--output', type=Path, default=Path('/tmp/franka_em_validation/automated'))
parser.add_argument('--headless', action='store_true')
args = parser.parse_args()
root = Path(__file__).resolve().parents[5]
os.chdir(root)
out = args.output.resolve()
out.mkdir(parents=True, exist_ok=True)
node = None
procs = {}
results = []
stats = {'owners': {}, 'joints': {}, 'trace': 0, 'sources': {}}
clutch = False
send_joy = True
axis = 0.0
last_joy = 0.0


def start(name, args):
    log = (out / f'{name}.log').open('w')
    p = subprocess.Popen(
        args, stdin=subprocess.PIPE, stdout=log, stderr=subprocess.STDOUT, start_new_session=True
    )
    procs[name] = p
    log.close()
    return p


def stop(name):
    p = procs.get(name)
    if p and p.poll() is None:
        os.killpg(p.pid, signal.SIGINT)
        try:
            p.wait(timeout=8)
        except subprocess.TimeoutExpired:
            os.killpg(p.pid, signal.SIGKILL)
            p.wait()


def pump(seconds):
    global last_joy
    until = time.monotonic() + seconds
    while time.monotonic() < until:
        rclpy.spin_once(node, timeout_sec=0.01)
        heartbeat.publish(String(data="mujoco-planner-session"))
        now = time.monotonic()
        if send_joy and now - last_joy > 0.02:
            msg = Joy()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.axes = [0.0] * 8
            msg.axes[1] = axis
            msg.buttons = [0] * 15
            msg.buttons[9] = int(clutch)
            joy.publish(msg)
            last_joy = now


def wait(predicate, seconds=15):
    until = time.monotonic() + seconds
    while time.monotonic() < until:
        if predicate():
            return True
        pump(0.03)
    return bool(predicate())


def check(name, ok, **details):
    item = {'name': name, 'passed': bool(ok), **details}
    results.append(item)
    print(json.dumps(item), flush=True)


def owner(name):
    return (
        stats['owners'].get('arm', {}).get('source') == name
        and stats['owners']['arm']['state'] == 2
    )


def authority(msg):
    stats['owners'] = {
        r.resource: {'source': r.source_instance, 'state': r.authority_state, 'lease': r.lease_id}
        for r in msg.resources
    }


def joints(msg):
    stats['joints'].update(zip(msg.name, msg.position))


def trace(msg):
    stats['trace'] += 1


def planner_activation(active):
    request = SetSourceActivation.Request(source_instance="TrajectoryPlanner",
        session_id="mujoco-planner-session", active=active, preempt=True)
    future = activation.call_async(request)
    if not wait(future.done) or not future.result().success:
        raise RuntimeError("planner activation/release failed")
    return future.result().lease_id


def planner_goal(duration=2.0):
    lease = planner_activation(True)
    goal = LeasedFollowJointTrajectory.Goal()
    goal.header.stamp = node.get_clock().now().to_msg()
    goal.resource = "arm"
    goal.lease_id = lease
    goal.trajectory.joint_names = [f'fr3_joint{i}' for i in range(1, 8)]
    positions = [stats['joints'][j] for j in goal.trajectory.joint_names]
    for t in (0.0, duration):
        point = JointTrajectoryPoint()
        point.positions = positions.copy()
        point.velocities = [0.0] * 7
        point.accelerations = [0.0] * 7
        point.time_from_start.sec = int(t)
        point.time_from_start.nanosec = int((t - int(t)) * 1e9)
        if t:
            point.positions[6] += 0.015
        goal.trajectory.points.append(point)
    future = planner.send_goal_async(goal)
    if not wait(future.done):
        raise RuntimeError('planner goal timeout')
    handle = future.result()
    if not handle.accepted:
        raise RuntimeError('planner goal rejected')
    return handle


try:
    start(
        'rt',
        [
            'ros2',
            'launch',
            'franka_manipulation_rt_launch',
            'rt_stack.launch.py',
            'backend:=mujoco',
            'task:=pick_up_the_black_bowl_between_the_plate_and_the_ramekin_and_place_it_on_the_plate',
            f'headless:={str(args.headless).lower()}',
            'with_cameras:=false',
            'load_pika_hardware:=true',
        ],
    )
    rclpy.init()
    node = rclpy.create_node(
        'franka_em_behavior_probe', parameter_overrides=[Parameter('use_sim_time', value=True)]
    )
    joy = node.create_publisher(Joy, '/test/franka/joy', 10)
    node.create_subscription(
        AuthorityStatus,
        '/execution_manager/authority_status',
        authority,
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
    )
    node.create_subscription(
        SourceLifecycleStatus, '/execution_manager/source_status',
        lambda msg: stats.update(sources={s.source_instance: s.state for s in msg.sources}),
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
    )
    node.create_subscription(JointState, '/joint_states', joints, 10)
    node.create_subscription(
        LeasedJointReference, '/execution_trace/policy/arm/joint_reference', trace, 10
    )
    planner = ActionClient(
        node,
        LeasedFollowJointTrajectory,
        '/execution_manager/ingress/planner/arm/follow_joint_trajectory',
    )
    activation = node.create_client(SetSourceActivation, "/execution_manager/source_activation")
    heartbeat = node.create_publisher(String, "/execution_manager/source_heartbeat", 10)
    if not wait(lambda: len(stats['joints']) >= 8 and node.get_clock().now().nanoseconds > 0, 40):
        raise RuntimeError('MuJoCo readiness failed')
    start(
        'workstation',
        [
            'ros2',
            'launch',
            'franka_manipulation_workstation_launch',
            'workstation_stack.launch.py',
            'use_sim_time:=true',
            'joy_topic:=/test/franka/joy',
            'device_id:=999',
        ],
    )
    # Invalid device index keeps the hardware driver from competing with synthetic Joy.
    if not wait(lambda: bool(stats['owners']), 20):
        raise RuntimeError('EM readiness failed')
    pump(3)
    check('idle_without_clutch', all(v['state'] == 0 for v in stats['owners'].values()))
    start(
        'example',
        [
            sys.executable,
            '-u',
            'examples/16_franka_gamepad_teleop.py',
            '--no-cam',
            '--no-record',
            '--amplitude',
            '0.03',
            '--period',
            '6',
        ],
    )
    if not wait(lambda: owner('DummyPolicy'), 120):
        raise RuntimeError('example homing/policy activation failed')
    check('example_homing_and_policy', True)
    q0 = stats['joints'].copy()
    pump(6)
    check(
        'policy_moves',
        max(abs(stats['joints'][j] - q0[j]) for j in q0) > 0.001,
        trace_count=stats['trace'],
    )
    for cycle in range(3):
        clutch = True
        axis = 0.10
        check(f'teleop_takeover_{cycle}', wait(lambda: owner('TeleopTwist')))
        check(f'candidate_waiting_{cycle}', wait(lambda: stats['sources'].get('DummyPolicy') == 1))
        pump(0.5)
        before = stats['trace']
        pump(0.6)
        check(f'policy_fenced_{cycle}', stats['trace'] == before)
        clutch = False
        axis = 0.0
        check(f'policy_resume_{cycle}', wait(lambda: owner('DummyPolicy')))
        check(f'candidate_controlling_{cycle}', wait(lambda: stats['sources'].get('DummyPolicy') == 3))
        pump(0.6)
    clutch = True
    check('joy_dropout_takeover', wait(lambda: owner('TeleopTwist')))
    send_joy = False
    check('joy_dropout_releases', wait(lambda: owner('DummyPolicy'), 8))
    clutch = False
    send_joy = True
    pump(1)
    handle = planner_goal()
    future = handle.get_result_async()
    check('planner_completion', wait(future.done, 20) and future.result().status == 4)
    planner_activation(False)
    check('policy_after_planner', wait(lambda: owner('DummyPolicy')))
    handle = planner_goal(10)
    pump(0.5)
    cancel = handle.cancel_goal_async()
    future = handle.get_result_async()
    check('planner_cancel', wait(future.done, 10) and future.result().status == 5)
    planner_activation(False)
    check('policy_after_cancel', wait(lambda: owner('DummyPolicy')))
    handle = planner_goal(10)
    check('planner_running', wait(lambda: owner('TrajectoryPlanner')))
    clutch = True
    check('teleop_preempts_planner', wait(lambda: owner('TeleopTwist')))
    future = handle.get_result_async()
    check('preempted_planner_terminal', wait(future.done, 10) and future.result().status in (5, 6))
    clutch = False
    check('policy_after_planner_preemption', wait(lambda: owner('DummyPolicy')))
    # Resume the task with a new path from current observation, not an old trajectory cursor.
    handle = planner_goal()
    future = handle.get_result_async()
    check('planner_fresh_reexecution', wait(future.done, 20) and future.result().status == 4)
    planner_activation(False)
    wait(lambda: owner('DummyPolicy'))
    clutch = True
    check('teleop_before_crash', wait(lambda: owner('TeleopTwist')))
    log = (out / 'workstation.log').read_text()
    match = re.search(r'\[gamepad_teleop_node-\d+\]: process started with pid \[(\d+)\]', log)
    if not match:
        raise RuntimeError('cannot identify this launch gamepad process')
    os.kill(int(match.group(1)), signal.SIGTERM)
    send_joy = False
    check(
        'teleop_crash_faults',
        wait(lambda: stats['owners'].get('arm', {}).get('state') == 3, 5),
        authority=stats['owners'],
    )
except Exception as exc:  # noqa: BLE001 - preserve diagnostics and stop every child
    check('infrastructure', False, error=repr(exc))
finally:
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    for name in ('example', 'workstation', 'rt'):
        stop(name)
    if node is not None:
        node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    (out / 'results.json').write_text(json.dumps(results, indent=2) + '\n')
sys.exit(0 if results and all(r['passed'] for r in results) else 1)
