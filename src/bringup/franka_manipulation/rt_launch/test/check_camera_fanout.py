#!/usr/bin/env python3
"""Manual simulation-only A/B check; run in a sourced ROS workspace.

python test/check_camera_fanout.py --domain 98 [--gui] [--dual-camera]
Starts its own MuJoCo with a unique SHM prefix; never commands hardware.
Measures wall-clock reception, SHM production and /clock RTF. JSON goes to stdout.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import struct
import subprocess
import sys
import tempfile
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image
import yaml


def receive(topics, seconds):
    rclpy.init()
    node = rclpy.create_node(f'camera_fanout_client_{os.getpid()}')
    start = time.monotonic()
    arrivals = {topic: [] for topic in topics}
    subscriptions = [node.create_subscription(
        Image, topic, lambda message, topic=topic: arrivals[topic].append(time.monotonic()),
        qos_profile_sensor_data) for topic in topics]
    while time.monotonic() < start + seconds + 2:
        rclpy.spin_once(node, timeout_sec=0.02)
    result = {}
    for topic, times in arrivals.items():
        times = [value for value in times if value >= start + 2]
        result[topic] = (len(times) - 1) / (times[-1] - times[0]) if len(times) > 1 else 0
    node.destroy_node()
    rclpy.shutdown()
    return result


def sequence(path):
    # Diagnostic counter only, never consumes payload. Production reader uses
    # aligned acquire atomics and validates the seqlock after copying pixels.
    with path.open('rb') as stream:
        header = struct.unpack('<8I', stream.read(32))
        assert header[0:2] == (0x4D4A4331, 2), header
        stream.seek(720)
        first = struct.unpack('<Q', stream.read(8))[0]
        stream.seek(720 + header[7])
        second = struct.unpack('<Q', stream.read(8))[0]
    return max(value for value in (first, second) if value % 2 == 0) / 2


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--domain', type=int, default=98)
    parser.add_argument('--seconds', type=float, default=10)
    parser.add_argument('--gui', action='store_true')
    parser.add_argument('--dual-camera', action='store_true')
    parser.add_argument('--subscriber', nargs='+', help=argparse.SUPPRESS)
    args = parser.parse_args()
    os.environ['ROS_DOMAIN_ID'] = str(args.domain)
    if args.subscriber:
        print(json.dumps(receive(args.subscriber, args.seconds)))
        return
    config = yaml.safe_load((Path(__file__).resolve().parents[1] / 'config/mujoco_plugins.yaml').read_text())
    camera = config['/**']['ros__parameters']['mujoco_plugins']['mujoco_camera_plugin']
    camera['shm_prefix'] = f'/pai_fanout_{os.getpid()}_'
    if not args.dual_camera:
        camera['pika_d405']['policy'] = 'disabled'
    names = [name for name, entry in camera.items()
             if isinstance(entry, dict) and entry.get('policy') == 'streaming']
    topics = [camera[name][key] for name in names for key in ('image_topic', 'depth_topic')]
    paths = {name: Path('/dev/shm' + camera['shm_prefix'] + name) for name in names}
    directory = Path(tempfile.mkdtemp(prefix='pai-camera-fanout-'))
    config_path = directory / 'plugins.yaml'
    config_path.write_text(yaml.safe_dump(config))
    print(json.dumps({'logs': str(directory), 'cameras': names, 'gui': args.gui, 'client_cyclonedds_uri': os.environ.get('CYCLONEDDS_URI', '')}), flush=True)
    rclpy.init()
    node = rclpy.create_node('camera_fanout_monitor')
    clocks = []
    subscription = node.create_subscription(
        Clock, '/clock', lambda msg: clocks.append((time.monotonic(), msg.clock.sec + msg.clock.nanosec * 1e-9)),
        qos_profile_sensor_data)

    def spin(seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.02)

    with (directory / 'launch.log').open('w') as log:
        launch = subprocess.Popen([
            'ros2', 'launch', 'franka_manipulation_rt_launch', 'mujoco_bringup.launch.py',
            f'headless:={str(not args.gui).lower()}',
            f'mujoco_plugins_yaml:={config_path}',
        ], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        clients = []
        try:
            deadline = time.monotonic() + 45
            while time.monotonic() < deadline and not all(node.get_publishers_info_by_topic(t) for t in topics):
                assert launch.poll() is None, f'Launch exited; inspect {directory}'
                spin(0.2)
            for topic in topics:
                publishers = node.get_publishers_info_by_topic(topic)
                assert [p.node_name for p in publishers] == ['mujoco_image_bridge'], topic
            cm = node.get_publisher_names_and_types_by_node('mujoco_ros2_control_node', '/')
            assert not any('sensor_msgs/msg/Image' in types for _, types in cm), cm
            phases = []
            for count in (0, 1, 2):
                clocks.clear()
                before = {name: sequence(path) for name, path in paths.items()}
                started = time.monotonic()
                clients = [subprocess.Popen([
                    sys.executable, str(Path(__file__).resolve()), '--domain', str(args.domain),
                    '--seconds', str(args.seconds), '--subscriber', *topics,
                ], stdout=subprocess.PIPE, text=True) for _ in range(count)]
                if not clients:
                    spin(3)
                while any(client.poll() is None for client in clients):
                    assert time.monotonic() - started < args.seconds + 20, 'Subscriber timed out'
                    spin(0.1)
                results = [json.loads(client.communicate()[0]) for client in clients]
                assert all(client.returncode == 0 for client in clients)
                elapsed = time.monotonic() - started
                assert len(clocks) > 1, 'No simulation clock'
                result = {
                    'subscribers': count, 'wall_fps': results,
                    'source_fps': {name: (sequence(path) - before[name]) / elapsed for name, path in paths.items()},
                    'rtf': (clocks[-1][1] - clocks[0][1]) / (clocks[-1][0] - clocks[0][0]),
                    'clock_backwards': any(b[1] < a[1] for a, b in zip(clocks, clocks[1:])),
                }
                phases.append(result)
                print(json.dumps(result), flush=True)
                assert not result['clock_backwards']
            for client in phases[2]['wall_fps']:
                for topic in topics:
                    assert client[topic] >= phases[1]['wall_fps'][0][topic] * 0.9 > 0, (topic, phases)
        finally:
            for client in clients:
                if client.poll() is None:
                    client.terminate()
                    client.wait(timeout=5)
            launch.send_signal(signal.SIGINT)
            try:
                launch.wait(timeout=12)
            except subprocess.TimeoutExpired:
                os.killpg(launch.pid, signal.SIGKILL)
                launch.wait()
            node.destroy_node()
            rclpy.shutdown()
            log.flush()
            launch_output = (directory / 'launch.log').read_text()
            assert launch.returncode == 0, f'Launch failed; inspect {directory}'
            assert '[ros2_control_node-2]: process has finished cleanly' in launch_output, (
                f'CM did not exit cleanly; inspect {directory}')
            assert '[mujoco_image_bridge-5]: process has finished cleanly' in launch_output, (
                f'Bridge did not exit cleanly; inspect {directory}')
            print(json.dumps({
                'shutdown': 'cm_and_bridge_clean', 'logs': str(directory),
                'other_process_errors': [line for line in launch_output.splitlines()
                                         if 'process has died' in line],
            }), flush=True)


if __name__ == '__main__':
    main()
