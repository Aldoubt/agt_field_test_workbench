#!/usr/bin/env python3
"""Isolated, receive-only replay of the recorded mapping and navigation LIO paths."""
import argparse
import csv
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rclpy.parameter import Parameter
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import Trigger
from agt_robot_interfaces.msg import AdapterStatus
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import yaml

ROOT = Path('/home/yangxuan/agt_field_test_ws')
EXP = Path(__file__).resolve().parent
BAG = ROOT / 'maps/20261024T112750Z-c714f7dc/raw_bag'
BUNDLE = ROOT / 'runs/20261024T112400Z-08259a40/calibration.yaml'


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--name', required=True)
    p.add_argument('--domain', required=True)
    p.add_argument('--mode', choices=('mapping', 'navigation'), default='mapping')
    p.add_argument('--variant', choices=('baseline', 'corrected', 'installed'), required=True)
    p.add_argument('--limit', type=float, default=0.)
    p.add_argument('--native-align', action='store_true')
    args = p.parse_args()
    output = EXP / args.name
    output.mkdir(exist_ok=False)
    os.environ.update(ROS_DOMAIN_ID=args.domain, ROS_LOCALHOST_ONLY='1',
                      OMP_NUM_THREADS='2', ROS_LOG_DIR=str(output / 'ros_logs'))
    if args.variant == 'baseline':
        prefix = EXP / 'baseline/prefix'
        binary = EXP / 'baseline/lio_node'
        (prefix / 'share/ament_index/resource_index/packages').mkdir(parents=True, exist_ok=True)
        (prefix / 'share/ament_index/resource_index/packages/fastlio2').touch()
        (prefix / 'lib/fastlio2').mkdir(parents=True, exist_ok=True)
        if not (prefix / 'lib/fastlio2/lio_node').exists():
            (prefix / 'lib/fastlio2/lio_node').symlink_to(binary)
        if not (prefix / 'share/fastlio2').exists():
            (prefix / 'share/fastlio2').symlink_to(ROOT / 'install/fastlio2/share/fastlio2')
    elif args.variant == 'installed':
        prefix = ROOT / 'install/fastlio2'
        binary = prefix / 'lib/fastlio2/lio_node'
    else:
        prefix = EXP / 'install/fastlio2'
        binary = prefix / 'lib/fastlio2/lio_node'
    os.environ['AMENT_PREFIX_PATH'] = str(prefix) + ':' + os.environ.get('AMENT_PREFIX_PATH', '')
    if args.native_align:
        os.environ['AMENT_PREFIX_PATH'] = str(EXP / 'align_prefix') + ':' + os.environ['AMENT_PREFIX_PATH']
    config = yaml.safe_load((EXP / 'original_fastlio.yaml').read_text())
    if args.mode == 'navigation':
        config.update(world_frame='odom', lidar_topic='/livox/lidar', imu_topic='/livox/imu')
    (output / 'fastlio.yaml').write_text(yaml.safe_dump(config, sort_keys=False))
    metadata = dict(vars(args), binary=str(binary), bag=str(BAG),
                    calibration=str(BUNDLE), motion=False, state='STARTING')
    (output / 'case.json').write_text(json.dumps(metadata, indent=2) + '\n')
    children, handles = [], []

    def spawn(command, label):
        stream = (output / (label + '.log')).open('w')
        handles.append(stream)
        process = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        children.append(process)
        return process

    def stop(process):
        if process.poll() is not None:
            return
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=12)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=5)

    rclpy.init()
    node = rclpy.create_node('offline_drift_observer', parameter_overrides=[
        Parameter('use_sim_time', value=True)])
    counts, ages, last_pose, tf_owners = {}, {}, {}, {}
    filter_summary = {'messages': 0, 'last': None}
    adapter_summary = {'messages': 0, 'last': None}
    pose_files = {}

    def pose(topic, message):
        if topic not in pose_files:
            stream = (output / (topic + '_poses.csv')).open('w')
            handles.append(stream)
            writer = csv.writer(stream)
            writer.writerow(['stamp', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])
            pose_files[topic] = writer
        pos, q = message.pose.pose.position, message.pose.pose.orientation
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        pose_files[topic].writerow([stamp, pos.x, pos.y, pos.z, q.x, q.y, q.z, q.w])
        counts[topic] = counts.get(topic, 0) + 1
        ages[topic] = node.get_clock().now().nanoseconds * 1e-9 - stamp
        last_pose[topic] = [stamp, pos.x, pos.y, pos.z]

    def status(message):
        adapter_summary['messages'] += 1
        adapter_summary['last'] = {name: getattr(message, name) for name in
            ('accepted_count', 'rejected_count', 'state', 'last_error')
            if hasattr(message, name)}

    def filter_status(message):
        filter_summary['messages'] += 1
        try:
            filter_summary['last'] = json.loads(message.data)
        except ValueError:
            filter_summary['last'] = message.data

    def tf(message):
        for t in message.transforms:
            key = t.header.frame_id + '->' + t.child_frame_id
            tf_owners.setdefault(key, set()).add('OBSERVED_WITHOUT_GID')

    node.create_subscription(Odometry, '/fastlio2/lio_odom', lambda m: pose('frontend', m), 100)
    canonical = '/agt/odometry/local' if args.mode == 'navigation' else '/mapping/base_odometry'
    node.create_subscription(Odometry, canonical, lambda m: pose('canonical', m), 100)
    node.create_subscription(AdapterStatus, '/agt/odometry/adapter_status', status, 20)
    node.create_subscription(String, '/mapping/self_filter/status', filter_status, 20)
    node.create_subscription(TFMessage, '/tf', tf, 100)
    node.create_subscription(TFMessage, '/tf_static', tf, QoSProfile(
        depth=20, reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL))
    export = node.create_client(Trigger, '/mapping/fastlio/export')
    try:
        spawn(['ros2', 'launch', 'agt_field_test_bringup', 'field_test.launch.py',
               'robot:=bunker', 'enable_chassis:=false', 'enable_mid360:=false',
               'enable_localization:=false', 'enable_navigation:=false',
               'calibration_bundle:=' + str(BUNDLE), 'lio_backend:=batch_lio',
               'use_sim_time:=true'], 'mount')
        if args.mode == 'mapping':
            frontend = spawn(['ros2', 'launch', 'agt_field_test_bringup', 'no_pgo.launch.py',
                   'lio_config:=' + str(output / 'fastlio.yaml'),
                   'output:=' + str(output / 'artifact'), 'use_sim_time:=true',
                   'base_adapter:=true', 'self_filter_enabled:=true'], 'frontend')
        else:
            if args.native_align:
                spawn(['ros2', 'run', 'agt_field_test_bringup', 'time_align.py',
                       '--ros-args', '-p', 'use_sim_time:=true'], 'alignment')
            input_args = (['lidar_topic:=/field/aligned/lidar',
                           'imu_topic:=/field/aligned/imu'] if args.native_align else [])
            frontend = spawn(['ros2', 'launch', 'agt_navigation_runtime',
                   'fastlio_navigation_lio.launch.py', 'use_sim_time:=true',
                   'fastlio_config:=' + str(output / 'fastlio.yaml'), *input_args], 'frontend')
            spawn(['ros2', 'run', 'agt_field_test_bringup', 'lio_artifact', '--ros-args',
                   '-p', 'use_sim_time:=true', '-p', 'world_frame:=odom',
                   '-p', 'output_dir:=' + str(output / 'artifact')], 'exporter')
        deadline = time.monotonic() + 25
        while not export.service_is_ready():
            if frontend.poll() is not None or time.monotonic() > deadline:
                raise RuntimeError('LIO/exporter startup failed')
            rclpy.spin_once(node, timeout_sec=.1)
        # Let the TF preflight and static mounting discovery complete.
        ready = time.monotonic() + 3
        while time.monotonic() < ready:
            rclpy.spin_once(node, timeout_sec=.1)
        player = spawn(['ros2', 'bag', 'play', str(BAG), '--clock', '100',
                        '--rate', '1.0', '--disable-keyboard-controls', '--topics',
                        '/livox/lidar', '/livox/imu'], 'playback')
        begin = time.monotonic()
        metadata['state'] = 'REPLAYING'
        last_write = 0.
        while player.poll() is None:
            rclpy.spin_once(node, timeout_sec=.05)
            elapsed = time.monotonic() - begin
            if frontend.poll() is not None:
                raise RuntimeError('frontend exited during replay')
            if elapsed - last_write > 5:
                progress = dict(elapsed_sec=round(elapsed, 1), counts=counts,
                                last_pose=last_pose, last_age_sec=ages,
                                adapter=adapter_summary, self_filter=filter_summary)
                (output / 'progress.json').write_text(json.dumps(progress, indent=2) + '\n')
                last_write = elapsed
            if args.limit and elapsed >= args.limit:
                stop(player)
                break
        if not args.limit and player.returncode != 0:
            raise RuntimeError('playback failed: ' + str(player.returncode))
        drain = time.monotonic() + 5
        while time.monotonic() < drain:
            rclpy.spin_once(node, timeout_sec=.05)
        future = export.call_async(Trigger.Request())
        deadline = time.monotonic() + 45
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
        if not future.done() or not future.result().success:
            raise RuntimeError('export failed')
        metadata.update(state='COMPLETE', elapsed_sec=time.monotonic()-begin,
                        counts=counts, last_pose=last_pose, last_age_sec=ages,
                        adapter=adapter_summary, self_filter=filter_summary,
                        tf_edges=sorted(tf_owners),
                        tf_publishers=[dict(node_name=i.node_name,
                            node_namespace=i.node_namespace,
                            gid=bytes(i.endpoint_gid).hex())
                            for i in node.get_publishers_info_by_topic('/tf')],
                        export_message=future.result().message)
    except BaseException as e:
        metadata.update(state='FAILED', error=repr(e))
        raise
    finally:
        for process in reversed(children):
            stop(process)
        node.destroy_node()
        rclpy.shutdown()
        for stream in handles:
            stream.close()
        (output / 'case.json').write_text(json.dumps(metadata, indent=2) + '\n')
        print(json.dumps(metadata, indent=2), flush=True)


if __name__ == '__main__':
    main()
