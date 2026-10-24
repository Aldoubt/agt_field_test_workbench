#!/usr/bin/env python3
"""Offline transport regression: 200 Hz IMU + 20k-point 10 Hz scans, byte-exact."""
import argparse
import json
import os
from pathlib import Path
import signal
import struct
import subprocess
import time

import rclpy
from rclpy.serialization import serialize_message
from sensor_msgs.msg import Imu
from livox_ros_driver2.msg import CustomMsg, CustomPoint


def check(binary, domain, imu_offset, lidar_offset):
    os.environ.update(ROS_DOMAIN_ID=str(domain), ROS_LOCALHOST_ONLY='1')
    log = open(os.devnull, 'w')
    process = subprocess.Popen([str(binary), '--ros-args',
        '-p', 'imu_offset_sec:=' + str(imu_offset),
        '-p', 'lidar_offset_sec:=' + str(lidar_offset),
        '-p', 'imu_input:=/test/imu', '-p', 'lidar_input:=/test/lidar'],
        stdout=log, stderr=log, start_new_session=True)
    rclpy.init()
    node = rclpy.create_node('time_align_transport_test')
    imu_pub = node.create_publisher(Imu, '/test/imu', 1000)
    lidar_pub = node.create_publisher(CustomMsg, '/test/lidar', 100)
    expected = {'imu': {}, 'lidar': {}}
    received = {'imu': set(), 'lidar': set()}
    errors = []

    def observe(kind, raw):
        sec, nsec = struct.unpack_from('<iI', raw, 4)
        stamp = sec * 10**9 + nsec
        if stamp not in expected[kind] or bytes(raw) != expected[kind].get(stamp):
            errors.append((kind, stamp, 'unmatched bytes'))
        received[kind].add(stamp)

    node.create_subscription(Imu, '/field/aligned/imu',
        lambda raw: observe('imu', raw), 1000, raw=True)
    node.create_subscription(CustomMsg, '/field/aligned/lidar',
        lambda raw: observe('lidar', raw), 100, raw=True)
    imu = Imu()
    imu.header.frame_id = 'imu'
    imu.linear_acceleration.z = 1.0
    imu.angular_velocity.x = .125
    cloud = CustomMsg()
    cloud.header.frame_id = 'livox_frame'
    cloud.point_num = 20000
    cloud.points = [CustomPoint(offset_time=i*5000, x=1.0, y=2.0, z=3.0,
                               reflectivity=20, tag=0, line=i%4) for i in range(20000)]
    templates = {'imu': serialize_message(imu), 'lidar': serialize_message(cloud)}
    string_length = struct.unpack_from('<I', templates['lidar'], 12)[0]
    timebase_position = 4 + ((16+string_length-4+7)//8)*8
    try:
        deadline = time.monotonic()+8
        while (imu_pub.get_subscription_count() < 1 or lidar_pub.get_subscription_count() < 1):
            if process.poll() is not None or time.monotonic() > deadline:
                raise RuntimeError('native node discovery failed')
            rclpy.spin_once(node, timeout_sec=.05)
        # Discover the return publishers before starting the measured stream.
        ready = time.monotonic()+1
        while time.monotonic() < ready:
            rclpy.spin_once(node, timeout_sec=.02)

        def publish(kind, index, stamp, offset, publisher):
            raw = bytearray(templates[kind])
            struct.pack_into('<iI', raw, 4, *divmod(stamp, 10**9))
            if kind == 'lidar':
                struct.pack_into('<Q', raw, timebase_position, stamp)
            shifted = stamp + int(offset*10**9)
            output = bytearray(raw)
            struct.pack_into('<iI', output, 4, *divmod(shifted, 10**9))
            if kind == 'lidar':
                struct.pack_into('<Q', output, timebase_position, shifted)
            expected[kind][shifted] = bytes(output)
            publisher.publish(bytes(raw))

        begin = time.monotonic()
        for i in range(400):
            deadline = begin + i*.005
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=min(.001, max(0.,deadline-time.monotonic())))
            stamp = 1000*10**9 + 995000000 + i*5000000
            publish('imu', i, stamp, imu_offset, imu_pub)
            if i % 20 == 0:
                publish('lidar', i//20, stamp, lidar_offset, lidar_pub)
            rclpy.spin_once(node, timeout_sec=0)
        deadline = time.monotonic()+4
        while time.monotonic() < deadline and any(len(received[k]) < len(expected[k]) for k in expected):
            rclpy.spin_once(node, timeout_sec=.01)
        report = dict(imu_offset_sec=imu_offset, lidar_offset_sec=lidar_offset,
            expected={k: len(v) for k,v in expected.items()},
            received={k: len(v) for k,v in received.items()}, errors=errors)
        assert not errors and all(len(received[k]) == len(expected[k]) for k in expected), report
        return report
    finally:
        node.destroy_node()
        rclpy.shutdown()
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            process.wait(timeout=5)
        log.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--binary', required=True, type=Path)
    parser.add_argument('--domain', type=int, default=142)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    reports = [check(args.binary, args.domain+i, *offsets)
               for i,offsets in enumerate(((0.0,0.0),(.012,-.003)))]
    text = json.dumps(reports, indent=2)+'\n'
    if args.output:
        args.output.write_text(text)
    print(text)
