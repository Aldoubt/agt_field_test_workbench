#!/usr/bin/env python3
"""Real Humble/DDS tests with RSP and synthetic odometry; no hardware claim."""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import unittest

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

ROOT = Path(os.environ['AGT_FIELD_TEST_WS'])
REPORTS = ROOT/'reports/fwmini_tf_humble'
REPORTS.mkdir(parents=True, exist_ok=True)


class HumbleTFTests(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.node = Node('fwmini_tf_integration_test')
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self.node)
        self.children = []
        self.handles = []
        self.owners = {}
        qos = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.node.create_subscription(TFMessage, '/tf_static', self.static, qos)

    def static(self, msg):
        for t in msg.transforms:
            self.owners.setdefault(t.child_frame_id, set()).add(t.header.frame_id)

    def tearDown(self):
        for process in reversed(self.children):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
        for handle in self.handles:
            handle.close()
        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    def spawn(self, command, name):
        path = REPORTS/(self._testMethodName+'_'+name+'.log')
        handle = path.open('w')
        self.handles.append(handle)
        process = subprocess.Popen(command, stdout=handle, stderr=subprocess.STDOUT, start_new_session=True)
        self.children.append(process)
        return process, path

    def launch(self, *args):
        return self.spawn(['ros2', 'launch', 'agt_field_test_bringup', 'field_test.launch.py',
                           'robot:=fwmini', 'enable_mid360:=false', *args], 'launch')

    def wait(self, predicate, timeout=15):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=.05)
            if predicate():
                return
        self.fail('timed out waiting for actual ROS condition')

    def test_mapping_tf_and_real_adapter_apply_mount_once_then_reject_duplicate(self):
        process, log = self.launch('fwmini_mode:=offline_mapping')
        self.wait(lambda: self.buffer.can_transform('base_footprint', 'livox_frame', Time()))
        tf = self.buffer.lookup_transform('base_footprint', 'livox_frame', Time()).transform
        self.assertAlmostEqual(tf.translation.x, .146725, places=6)
        self.assertAlmostEqual(tf.translation.y, .002608, places=6)
        self.assertAlmostEqual(tf.translation.z, .447040, places=6)
        self.assertAlmostEqual(tf.rotation.y, math.sin(.2152/2), places=6)
        imu = self.buffer.lookup_transform('lidar_link', 'imu_link', Time()).transform
        self.assertAlmostEqual(imu.translation.x, .011, places=6)
        self.assertAlmostEqual(imu.translation.z, -.04412, places=6)
        publisher = self.node.create_publisher(Odometry, '/fastlio2/lio_odom', 10)
        samples = []
        self.node.create_subscription(Odometry, '/mapping/fwmini/base_odometry', samples.append, 10)
        deadline = time.monotonic()+15
        while not samples and time.monotonic() < deadline:
            msg = Odometry()
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.header.frame_id, msg.child_frame_id = 'lidar', 'body'
            msg.pose.pose.orientation.w = 1.0
            publisher.publish(msg)
            rclpy.spin_once(self.node, timeout_sec=.05)
            self.assertIsNone(process.poll(), log.read_text())
        self.assertTrue(samples, log.read_text())
        pos = samples[-1].pose.pose.position
        c, s = math.cos(.2152), math.sin(.2152)
        self.assertAlmostEqual(pos.x, -.011-c*.146725+s*.37704, places=5)
        self.assertAlmostEqual(pos.y, -.02329-.002608, places=5)
        self.assertAlmostEqual(pos.z, .04412-s*.146725-c*.37704, places=5)
        self.wait(lambda: self.buffer.can_transform('lidar', 'base_footprint', Time()))
        footprint = self.buffer.lookup_transform('lidar', 'base_footprint', Time()).transform
        self.assertAlmostEqual(footprint.translation.z, .04412-s*.146725-c*.44704, places=5)
        for child in ('base_link', 'lidar_link', 'livox_frame', 'imu_link'):
            self.assertEqual(len(self.owners[child]), 1)
        endpoints = self.node.get_publishers_info_by_topic('/tf_static')
        self.assertEqual([item.node_name for item in endpoints], ['fwmini_robot_state_publisher'])
        self.spawn(['ros2', 'run', 'tf2_ros', 'static_transform_publisher', '--x', '.9',
                    '--frame-id', 'base_link', '--child-frame-id', 'livox_frame'], 'duplicate')
        self.wait(lambda: process.poll() is not None)
        self.assertIn('duplicate TF publisher/parent', log.read_text())

    def test_candidate_navigation_and_unsupervised_localization_are_blocked(self):
        p, log = self.launch('enable_navigation:=true', 'enable_chassis:=true', 'enable_localization:=true')
        self.wait(lambda: p.poll() is not None)
        self.assertIn('formal navigation requires VERIFIED', log.read_text())
        p, log = self.launch('enable_localization:=true', 'fwmini_mode:=supervised_localization')
        self.wait(lambda: p.poll() is not None)
        self.assertIn('requires supervised:=true', log.read_text())

    def test_missing_tf_fails_real_preflight(self):
        candidate = ROOT/'calibration/fwmini/fwmini_mid360_candidate.yaml'
        p, log = self.spawn(['ros2', 'run', 'agt_field_test_bringup', 'fwmini_tf_contract.py',
                             '--profile', str(candidate), '--preflight', '--timeout', '2'], 'missing_tf')
        self.wait(lambda: p.poll() is not None, timeout=8)
        self.assertEqual(p.returncode, 3, log.read_text())
        self.assertIn('missing mount TF', log.read_text())

    def test_supervised_localization_launch_starts_after_actual_tf_preflight(self):
        cloud = REPORTS/'synthetic_launch_only.pcd'
        cloud.write_text('VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH 4\nHEIGHT 1\nPOINTS 4\nDATA ascii\n0 0 0\n1 0 0\n0 1 0\n1 1 0\n')
        p, log = self.launch('enable_localization:=true', 'fwmini_mode:=supervised_localization',
                             'supervised:=true', 'enable_map_tracking:=false', f'global_map:={cloud}')
        self.wait(lambda: 'FWMINI_TF_READY' in log.read_text())
        self.wait(lambda: 'agt_batch_lio_adapter' in self.node.get_node_names(), timeout=20)
        self.wait(lambda: {'agt_localization_manager', 'agt_global_relocalization'}.issubset(self.node.get_node_names()), timeout=20)
        samples = []
        self.node.create_subscription(Odometry, '/agt/odometry/local', samples.append, 10)
        publisher = self.node.create_publisher(Odometry, '/aft_mapped_to_init', 10)
        deadline = time.monotonic()+15
        while not samples and time.monotonic() < deadline:
            msg = Odometry()
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.header.frame_id, msg.child_frame_id = 'camera_init', 'body'
            msg.pose.pose.orientation.w = 1.0
            publisher.publish(msg)
            rclpy.spin_once(self.node, timeout_sec=.05)
            self.assertIsNone(p.poll(), log.read_text())
        self.assertTrue(samples, log.read_text())
        c, s = math.cos(.2152), math.sin(.2152)
        self.assertAlmostEqual(samples[-1].pose.pose.position.x, -.011-c*.146725+s*.37704, places=5)
        self.assertAlmostEqual(samples[-1].pose.pose.position.z, .04412-s*.146725-c*.37704, places=5)
        self.assertIsNone(p.poll(), log.read_text())
        self.assertNotIn('process has died', log.read_text())
        self.assertNotIn('fwmini_chassis_adapter', self.node.get_node_names())
        self.assertNotIn('yhs_can_control', self.node.get_node_names())


if __name__ == '__main__':
    suite = unittest.defaultTestLoader.loadTestsFromTestCase(HumbleTFTests)
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    (REPORTS/'result.json').write_text(json.dumps({'tests': result.testsRun,
        'failures': len(result.failures), 'errors': len(result.errors),
        'environment': 'Ubuntu 22.04 / ROS 2 Humble Docker',
        'data': 'synthetic odometry and launch-only point cloud',
        'physical_calibration_verified': False}, indent=2))
    raise SystemExit(not result.wasSuccessful())
