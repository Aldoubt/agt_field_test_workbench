#!/usr/bin/env python3
"""No CAN: real Humble adapter + guard + simulated manufacturer feedback."""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import unittest

os.environ.setdefault('ROS_DOMAIN_ID', '216')
import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger
from rclpy.qos import QoSProfile, DurabilityPolicy
from yhs_can_interfaces.msg import ChassisInfoFb, IoFb, CtrlCmd

ROOT = Path(os.environ.get('AGT_FIELD_TEST_WS', str(Path(__file__).resolve().parents[3])))
REPORTS = ROOT/'reports/fwmini_control_humble'
REPORTS.mkdir(parents=True, exist_ok=True)


class ControlTests(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.node = rclpy.create_node('fwmini_simulated_manufacturer')
        self.processes = []
        self.logs = []
        self.samples = []
        self.status = {}
        self.last_output_gear = 0
        self.fixed_gear = None
        self.moving = 0.0
        self.estop = self.remote = False
        self.emit_ctrl = self.emit_io = True
        self.feedback_stamp = None
        self.keepalive = False
        self.started = time.monotonic()
        self.evidence = (REPORTS/(self._testMethodName+'.jsonl')).open('w')
        self.node.create_subscription(CtrlCmd, '/ctrl_cmd', self.on_cmd, 100)
        self.node.create_subscription(String, '/fwmini/control_status', self.on_status, 10)
        self.ctrl = self.node.create_publisher(ChassisInfoFb, '/chassis_info_fb', 10)
        self.io = self.node.create_publisher(IoFb, '/fwmini/io_fb', 1)
        self.manual = self.node.create_publisher(Twist, '/agt/hmi/cmd_vel', 10)
        self.raw = self.node.create_publisher(Twist, '/cmd_vel', 10)
        self.external = self.node.create_publisher(Bool, '/fwmini/external_stop', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.adapter = self.spawn(['ros2', 'run', 'fwmini_chassis_adapter', 'fwmini_chassis_adapter_node',
                                   '--ros-args', '-p', 'switch_timeout:=1.2'], 'adapter')
        self.guard = self.spawn(['ros2', 'run', 'agt_base_control', 'cmd_vel_guard', '--ros-args',
                                '-p', 'kinematic_profile:=fwmini', '-p', 'output_topic:=/fwmini/cmd_vel_guarded',
                                '-p', 'default_control_mode:=manual', '-p', 'require_localization_status:=false',
                                '-p', 'command_timeout_sec:=0.2', '-p', 'max_linear_x:=0.05',
                                '-p', 'max_reverse_x:=0.03', '-p', 'max_linear_y:=0.03',
                                '-p', 'max_angular_z:=0.15'], 'guard')
        self.addCleanup(self.cleanup)
        self.wait(lambda: self.status.get('state') == 'DISABLED' and self.node.count_publishers('/fwmini/cmd_vel_guarded') == 1)
        self.timer = self.node.create_timer(.02, self.feedback)
        self.spin(.15)

    def spawn(self, command, label):
        handle = (REPORTS/(self._testMethodName+'_'+label+'.log')).open('w')
        self.logs.append(handle)
        process = subprocess.Popen(command, stdout=handle, stderr=subprocess.STDOUT, start_new_session=True)
        self.processes.append(process)
        return process

    def on_status(self, msg):
        self.status = json.loads(msg.data)

    def on_cmd(self, msg):
        sample = {'time_sec': time.monotonic()-self.started, 'gear': msg.ctrl_cmd_gear,
                  'vx_mps': msg.ctrl_cmd_x_linear, 'vy_mps': msg.ctrl_cmd_y_linear,
                  'wz_degps': msg.ctrl_cmd_z_angular,
                  'last_status': dict(self.status)}
        self.last_output_gear = msg.ctrl_cmd_gear
        self.samples.append(sample)
        self.evidence.write(json.dumps(sample, allow_nan=False)+'\n')
        self.evidence.flush()

    def feedback(self):
        if self.keepalive:
            self.manual.publish(Twist())
        io = IoFb()
        io.io_fb_estop, io.io_fb_joypad_ctrl = self.estop, self.remote
        if self.emit_io:
            self.io.publish(io)
        if self.emit_ctrl:
            msg = ChassisInfoFb()
            msg.header.stamp = self.feedback_stamp or self.node.get_clock().now().to_msg()
            msg.ctrl_fb.ctrl_fb_gear = self.last_output_gear if self.fixed_gear is None else self.fixed_gear
            msg.ctrl_fb.ctrl_fb_x_linear = self.moving
            msg.io_fb = io
            self.ctrl.publish(msg)

    def wait(self, predicate, timeout=5):
        end = time.monotonic()+timeout
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=.005)
            if predicate():
                return
        self.fail('ROS timeout: '+json.dumps(self.status))

    def spin(self, duration):
        end = time.monotonic()+duration
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=.005)

    def call(self, name, value=None):
        kind = Trigger if value is None else SetBool
        client = self.node.create_client(kind, name)
        self.assertTrue(client.wait_for_service(timeout_sec=3), name)
        request = Trigger.Request() if value is None else SetBool.Request(data=value)
        future = client.call_async(request)
        self.wait(future.done)
        response = future.result()
        self.node.destroy_client(client)
        return response

    def select(self, mode):
        # Recovery after a deliberately stopped feedback publisher must observe
        # newly received healthy feedback before requesting a new handshake.
        self.wait(lambda: 0 <= self.status.get('feedback_age_sec', -1) < .08
                  and 0 <= self.status.get('io_feedback_age_sec', -1) < .08)
        response = self.call('/fwmini/select_'+mode)
        self.assertTrue(response.success, response.message)
        self.wait(lambda: self.status.get('state') == 'READY')
        self.assertEqual(self.status['actual_gear'], 6 if mode == 'four_wheel_steer' else 8)

    def enable(self):
        self.wait(lambda: self.status.get('guard_epoch_seen') == self.status.get('epoch')
                  and 0 <= self.status.get('guard_feedback_age_sec', -1) < .1)
        response = self.call('/fwmini/enable_motion', True)
        self.assertTrue(response.success, response.message)
        self.wait(lambda: self.status.get('state') == 'ACTIVE')
        self.wait(lambda: self.status.get('guard_epoch_seen') == self.status.get('epoch'))

    @staticmethod
    def zero(sample):
        return all(sample[k] == 0 for k in ('vx_mps', 'vy_mps', 'wz_degps'))

    @staticmethod
    def velocity(x=0., y=0., z=0.):
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = x, y, z
        return msg

    def drive(self, msg, duration=.14):
        start = len(self.samples)
        end = time.monotonic()+duration
        while time.monotonic() < end:
            self.manual.publish(msg)
            self.spin(.02)
        return self.samples[start:]

    def assert_stopped(self, duration=.10):
        self.spin(.04)  # consume any pre-fault samples already in flight
        start = len(self.samples)
        self.spin(duration)
        self.assertTrue(self.samples[start:])
        self.assertTrue(all(self.zero(s) for s in self.samples[start:]), self.samples[start:])

    def test_startup_is_disabled_and_single_owner(self):
        self.assertTrue(self.samples)
        self.assertTrue(all(s['gear'] == 0 and self.zero(s) for s in self.samples))
        self.assertFalse(self.call('/fwmini/enable_motion', True).success)
        self.assertEqual(self.node.count_publishers('/ctrl_cmd'), 1)
        self.assertEqual(self.node.count_subscribers('/cmd_vel'), 0)
        state = json.loads(self.call('/fwmini/get_control_state').message)
        for key in ('requested_mode', 'target_gear', 'actual_gear', 'feedback_age_sec', 'io_feedback_age_sec',
                    'motion_allowed', 'command_mps_radps', 'reason', 'last_mode_switch_result'):
            self.assertIn(key, state)
        second = self.spawn(['ros2', 'run', 'fwmini_chassis_adapter', 'fwmini_chassis_adapter_node'], 'second_owner')
        self.wait(lambda: second.poll() is not None)
        self.assertNotEqual(second.returncode, 0)
        self.assertEqual(self.node.count_publishers('/ctrl_cmd'), 1)

    def test_modes_stop_before_shift_and_require_new_commands(self):
        self.moving = .10
        self.call('/fwmini/select_four_wheel_steer')
        self.spin(.18)
        self.assertEqual(self.status['state'], 'STOPPING')
        self.assertTrue(all(s['gear'] == 0 and self.zero(s) for s in self.samples))
        self.moving = 0.
        self.wait(lambda: self.status.get('state') == 'READY')
        self.assertTrue(any(s['gear'] == 6 for s in self.samples))
        self.assertTrue(all(self.zero(s) for s in self.samples))
        self.enable()
        samples = self.drive(self.velocity(.05, 0., .10))
        self.assertTrue(any(s['vx_mps'] > 0 and s['wz_degps'] > 0 for s in samples))
        self.assertTrue(all(s['vy_mps'] == 0 for s in samples))
        old_epoch = self.status['epoch']
        self.call('/fwmini/select_lateral')
        self.spin(.04)
        start = len(self.samples)
        self.wait(lambda: self.status.get('state') == 'READY')
        self.assertTrue(all(self.zero(s) for s in self.samples[start:]))
        self.assertTrue(any(s['gear'] == 8 for s in self.samples[start:]))
        self.enable()
        self.assertNotEqual(self.status['epoch'], old_epoch)
        # No new upstream command: cached 4T4D velocity must never reappear.
        self.assert_stopped(.06)
        samples = self.drive(self.velocity(.02, .03, 0.))
        self.assertTrue(any(s['vy_mps'] > 0 for s in samples))
        self.assertTrue(all(s['wz_degps'] == 0 for s in samples))

    def test_guard_rejects_unsupported_axes_and_nonfinite_commands(self):
        for mode, msg in [('four_wheel_steer', self.velocity(0., .01, 0.)),
                          ('lateral', self.velocity(0., 0., .01)),
                          ('four_wheel_steer', self.velocity(float('nan'), 0., 0.)),
                          ('lateral', self.velocity(0., float('inf'), 0.))]:
            with self.subTest(mode=mode, msg=repr(msg)):
                self.select(mode)
                self.enable()
                self.manual.publish(msg)
                self.wait(lambda: self.status.get('state') == 'FAULT')
                self.assertIn(self.status['reason'], ('guard:unsupported_velocity_axis', 'guard:nonfinite_command'))
                self.assert_stopped()
                self.assertFalse(self.call('/fwmini/enable_motion', True).success)

    def test_adapter_independently_rejects_axis_and_stale_epoch(self):
        # Keep the upstream watchdog satisfied while isolating adapter validation.
        self.keepalive = True
        direct = self.node.create_publisher(TwistStamped, '/fwmini/cmd_vel_guarded', 1)
        self.wait(lambda: direct.get_subscription_count() > 0)
        self.select('four_wheel_steer')
        self.enable()
        old_epoch = self.status['epoch']
        self.call('/fwmini/select_lateral')
        self.wait(lambda: self.status.get('state') == 'READY')
        self.enable()
        msg = TwistStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = 'fwmini_epoch_'+str(old_epoch)
        msg.twist = self.velocity(.05, 0., .1)
        end = time.monotonic()+.06
        while time.monotonic() < end:
            msg.header.stamp = self.node.get_clock().now().to_msg()
            direct.publish(msg)
            self.spin(.01)
        self.assertEqual(self.status['state'], 'ACTIVE')
        self.assert_stopped(.03)
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = 'fwmini_epoch_'+str(self.status['epoch'])
        # Depth-1 guarded QoS can replace a one-shot packet with a concurrent
        # guard zero; keep injecting until the adapter actually observes it.
        end = time.monotonic()+2
        while self.status.get('state') != 'FAULT' and time.monotonic() < end:
            msg.header.stamp = self.node.get_clock().now().to_msg()
            direct.publish(msg)
            self.spin(.01)
        self.assertEqual(self.status.get('state'), 'FAULT')
        self.assertEqual(self.status['reason'], 'unsupported_velocity_axis')
        self.assert_stopped()

    def test_mismatched_and_discontinuous_feedback_never_ready(self):
        self.fixed_gear = 8
        self.call('/fwmini/select_four_wheel_steer')
        self.wait(lambda: self.status.get('state') == 'SWITCHING')
        self.spin(.12)
        self.assertFalse(self.call('/fwmini/enable_motion', True).success)
        self.assertEqual(self.status['state'], 'SWITCHING')
        self.wait(lambda: self.status.get('state') == 'FAULT')
        self.assertEqual(self.status['last_mode_switch_result'], 'FAILED:mode_switch_timeout')
        self.assert_stopped()
        self.fixed_gear = None
        self.call('/fwmini/select_lateral')
        self.wait(lambda: self.status.get('state') == 'SWITCHING')
        self.emit_ctrl = False
        self.spin(.25)
        self.assertEqual(self.status['state'], 'FAULT')
        self.assertEqual(self.status['reason'], 'ctrl_feedback_stale')
        self.assert_stopped()

    def test_disable_stop_estop_authority_and_feedback_expiry_clear_intent(self):
        for fault in ('disable', 'stop', 'estop', 'remote', 'ctrl_stale', 'io_stale', 'external'):
            with self.subTest(fault=fault):
                self.estop = self.remote = False
                self.emit_ctrl = self.emit_io = True
                self.external.publish(Bool(data=False))
                self.spin(.06)
                self.select('lateral')
                self.enable()
                self.assertTrue(any(s['vy_mps'] > 0 for s in self.drive(self.velocity(0., .03, 0.))))
                if fault == 'disable':
                    self.call('/fwmini/enable_motion', False)
                elif fault == 'stop':
                    self.call('/fwmini/stop')
                elif fault == 'estop':
                    self.estop = True
                elif fault == 'remote':
                    self.remote = True
                elif fault == 'ctrl_stale':
                    self.emit_ctrl = False
                elif fault == 'io_stale':
                    self.emit_io = False
                else:
                    self.external.publish(Bool(data=True))
                # Keep upstream commands fresh to isolate the selected safety input.
                end = time.monotonic()+1
                while self.status.get('state') not in ('DISABLED', 'FAULT') and time.monotonic() < end:
                    self.manual.publish(self.velocity(0., .03, 0.))
                    self.spin(.02)
                self.assertIn(self.status.get('state'), ('DISABLED', 'FAULT'))
                expected = {'disable': 'operator_disabled', 'stop': 'external_stop_service',
                            'estop': 'emergency_stop', 'remote': 'control_authority_lost',
                            'ctrl_stale': 'ctrl_feedback_stale', 'io_stale': 'io_feedback_stale',
                            'external': 'external_stop'}
                self.assertEqual(self.status['reason'], expected[fault])
                self.assert_stopped()
                self.assertEqual(self.status['output_gear'], 0)
                self.estop = self.remote = False
                self.emit_ctrl = self.emit_io = True
                self.external.publish(Bool(data=False))
                self.spin(.06)
                self.select('lateral')
                self.enable()
                self.assert_stopped(.05)  # Explicit recovery still cannot replay intent.
                self.call('/fwmini/enable_motion', False)

    def test_upstream_watchdog_and_feedback_stamp_validation(self):
        self.select('four_wheel_steer')
        self.enable()
        self.drive(self.velocity(.05, 0., 0.))
        self.wait(lambda: self.status.get('state') == 'FAULT')
        self.assertIn(self.status['reason'], ('guard:upstream_command_timeout', 'command_timeout'))
        self.assert_stopped()
        self.select('lateral')
        self.feedback_stamp = self.node.get_clock().now().to_msg()
        self.spin(.08)
        self.assertEqual(self.status['state'], 'FAULT')
        self.assertEqual(self.status['reason'], 'invalid_ctrl_feedback_stamp')

    def test_lifecycle_stops_zero_and_foreign_owner_forces_exit(self):
        self.select('lateral')
        self.enable()
        self.drive(self.velocity(0., .03, 0.))
        os.killpg(self.adapter.pid, signal.SIGINT)
        self.wait(lambda: self.adapter.poll() is not None)
        self.assertEqual(self.adapter.returncode, 0)
        self.spin(.08)
        self.assertEqual(self.samples[-1]['gear'], 0)
        self.assertTrue(self.zero(self.samples[-1]))
        self.wait(lambda: self.node.count_publishers('/ctrl_cmd') == 0)
        old_epoch = self.status['epoch']
        self.adapter = self.spawn(['ros2', 'run', 'fwmini_chassis_adapter', 'fwmini_chassis_adapter_node'], 'restart')
        self.wait(lambda: self.status.get('state') == 'DISABLED' and self.status.get('epoch') != old_epoch)
        self.assert_stopped()
        foreign = self.node.create_publisher(CtrlCmd, '/ctrl_cmd', 1)
        self.wait(lambda: self.adapter.poll() is not None)
        self.assertNotEqual(self.adapter.returncode, 0)
        self.node.destroy_publisher(foreign)

    def test_bunker_keeps_nonholonomic_constraints(self):
        outputs = []
        self.node.create_subscription(Twist, '/test/bunker_output', outputs.append, 10)
        publisher = self.node.create_publisher(Twist, '/test/bunker_input', 10)
        self.spawn(['ros2', 'run', 'agt_base_control', 'cmd_vel_guard', '--ros-args',
                    '-p', 'kinematic_profile:=bunker', '-p', 'default_control_mode:=manual',
                    '-p', 'manual_input_topic:=/test/bunker_input', '-p', 'output_topic:=/test/bunker_output',
                    '-p', 'require_localization_status:=false'], 'bunker_guard')
        self.wait(lambda: publisher.get_subscription_count() > 0 and outputs)
        end = time.monotonic()+.25
        while time.monotonic() < end:
            publisher.publish(self.velocity(.05, 0., .1))
            self.spin(.02)
        self.assertTrue(any(m.linear.x > 0 and m.angular.z > 0 for m in outputs))
        self.assertTrue(all(m.linear.y == 0 for m in outputs))
        publisher.publish(self.velocity(.05,.03,.1));self.spin(.05)
        self.assertEqual(outputs[-1].linear.x,0.)
        self.assertEqual(outputs[-1].angular.z,0.)

    def cleanup(self):
        failed_shutdowns = []
        for process in reversed(self.processes):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                end = time.monotonic()+4
                while process.poll() is None and time.monotonic() < end:
                    rclpy.spin_once(self.node, timeout_sec=.01)
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=3)
                if process.returncode != 0:
                    failed_shutdowns.append(process.returncode)
        for handle in self.logs:
            handle.close()
        self.evidence.close()
        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        self.assertFalse(failed_shutdowns, "normal ROS node shutdown must exit 0: "+repr(failed_shutdowns))


if __name__ == '__main__':
    suite = unittest.defaultTestLoader.loadTestsFromTestCase(ControlTests)
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    (REPORTS/'result.json').write_text(json.dumps({
        'tests': result.testsRun, 'failures': len(result.failures), 'errors': len(result.errors),
        'environment': 'Ubuntu 22.04 / ROS 2 Humble Docker', 'ros_domain_id': os.environ['ROS_DOMAIN_ID'],
        'can_driver_started': False, 'feedback': 'SIMULATED_MANUFACTURER', 'physical_acceptance': 'NOT_RUN',
    }, indent=2)+'\n')
    raise SystemExit(0 if result.wasSuccessful() else 1)
