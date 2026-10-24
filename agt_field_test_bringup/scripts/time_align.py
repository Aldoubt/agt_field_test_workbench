#!/usr/bin/env python3
"""Apply explicit timestamp offsets only to streams whose offset is nonzero.

Raw recording topics stay untouched. The LiDAR relay uses a latest-only input
queue because each Livox CustomMsg contains a complete scan; stale scans must
not accumulate behind timestamp correction.
"""
import signal
from rclpy.signals import SignalHandlerOptions
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import (
    QoSProfile, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy,
    qos_profile_sensor_data,
)
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from livox_ros_driver2.msg import CustomMsg


class Align(Node):
    def __init__(self):
        super().__init__('field_time_align')
        self.io = int(float(self.declare_parameter('imu_offset_sec', 0.).value) * 1e9)
        self.lo = int(float(self.declare_parameter('lidar_offset_sec', 0.).value) * 1e9)
        self.align_imu = bool(self.declare_parameter('align_imu', self.io != 0).value) and self.io != 0
        self.align_lidar = bool(self.declare_parameter('align_lidar', self.lo != 0).value) and self.lo != 0
        self.imu_group = MutuallyExclusiveCallbackGroup()
        self.lidar_group = MutuallyExclusiveCallbackGroup()
        self.ip = None
        self.lp = None

        if self.align_imu:
            self.ip = self.create_publisher(Imu, '/field/aligned/imu', 20)
            self.create_subscription(
                Imu, self.declare_parameter('imu_input', '/livox/imu').value,
                self.imu, qos_profile_sensor_data, callback_group=self.imu_group)

        if self.align_lidar:
            # Bound the large scan queue to the newest packet rather than
            # replaying stale point clouds when correction cannot keep up.
            lidar_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
            )
            self.lp = self.create_publisher(CustomMsg, '/field/aligned/lidar', 1)
            self.create_subscription(
                CustomMsg, self.declare_parameter('lidar_input', '/livox/lidar').value,
                self.lidar, lidar_qos, callback_group=self.lidar_group)

        self.get_logger().info(
            'Timestamp correction streams: '
            f'IMU={self.io} ns ({"enabled" if self.align_imu else "bypassed"}), '
            f'LiDAR={self.lo} ns ({"enabled/latest-only" if self.align_lidar else "bypassed"})')

    @staticmethod
    def shift(msg, offset):
        ns = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec + offset
        if ns < 0:
            raise ValueError('timestamp offset underflow')
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(ns, 10**9)

    def imu(self, msg):
        self.shift(msg, self.io)
        self.ip.publish(msg)

    def lidar(self, msg):
        self.shift(msg, self.lo)
        base = int(msg.timebase) + self.lo
        if base < 0:
            raise ValueError('Livox timebase underflow')
        msg.timebase = base
        self.lp.publish(msg)


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Align()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    running = [True]

    def stop(*_):
        running[0] = False
        executor.wake()

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    try:
        while running[0] and rclpy.ok():
            executor.spin_once(timeout_sec=0.02)
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
