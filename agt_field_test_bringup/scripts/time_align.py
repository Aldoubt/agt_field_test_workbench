#!/usr/bin/env python3
"""Explicit constant timestamp correction; raw recording topics stay untouched."""
import math
import signal
from rclpy.signals import SignalHandlerOptions
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from livox_ros_driver2.msg import CustomMsg

class Align(Node):
    def __init__(self):
        super().__init__('field_time_align')
        self.io=int(float(self.declare_parameter('imu_offset_sec',0.).value)*1e9)
        self.lo=int(float(self.declare_parameter('lidar_offset_sec',0.).value)*1e9)
        self.ip=self.create_publisher(Imu,'/field/aligned/imu',20)
        self.lp=self.create_publisher(CustomMsg,'/field/aligned/lidar',20)
        self.create_subscription(Imu,self.declare_parameter('imu_input','/livox/imu').value,self.imu,qos_profile_sensor_data)
        self.create_subscription(CustomMsg,self.declare_parameter('lidar_input','/livox/lidar').value,self.lidar,qos_profile_sensor_data)
    def shift(self,msg,offset):
        ns=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec+offset
        if ns<0: raise ValueError('timestamp offset underflow')
        msg.header.stamp.sec,msg.header.stamp.nanosec=divmod(ns,10**9)
    def imu(self,msg):
        self.shift(msg,self.io);self.ip.publish(msg)
    def lidar(self,msg):
        self.shift(msg,self.lo)
        base=int(msg.timebase)+self.lo
        if base<0: raise ValueError('Livox timebase underflow')
        msg.timebase=base;self.lp.publish(msg)

def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO);node=Align();running=[True]
    def stop(*_):running[0]=False
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    try:
        while running[0] and rclpy.ok():rclpy.spin_once(node,timeout_sec=.02)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
