#!/usr/bin/env python3
"""Optional quality/position prior on global candidates. No odom/TF publishers."""
import hashlib,json,math,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import NavSatFix,Imu
from agt_asensing_driver.msg import INSStatus
from std_msgs.msg import String
from agt_robot_interfaces.msg import LocalizationStatus
from tf2_ros import Buffer,TransformListener
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from agt_field_test_bringup.gps import prior,load_alignment
class Gate(Node):
    def __init__(self):
        super().__init__('field_gps_candidate_gate');self.messages={}
        path=self.declare_parameter('alignment_file','').value
        self.alignment=load_alignment(path,self.declare_parameter('map_sha256','').value,self.declare_parameter('calibration_sha256','').value)
        self.buffer=Buffer();self.listener=TransformListener(self.buffer,self);self.localized_at=0.
        self.create_subscription(LocalizationStatus,'/agt/localization/status',self.localization,10)
        self.pub=self.create_publisher(PoseWithCovarianceStamped,'/agt/relocalization/pose',10)
        self.status=self.create_publisher(String,'/field/gps/candidate_status',10)
        self.create_subscription(PoseWithCovarianceStamped,'/field/relocalization/raw_pose',self.candidate,10)
        for typ,topic,name in [(NavSatFix,'/ins/navsatfix','fix'),(INSStatus,'/ins/status','status'),(Imu,'/ins/imu','imu')]:
            self.create_subscription(typ,topic,lambda msg,k=name:self.messages.update({k:msg}),qos_profile_sensor_data)
    def localization(self,msg):
        self.localized_at=time.monotonic() if msg.state==msg.STATE_LOCALIZED and msg.local_odom_fresh and msg.global_correction_valid else 0.
    def candidate(self,msg):
        m=self.messages;pose=msg.pose.pose;q=pose.orientation
        heading=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        reference=None
        if time.monotonic()-self.localized_at<.5:
            try:
                tf=self.buffer.lookup_transform('map','base_link',Time()).transform;q=tf.rotation
                reference=dict(x=tf.translation.x,y=tf.translation.y,yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
            except Exception:pass
        result=dict(usable=False,reasons=['GPS_NOT_OBSERVED'],weight=0.)
        if all(k in m for k in ('fix','status','imu')):
            stamp=lambda x:x.header.stamp.sec+x.header.stamp.nanosec/1e9
            s={k:getattr(m['status'],k) for k in ('rtk_fixed','position_status_valid','position_std_valid','east_std','north_std','up_std')}
            s.update(fix_stamp=stamp(m['fix']),status_stamp=stamp(m['status']),imu_stamp=stamp(m['imu']))
            f=m['fix'];result=prior(dict(latitude=f.latitude,longitude=f.longitude,altitude=f.altitude,status=f.status.status),s,self.alignment,self.get_clock().now().nanoseconds/1e9,reference or dict(x=pose.position.x,y=pose.position.y,yaw=heading),gate_innovation=reference is not None)
        # Invalid/degraded GPS removes its constraint. A quality-valid GPS jump
        # relative to this candidate rejects only this candidate, never moves TF.
        reject=bool(result.get('usable') and math.hypot(result['map_prior']['x']-pose.position.x,result['map_prior']['y']-pose.position.y)>self.alignment.get('max_candidate_distance_m',3.))
        if reject:result['reasons'].append('candidate_outside_verified_GPS_prior')
        result.update(candidate_accepted=not reject,scope='GLOBAL_CANDIDATE_PRIOR',tf_publisher=False)
        self.status.publish(String(data=json.dumps(result)))
        if not reject:self.pub.publish(msg)
def main():
    rclpy.init();node=Gate()
    try:rclpy.spin(node)
    finally:node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
