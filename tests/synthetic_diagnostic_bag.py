#!/usr/bin/env python3
"""Reproducible synthetic telemetry, never a field bag/accuracy validation."""
import copy,json,math,subprocess,sys,tempfile
from pathlib import Path
import rosbag2_py
from rclpy.serialization import serialize_message
from nav_msgs.msg import Odometry,Path as NavPath
from geometry_msgs.msg import Twist,PoseStamped,TransformStamped
from tf2_msgs.msg import TFMessage
R=Path(__file__).resolve().parents[1]
with tempfile.TemporaryDirectory() as td:
    bag=Path(td)/'bag';w=rosbag2_py.SequentialWriter();w.open(rosbag2_py.StorageOptions(uri=str(bag),storage_id='sqlite3'),rosbag2_py.ConverterOptions('',''))
    topics={'/mux/cmd_vel':'geometry_msgs/msg/Twist','/agt/odometry/local':'nav_msgs/msg/Odometry','/wheel/odom':'nav_msgs/msg/Odometry','/field/map_pose':'geometry_msgs/msg/PoseStamped','/plan':'nav_msgs/msg/Path','/tf':'tf2_msgs/msg/TFMessage'}
    for name,typ in topics.items():w.create_topic(rosbag2_py.TopicMetadata(name=name,type=typ,serialization_format='cdr'))
    for i in range(500):
        t=i*.02;ns=int((100+t)*1e9);yaw=.1*math.sin(2*math.pi*.6*t);rate=.1*2*math.pi*.6*math.cos(2*math.pi*.6*t)
        cmd=Twist();cmd.linear.x=.05;cmd.angular.z=rate
        od=Odometry();od.header.stamp.sec,od.header.stamp.nanosec=divmod(ns,10**9);od.header.frame_id='odom';od.child_frame_id='base_link';od.pose.pose.position.x=.05*t;od.pose.pose.orientation.z=math.sin(yaw/2);od.pose.pose.orientation.w=math.cos(yaw/2)
        wheel=Odometry();wheel.header=copy.deepcopy(od.header);wheel.pose=copy.deepcopy(od.pose)
        pose=PoseStamped();pose.header=copy.deepcopy(od.header);pose.header.frame_id='map';pose.pose=copy.deepcopy(od.pose.pose);pose.pose.position.y=.04*math.sin(t)
        path=NavPath();path.header=copy.deepcopy(pose.header);p1=PoseStamped();p2=PoseStamped();p2.pose.position.x=10.;path.poses=[p1,p2]
        transform=TransformStamped();transform.header=copy.deepcopy(pose.header);transform.child_frame_id='odom';transform.transform.rotation.w=1.;transform.transform.translation.x=.1 if t>=5 else 0.
        for topic,msg in [('/mux/cmd_vel',cmd),('/agt/odometry/local',od),('/wheel/odom',wheel),('/field/map_pose',pose),('/plan',path),('/tf',TFMessage(transforms=[transform]))]:w.write(topic,serialize_message(msg),ns)
    del w
    output=Path(td)/'report';subprocess.run([sys.executable,str(R/'scripts/analyze_bag.py'),'--bag',str(bag),'--output',str(output)],check=True)
    result=json.loads((output/'diagnostic.json').read_text());m=result['metrics']
    assert abs(m['oscillation_hz']-.6)<.12,m
    assert abs(m['max_lateral_error_m']-.04)<.005,m
    assert abs(m['max_map_odom_translation_step_m']-.1)<1e-6,m
    result['scope']='SYNTHETIC_ROSBAG_READER_TEST';result['real_field_bag']='NOT_RUN';result.pop('series',None)
    (R/'docs/evidence/field_loop_20261023/synthetic_diagnostics.json').write_text(json.dumps(result,indent=2))
    print(json.dumps(m))
