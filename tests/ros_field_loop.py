#!/usr/bin/env python3
"""Actual Humble DDS and native PCL tests using synthetic inputs, no CAN devices."""
import json,os,signal,subprocess,tempfile,time,unittest,struct,hashlib,shutil
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy._rclpy_pybind11 import NodeNameNonExistentError
from sensor_msgs.msg import PointCloud2,PointField,Imu
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from nav_msgs.msg import Odometry
from std_msgs.msg import Header,Bool
from geometry_msgs.msg import Twist
from bunker_msgs.msg import BunkerStatus
from std_srvs.srv import Trigger
from livox_ros_driver2.msg import CustomMsg,CustomPoint
import yaml
from tf2_ros import Buffer,TransformListener
from rclpy.time import Time
REPO=Path(__file__).resolve().parents[1]
class HumbleLoop(unittest.TestCase):
    @classmethod
    def setUpClass(cls):rclpy.init();cls.node=Node('synthetic_field_loop_test')
    @classmethod
    def tearDownClass(cls):cls.node.destroy_node();rclpy.shutdown()
    def wait(self,condition,timeout=8):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            rclpy.spin_once(self.node,timeout_sec=.02)
            try:
                if condition():return
            except NodeNameNonExistentError:pass
        self.fail('DDS condition timed out')
    def test_no_pgo_export_and_full_resolution_clean(self):
        with tempfile.TemporaryDirectory() as td:
            out=Path(td)/'artifact'
            proc=subprocess.Popen([str(REPO.parent.parent/'install/agt_field_test_bringup/lib/agt_field_test_bringup/lio_artifact'),'--ros-args','-p','output_dir:='+str(out)],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,start_new_session=True)
            try:
                cloud=self.node.create_publisher(PointCloud2,'/fastlio2/world_cloud',10);odom=self.node.create_publisher(Odometry,'/fastlio2/lio_odom',10)
                cli=self.node.create_client(Trigger,'/mapping/fastlio/export')
                self.wait(lambda:cli.service_is_ready() and cloud.get_subscription_count()>0)
                h=Header(frame_id='lidar');h.stamp=self.node.get_clock().now().to_msg()
                # Tail of the full cloud lies beyond the UI preview limit (12000).
                points=[[float(i)*.001,0.,.3] for i in range(20000)]
                cloud.publish(create_cloud_xyz32(h,points));o=Odometry();o.header=h;o.pose.pose.orientation.w=1.;odom.publish(o)
                self.wait(lambda:(out/'points.raw').is_file() and (out/'points.raw').stat().st_size>=19000*12)
                time.sleep(.2);future=cli.call_async(Trigger.Request());self.wait(future.done);self.assertTrue(future.result().success,future.result().message)
                source=out/'map_package/map.pcd';before=hashlib.sha256(source.read_bytes()).hexdigest()
                spec=Path(td)/'ops.yaml';spec.write_text(yaml.safe_dump({'operations':[{'type':'delete','min':[15.,-1.,0.],'max':[21.,1.,1.]}], 'projection':{'resolution':.1,'origin':[0.,-1.,0.],'range':[21.,2.],'ground_min':-.1,'z_min':.1,'z_max':1.,'min_points':1,'occupied_thresh':.65,'free_thresh':.196}}))
                target=Path(td)/'clean';subprocess.run(['ros2','run','agt_field_test_bringup','pcd_ops',str(source),str(spec),str(target)],check=True)
                result=yaml.safe_load((target/'result.yaml').read_text());self.assertEqual(result['input_points'],20000);self.assertLess(result['output_points'],16000)
                self.assertTrue(result['processed_full_resolution']);self.assertTrue((target/'map.pgm').is_file());self.assertEqual(hashlib.sha256(source.read_bytes()).hexdigest(),before)
                again=cli.call_async(Trigger.Request());self.wait(again.done);self.assertFalse(again.result().success)
                self.assertEqual(len(self.node.get_publishers_info_by_topic('/ctrl_cmd')),0)
            finally:os.killpg(proc.pid,signal.SIGINT);proc.wait(timeout=8)
    def test_bunker_guard_feedback_deadman_and_no_replay(self):
        exe=str(REPO.parent.parent/'install/agt_base_control/lib/agt_base_control/cmd_vel_guard')
        proc=subprocess.Popen([exe,'--ros-args','-p','require_chassis_feedback:=true','-p','require_localization_status:=false','-p','require_deadman:=true','-p','default_control_mode:=manual','-p','output_topic:=/test/guard'],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,start_new_session=True)
        observed=[]
        sub=self.node.create_subscription(Twist,'/test/guard',lambda m:observed.append(m.linear.x),10)
        feedback=self.node.create_publisher(BunkerStatus,'/bunker_status',10);deadman=self.node.create_publisher(Bool,'/agt/hmi/deadman',1);cmd=self.node.create_publisher(Twist,'/agt/hmi/cmd_vel',10)
        status=BunkerStatus();status.vehicle_state=0;status.control_mode=1;status.error_code=0
        def pump(duration,held=True,send=False,fb=True):
            end=time.monotonic()+duration
            while time.monotonic()<end:
                if fb:feedback.publish(status)
                deadman.publish(Bool(data=held))
                if send:
                    t=Twist();t.linear.x=.1;cmd.publish(t)
                rclpy.spin_once(self.node,timeout_sec=.02)
        try:
            self.wait(lambda:len(observed)>0);self.assertTrue(all(v==0 for v in observed))
            pump(.4,send=True);self.assertGreater(max(observed),0.)
            pump(.1,held=False);self.assertEqual(observed[-1],0.)
            pump(.15);self.assertEqual(observed[-1],0.)
            pump(.2,send=True);self.assertGreater(observed[-1],0.)
            pump(.45,fb=False);self.assertEqual(observed[-1],0.)
            pump(.15);self.assertEqual(observed[-1],0.)
            self.assertEqual(len(self.node.get_publishers_info_by_topic('/ctrl_cmd')),0)
        finally:
            os.killpg(proc.pid,signal.SIGINT);proc.wait(timeout=8);self.node.destroy_subscription(sub)
    def test_no_pgo_launch_graph_has_aligned_inputs_and_no_chassis(self):
        with tempfile.TemporaryDirectory() as td:
            config=REPO.parent/'agt-lio-pgo-mapping/bringup/agt_mapping_bringup/config/fastlio2_mid360.yaml'
            data=yaml.safe_load(config.read_text());data['esti_il']=False
            frozen=Path(td)/'fastlio.yaml';frozen.write_text(yaml.safe_dump(data))
            log=(Path(td)/'launch.log').open('w')
            proc=subprocess.Popen(['ros2','launch','agt_field_test_bringup','no_pgo.launch.py','lio_config:='+str(frozen),'output:='+str(Path(td)/'artifact'),'use_sim_time:=false'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            try:
                self.wait(lambda:any(n=='fastlio_mapping' for n,ns in self.node.get_node_names_and_namespaces()),15)
                self.wait(lambda:all(topic in [n for n,t in self.node.get_subscriber_names_and_types_by_node('fastlio_mapping','/fastlio2')] for topic in ('/field/aligned/imu','/mapping/sensor/livox')),10)
                subscriptions=self.node.get_subscriber_names_and_types_by_node('fastlio_mapping','/fastlio2')
                names=[n for n,types in subscriptions]
                self.assertIn('/field/aligned/imu',names);self.assertIn('/mapping/sensor/livox',names)
                self.assertFalse(any('pgo' in n.lower() for n,ns in self.node.get_node_names_and_namespaces()))
                self.assertEqual(self.node.count_publishers('/ctrl_cmd'),0)
                self.assertIsNone(proc.poll())
            finally:
                os.killpg(proc.pid,signal.SIGINT)
                try:proc.wait(timeout=8)
                except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
                log.close();shutil.copyfile(Path(td)/'launch.log',REPO/'docs/evidence/field_loop_20261023/no_pgo_graph.log')
    def test_bunker_bundle_reaches_robot_state_publisher(self):
        bundle=REPO/'agt_field_test_bringup/config/bunker_calibration_candidate.yaml'
        with tempfile.TemporaryDirectory() as td:
            log=(Path(td)/'rsp.log').open('w')
            proc=subprocess.Popen(['ros2','launch','agt_field_test_bringup','field_test.launch.py','robot:=bunker','enable_chassis:=false','enable_mid360:=false','calibration_bundle:='+str(bundle)],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            buffer=Buffer();listener=TransformListener(buffer,self.node)
            try:
                def ready():
                    try:return buffer.can_transform('base_link','livox_frame',Time())
                    except Exception:return False
                self.wait(ready,12)
                t=buffer.lookup_transform('base_link','livox_frame',Time()).transform
                self.assertAlmostEqual(t.translation.x,.406074739840648,places=7)
                self.assertAlmostEqual(t.translation.z,.727468117997015,places=7)
                self.assertEqual(self.node.count_publishers('/ctrl_cmd'),0)
                self.assertIsNone(proc.poll())
            finally:
                os.killpg(proc.pid,signal.SIGINT)
                try:proc.wait(timeout=8)
                except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
                listener.unregister();log.close()
    def test_bunker_localization_query_and_lio_use_aligned_sensor(self):
        with tempfile.TemporaryDirectory() as td:
            map_path=Path(td)/'map.pcd'
            map_path.write_text('VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH 3\nHEIGHT 1\nPOINTS 3\nDATA ascii\n0 0 0\n1 0 1\n0 1 2\n')
            bundle=REPO/'agt_field_test_bringup/config/bunker_calibration_candidate.yaml'
            log=(Path(td)/'localization.log').open('w')
            proc=subprocess.Popen(['ros2','launch','agt_field_test_bringup','field_test.launch.py','robot:=bunker','enable_chassis:=false','enable_mid360:=false','enable_localization:=true','lio_backend:=fastlio2','global_map:='+str(map_path),'calibration_bundle:='+str(bundle)],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            try:
                self.wait(lambda:any(n=='livox_format_bridge' for n,ns in self.node.get_node_names_and_namespaces()),15)
                def inputs(name,namespace):return [n for n,t in self.node.get_subscriber_names_and_types_by_node(name,namespace)]
                self.wait(lambda:'/field/aligned/lidar' in inputs('livox_format_bridge','/'),10)
                self.wait(lambda:'/field/aligned/imu' in inputs('lio_node','/fastlio2'),10)
                self.assertEqual(self.node.count_publishers('/ctrl_cmd'),0)
                self.assertIsNone(proc.poll())
            finally:
                os.killpg(proc.pid,signal.SIGINT)
                try:proc.wait(timeout=10)
                except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
                log.close();shutil.copyfile(Path(td)/'localization.log',REPO/'docs/evidence/field_loop_20261023/bunker_localization_graph.log')
    def test_raw_bag_recorder_subscribes_before_processing(self):
        with tempfile.TemporaryDirectory() as td:
            imu=self.node.create_publisher(Imu,'/livox/imu',10);lidar=self.node.create_publisher(CustomMsg,'/livox/lidar',10)
            log=(Path(td)/'recorder.log').open('w')
            proc=subprocess.Popen(['ros2','bag','record','--node-name','field_raw_test','-o',str(Path(td)/'raw'),'/livox/lidar','/livox/imu'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            try:
                self.wait(lambda:any(n=='field_raw_test' for n,ns in self.node.get_node_names_and_namespaces()),10)
                info=subprocess.check_output(['ros2','node','info','/field_raw_test'],text=True,timeout=5)
                self.assertIn('/livox/lidar:',info);self.assertIn('/livox/imu:',info)
                for _ in range(15):
                    i=Imu();i.header.stamp=self.node.get_clock().now().to_msg();l=CustomMsg();l.header=i.header;l.timebase=i.header.stamp.sec*10**9+i.header.stamp.nanosec
                    imu.publish(i);lidar.publish(l);rclpy.spin_once(self.node,timeout_sec=.02)
            finally:
                os.killpg(proc.pid,signal.SIGINT);proc.wait(timeout=10);log.close()
            metadata=yaml.safe_load((Path(td)/'raw/metadata.yaml').read_text())['rosbag2_bagfile_information']
            self.assertGreater(metadata['message_count'],10)
            self.assertEqual({t['topic_metadata']['name'] for t in metadata['topics_with_message_count']},{'/livox/lidar','/livox/imu'})
    def test_timestamp_policy_reaches_dds(self):
        proc=subprocess.Popen([str(REPO.parent.parent/'install/agt_field_test_bringup/lib/agt_field_test_bringup/time_align.py'),'--ros-args','-p','imu_offset_sec:=0.125','-p','lidar_offset_sec:=-0.025'],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,start_new_session=True)
        seen={}
        imu=self.node.create_publisher(Imu,'/livox/imu',10);lidar=self.node.create_publisher(CustomMsg,'/livox/lidar',10)
        si=self.node.create_subscription(Imu,'/field/aligned/imu',lambda m:seen.update(imu=m),10)
        sl=self.node.create_subscription(CustomMsg,'/field/aligned/lidar',lambda m:seen.update(lidar=m),10)
        try:
            self.wait(lambda:imu.get_subscription_count()>0 and lidar.get_subscription_count()>0)
            i=Imu();i.header.stamp.sec=100
            l=CustomMsg();l.header.stamp.sec=100;l.timebase=100000000000;l.point_num=1;p=CustomPoint();p.offset_time=1234;l.points=[p]
            end=time.monotonic()+3.
            while len(seen)<2 and time.monotonic()<end:
                imu.publish(i);lidar.publish(l);rclpy.spin_once(self.node,timeout_sec=.05)
            self.assertEqual(len(seen),2)
            self.assertEqual(seen['imu'].header.stamp.nanosec,125000000)
            self.assertEqual(seen['lidar'].timebase,99975000000)
            self.assertEqual(seen['lidar'].header.stamp.sec,99)
            self.assertEqual(seen['lidar'].header.stamp.nanosec,975000000)
            self.assertEqual(seen['lidar'].points[0].offset_time,1234)
            self.assertEqual(l.timebase,100000000000)
        finally:os.killpg(proc.pid,signal.SIGINT);proc.wait(timeout=8);self.node.destroy_subscription(si);self.node.destroy_subscription(sl)
if __name__=='__main__':unittest.main(verbosity=2)
