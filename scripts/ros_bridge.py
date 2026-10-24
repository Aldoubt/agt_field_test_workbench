#!/usr/bin/env python3
"""Local Unix socket to a fixed list of ROS operations; never executes commands."""
import json
import math
import os
from pathlib import Path
import queue
import signal
import socketserver
import sys
import threading
import time

REPO=Path(__file__).resolve().parents[1];ROOT=REPO.parent.parent
sys.path.insert(0,str(REPO))
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist,PoseStamped
from std_msgs.msg import String,Bool
from std_srvs.srv import Trigger,SetBool
from nav_msgs.msg import Odometry,Path as NavPath,OccupancyGrid
from nav2_msgs.action import NavigateToPose,FollowWaypoints
from lifecycle_msgs.srv import GetState
from rclpy.action import ActionClient
from tf2_ros import Buffer,TransformListener
from agt_robot_interfaces.msg import LocalizationStatus,MissionStatus
from agt_robot_interfaces.action import ExecuteInspectionMission
from bunker_msgs.msg import BunkerStatus
from agt_asensing_driver.msg import INSStatus
from sensor_msgs.msg import NavSatFix,Imu
from workbench.routes import Routes,relative,pose
from workbench.maps import Maps,hash_file
from workbench.goal_safety import validate_endpoints
from workbench.gps import prior,load_alignment
from workbench.runtime import write_json

class Bridge(Node):
    def __init__(self):
        super().__init__('field_workbench_bridge');self.inbox=queue.Queue();self.session=None;self.last_session_mtime=None
        self.samples={};self.times={};self.file_identities={};self.goals=[];self.action_events=[];self.goal_result=None;self.goal_epoch=0;self.pending_goal=False
        self.jog_until=0.;self.jog_target=Twist();self.enable_future=None;self.closed_leases=set();self.lease=None;self.lease_seq=-1
        self.buffer=Buffer();self.listener=TransformListener(self.buffer,self)
        self.map_observation=self.create_publisher(PoseStamped,'/field/map_pose',10)
        self.cmd=self.create_publisher(Twist,'/agt/hmi/cmd_vel',1)
        self.mode=self.create_publisher(String,'/agt/control/mode',1)
        self.deadman=self.create_publisher(Bool,'/agt/hmi/deadman',1)
        self.navigate=ActionClient(self,NavigateToPose,'/navigate_to_pose')
        self.waypoints=ActionClient(self,FollowWaypoints,'/follow_waypoints')
        self.mission=ActionClient(self,ExecuteInspectionMission,'/agt/mission/execute')
        self.mission_status={}
        self.states={};self.state_clients={n:self.create_client(GetState,'/'+n+'/get_state') for n in ('controller_server','bt_navigator')}
        def sub(typ,topic,name,qos=10):
            self.create_subscription(typ,topic,lambda m:self.observe(name,m),qos)
        sub(LocalizationStatus,'/agt/localization/status','localization')
        sub(MissionStatus,'/agt/mission/status','mission')
        sub(Odometry,'/agt/odometry/local','lio');sub(Odometry,'/wheel/odom','wheel')
        sub(BunkerStatus,'/bunker_status','bunker');sub(String,'/fwmini/control_status','fwmini')
        sub(NavPath,'/plan','plan');sub(OccupancyGrid,'/local_costmap/costmap','costmap')
        sub(INSStatus,'/ins/status','ins');sub(NavSatFix,'/ins/navsatfix','fix');sub(Imu,'/ins/imu','ins_imu',qos_profile_sensor_data)
        self.create_timer(.02,self.tick);self.create_timer(1.,self.poll_lifecycle)
    def observe(self,name,msg):self.samples[name]=msg;self.times[name]=time.monotonic()
    def current(self):
        path=ROOT/'work/current_session.json'
        if path.is_file() and path.stat().st_mtime_ns!=self.last_session_mtime:
            self.session=json.loads(path.read_text());self.last_session_mtime=path.stat().st_mtime_ns
        if not path.is_file():self.session=None;self.last_session_mtime=None
        if self.session and self.session.get('job_id'):
            job=ROOT/'runs'/self.session['job_id']/'run.json'
            try:
                record=json.loads(job.read_text())
                if record.get('state')!='RUNNING':return None
                os.kill(record['pid'],0)
            except (OSError,KeyError):return None
        return self.session
    def fresh(self,name,limit=.3):return name in self.times and time.monotonic()-self.times[name]<limit
    def physical(self):
        s=self.current()
        if not s:return False,'no_session'
        if s['robot']=='bunker':
            b=self.samples.get('bunker')
            return (bool(self.fresh('bunker') and b.vehicle_state==0 and b.control_mode==1 and b.error_code==0),'bunker_can_authority_or_fault')
        f=json.loads(self.samples['fwmini'].data) if self.fresh('fwmini') else {}
        return (bool(f.get('state') in ('READY','ACTIVE') and 0<=f.get('feedback_age_sec',-1)<.2 and 0<=f.get('io_feedback_age_sec',-1)<.2),'fwmini_not_READY_or_stale')
    def chassis_status(self):
        """Read-only communication health; independent from motion permission."""
        s=self.current()
        if not s:return dict(enabled=False,online=False,state='NO_SESSION')
        name=s['robot'];online=self.fresh(name,.5)
        result=dict(enabled=s['task'] in ('manual','navigation'),online=online,state='NO_FEEDBACK')
        if not online:return result
        if name=='bunker':
            b=self.samples['bunker']
            result.update(state=int(b.vehicle_state),control_mode=int(b.control_mode),error_code=int(b.error_code))
        else:
            try:f=json.loads(self.samples['fwmini'].data)
            except (ValueError,TypeError):return dict(result,online=False,state='INVALID_FEEDBACK')
            result.update(state=str(f.get('state','UNKNOWN')),feedback_age_sec=f.get('feedback_age_sec'),
                          io_feedback_age_sec=f.get('io_feedback_age_sec'))
        return result
    def localized(self):
        m=self.samples.get('localization')
        return bool(self.fresh('localization',.5) and m.state==m.STATE_LOCALIZED and m.local_odom_fresh and m.global_correction_valid and self.fresh('lio'))
    def map_pose(self):
        t=self.buffer.lookup_transform('map','base_link',Time()).transform;q=t.rotation
        return dict(x=t.translation.x,y=t.translation.y,yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
    def verified_file(self,path,sha):
        p=Path(path);st=p.stat();signature=(st.st_ino,st.st_size,st.st_mtime_ns,st.st_ctime_ns)
        if self.file_identities.get((str(p),sha))!=signature:
            if hash_file(p)!=sha:raise ValueError('active artifact changed: '+str(p))
            self.file_identities[(str(p),sha)]=signature
    def check_navigation(self):
        s=self.current()
        if not s or s['task']!='navigation' or not s.get('safety_report'):raise ValueError('start an accepted navigation session first')
        m=Maps(ROOT).get(s['map_id'])
        from workbench.safety import report as verify_report
        verify_report(s['safety_report']['path'],s['robot'],s['profile']['profile_sha256'],s['map_sha256'],s['calibration_sha256'])
        self.verified_file(s['profile_snapshot'],s['profile']['profile_sha256'])
        self.verified_file(s['calibration'],s['calibration_snapshot_sha256'])
        self.verified_file(m['yaml'],s['map_yaml_sha256'])
        self.verified_file(s['localization_pcd'],s['localization_pcd_sha256'])
        self.verified_file(m['pgm'],s['map_sha256'])
        if m['status']!='ACCEPTED' or m['sha256']!=s['map_sha256'] or m.get('source_pcd')!=s['localization_pcd'] or m.get('source_pcd_sha256')!=s['localization_pcd_sha256']:raise ValueError('active map identity/review changed')
        if not self.localized():raise ValueError('fresh reliable localization required')
        ok,why=self.physical()
        if not ok:raise ValueError(why)
        if s['robot']=='fwmini' and json.loads(self.samples['fwmini'].data).get('target_gear')!=6:raise ValueError('Nav2 RPP requires 4T4D gear 6; lateral is supervised jog only')
        if any(self.states.get(n)!=3 for n in self.state_clients):raise ValueError('Nav2 controller and navigator must be ACTIVE')
        return s
    def poll_lifecycle(self):
        for name,client in self.state_clients.items():
            if client.service_is_ready():
                f=client.call_async(GetState.Request());f.add_done_callback(lambda future,n=name:self.states.update({n:future.result().current_state.id}) if future.result() else None)
    def call_service(self,name,typ,req):
        client=self.create_client(typ,name)
        if not client.service_is_ready():self.destroy_client(client);raise ValueError('service unavailable: '+name)
        future=client.call_async(req);future.add_done_callback(lambda _:self.destroy_client(client));return future
    def goal(self,points,route_id=None):
        s=self.check_navigation()
        validate_endpoints(Maps(ROOT).get(s['map_id']), points, s['profile'])
        if self.goals or self.pending_goal:raise ValueError('cancel the active goal before starting another')
        def stamped(p):
            msg=PoseStamped();msg.header.frame_id='map';msg.header.stamp=self.get_clock().now().to_msg()
            msg.pose.position.x=float(p['x']);msg.pose.position.y=float(p['y']);msg.pose.orientation.z=math.sin(p['yaw']/2);msg.pose.orientation.w=math.cos(p['yaw']/2);return msg
        if len(points)==1:client=self.navigate;goal=NavigateToPose.Goal(pose=stamped(points[0]))
        else:client=self.waypoints;goal=FollowWaypoints.Goal(poses=[stamped(p) for p in points])
        if not client.server_is_ready():raise ValueError('Nav2 action unavailable')
        self.mode.publish(String(data='navigation'))
        self.pending_goal=True;epoch=self.goal_epoch
        def send():
            if epoch!=self.goal_epoch:return
            try:self.check_navigation()
            except Exception as exc:self.stop();self.record('rejected',str(exc));return
            f=client.send_goal_async(goal,feedback_callback=lambda event:self.record('feedback',str(event.feedback)))
            f.add_done_callback(accepted)
        def accepted(future):
            handle=future.result()
            self.pending_goal=False
            if epoch!=self.goal_epoch:
                if handle.accepted:handle.cancel_goal_async()
                return
            if not handle.accepted:self.record('rejected','Nav2 rejected goal');return
            self.goals.append(handle)
            handle.get_result_async().add_done_callback(lambda result:self.completed(result.result(),handle))
        if s['robot']=='fwmini' and json.loads(self.samples['fwmini'].data)['state']=='READY':
            arm=self.call_service('/fwmini/enable_motion',SetBool,SetBool.Request(data=True))
            def armed(future):
                if epoch!=self.goal_epoch:return
                if future.result() and future.result().success:send()
                else:self.stop();self.record('rejected','FW-mini enable failed')
            arm.add_done_callback(armed)
        else:send()
        self.record('requested',dict(points=points,route_id=route_id,map_id=s['map_id']))
        return dict(state='REQUESTED',points=points)
    def start_mission(self,route):
        """Hand a map-bound photo route to V3's existing stop-and-shoot runtime."""
        s=self.check_navigation()
        validate_endpoints(Maps(ROOT).get(s['map_id']), route['points'], s['profile'])
        if any(p.get('photo') for p in route['points']) and not s.get('enable_camera_gimbal'):
            raise ValueError('photo route requires the camera/gimbal executor enabled in this session')
        if self.goals or self.pending_goal:raise ValueError('cancel the active goal before starting another')
        if not self.mission.server_is_ready():raise ValueError('inspection mission action unavailable')
        mission_file=Path(route.get('mission_file',''))
        if not mission_file.is_file() or mission_file.parent!=Routes(ROOT).directory/route['id']:
            raise ValueError('immutable route mission file is missing')
        goal=ExecuteInspectionMission.Goal(mission_file=str(mission_file),resume=False)
        self.mode.publish(String(data='navigation'))
        self.pending_goal=True;epoch=self.goal_epoch
        def send():
            if epoch!=self.goal_epoch:return
            try:self.check_navigation()
            except Exception as exc:self.stop();self.record('mission_rejected',str(exc));return
            future=self.mission.send_goal_async(goal)
            future.add_done_callback(accepted)
        def accepted(future):
            handle=future.result();self.pending_goal=False
            if epoch!=self.goal_epoch:
                if handle.accepted:handle.cancel_goal_async()
                return
            if not handle.accepted:self.record('mission_rejected','mission runtime rejected goal');return
            self.goals.append(handle)
            handle.get_result_async().add_done_callback(lambda result:self.completed(result.result(),handle))
        if s['robot']=='fwmini' and json.loads(self.samples['fwmini'].data)['state']=='READY':
            arm=self.call_service('/fwmini/enable_motion',SetBool,SetBool.Request(data=True))
            def armed(future):
                if epoch!=self.goal_epoch:return
                if future.result() and future.result().success:send()
                else:self.stop();self.record('mission_rejected','FW-mini enable failed')
            arm.add_done_callback(armed)
        else:send()
        self.record('mission_requested',dict(route_id=route['id'],mission_file=str(mission_file),map_id=s['map_id']))
        return dict(state='REQUESTED',route_id=route['id'],executor='agt_navigation_runtime/mission_runtime')
    def record(self,event,data):
        row=dict(time=time.time(),event=event,data=data);self.action_events.append(row)
        s=self.current()
        if s:
            with (ROOT/'runs'/s['id']/'actions.jsonl').open('a') as stream:stream.write(json.dumps(row)+'\n')
    def completed(self,result,handle):
        self.goal_result=dict(status=result.status,result=str(result.result));self.record('result',self.goal_result)
        if handle in self.goals:self.goals.remove(handle)
    def stop(self):
        self.goal_epoch+=1;self.pending_goal=False;self.enable_future=None
        if self.lease:self.closed_leases.add(self.lease)
        self.jog_until=0.;self.cmd.publish(Twist());self.deadman.publish(Bool(data=False))
        for g in self.goals:g.cancel_goal_async()
        self.goals=[]
        s=self.current()
        if s and s['robot']=='fwmini':
            try:self.call_service('/fwmini/enable_motion',SetBool,SetBool.Request(data=False))
            except ValueError:pass
        return dict(state='STOP_REQUESTED')
    def dispatch(self,operation,p):
        s=self.current()
        if operation=='ping':return dict(ready=True)
        if operation=='stop':
            if p.get('lease'):self.closed_leases.add(p['lease'])
            if self.lease:self.closed_leases.add(self.lease)
            return self.stop()
        if operation=='status':return self.snapshot()
        if operation not in ('ping','status','stop') and (not s or p.get('robot')!=s['robot']):raise ValueError('selected robot differs from active session')
        if operation in ('distance','route','goal') and p.get('map_id')!=s['map_id']:raise ValueError('selected map differs from active session')
        if operation=='distance':
            self.check_navigation();return self.goal([relative(self.map_pose(),p['distance_m'])])
        if operation=='route':
            self.check_navigation();r=Routes(ROOT).validate(p['route_id'],s['map_id'])
            if any(point.get('photo') for point in r['points']):return self.start_mission(r)
            return self.goal(r['points'],r['id'])
        if operation=='goal':return self.goal([pose(p['pose'])])
        if operation=='mode':
            if not s or s['robot']!='fwmini' or p.get('mode') not in ('four_wheel_steer','lateral'):raise ValueError('FW-mini mode must be four_wheel_steer or lateral')
            self.stop();self.call_service('/fwmini/select_'+p['mode'],Trigger,Trigger.Request());return dict(state='SWITCH_REQUESTED')
        if operation=='relocalize':
            self.call_service('/agt/localization/relocalize',Trigger,Trigger.Request());return dict(state='REQUESTED')
        if operation=='jog':
            lease=p.get('lease');seq=p.get('sequence',-1)
            if not isinstance(lease,str) or lease in self.closed_leases or not isinstance(seq,int):raise ValueError('closed/invalid deadman lease')
            if lease==self.lease and seq<=self.lease_seq:raise ValueError('stale jog sequence')
            if self.lease and lease!=self.lease and time.monotonic()<self.jog_until:raise ValueError('another jog gesture owns the lease')
            self.lease=lease;self.lease_seq=seq
            if not s or s['task']!='manual' or not s.get('checks'):raise ValueError('supervised manual session required')
            ok,why=self.physical()
            if not ok:raise ValueError(why)
            limits=s['profile'];v=Twist()
            values=[float(p.get(k,0.)) for k in ('vx','vy','wz')]
            if not all(map(math.isfinite,values)):raise ValueError('finite jog required')
            v.linear.x=max(-limits['reverse_mps'],min(limits['forward_mps'],values[0]));v.linear.y=max(-limits['lateral_mps'],min(limits['lateral_mps'],values[1]));v.angular.z=max(-limits['angular_radps'],min(limits['angular_radps'],values[2]))
            if s['robot']=='bunker' and values[1]!=0:raise ValueError('Bunker cannot move laterally')
            if s['robot']=='fwmini':
                f=json.loads(self.samples['fwmini'].data)
                if (f['target_gear']==6 and values[1]!=0) or (f['target_gear']==8 and values[2]!=0):raise ValueError('jog axis unsupported in selected gear')
                if f['state']=='READY' and self.enable_future is None:self.enable_future=self.call_service('/fwmini/enable_motion',SetBool,SetBool.Request(data=True))
            self.mode.publish(String(data='manual'));self.jog_target=v;self.jog_until=time.monotonic()+.18
            return dict(state='LEASE_REFRESHED',expires_sec=.18)
        raise ValueError('operation not whitelisted')
    def tick(self):
        try:
            p=self.map_pose();msg=PoseStamped();msg.header.frame_id='map';msg.header.stamp=self.get_clock().now().to_msg()
            msg.pose.position.x=p['x'];msg.pose.position.y=p['y'];msg.pose.orientation.z=math.sin(p['yaw']/2);msg.pose.orientation.w=math.cos(p['yaw']/2)
            self.map_observation.publish(msg)
        except Exception:pass
        for _ in range(min(self.inbox.qsize(),20)):
            op,p,reply,received=self.inbox.get()
            try:
                if op not in ('stop','status','ping') and time.monotonic()-received>(.18 if op=='jog' else 2.):raise ValueError('queued operation expired')
                reply.put(self.dispatch(op,p))
            except Exception as exc:reply.put({'error':str(exc)})
        if self.jog_until:
            ok,_=self.physical()
            if time.monotonic()>=self.jog_until or not ok:self.stop();self.enable_future=None
            else:
                self.deadman.publish(Bool(data=True));self.cmd.publish(self.jog_target)
        if self.goals:
            try:self.check_navigation()
            except Exception:self.stop();self.record('cancelled','runtime gate closed')
    def snapshot(self):
        result=dict(session=self.current(),localized=self.localized(),nav2_states=self.states,feedback_fresh=self.physical(),
                    chassis=self.chassis_status(),goal_result=self.goal_result)
        mission=self.samples.get('mission') if self.fresh('mission',5) else None
        result['mission']=dict(state=int(mission.state),mission_id=mission.mission_id,point_id=mission.point_id,
            point_index=int(mission.point_index),point_count=int(mission.point_count),detail=mission.detail,
            error_code=int(mission.error_code)) if mission else None
        try:result['pose']=self.map_pose()
        except Exception:result['pose']=None
        path=self.samples.get('plan');result['plan']=[dict(x=p.pose.position.x,y=p.pose.position.y) for p in path.poses] if path and self.fresh('plan',5) else []
        grid=self.samples.get('costmap')
        if grid and grid.info.width*grid.info.height<=1000000:
            result['costmap']=dict(width=grid.info.width,height=grid.info.height,resolution=grid.info.resolution,origin=[grid.info.origin.position.x,grid.info.origin.position.y],frame_id=grid.header.frame_id,data=list(grid.data))
            try:
                tf=self.buffer.lookup_transform('map',grid.header.frame_id,Time()).transform;q=tf.rotation
                yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));o=grid.info.origin.position
                oq=grid.info.origin.orientation;oyaw=math.atan2(2*(oq.w*oq.z+oq.x*oq.y),1-2*(oq.y*oq.y+oq.z*oq.z))
                result['costmap']['map_origin']=[tf.translation.x+math.cos(yaw)*o.x-math.sin(yaw)*o.y,
                    tf.translation.y+math.sin(yaw)*o.x+math.cos(yaw)*o.y,yaw+oyaw]
            except Exception:result['costmap']['transform_state']='UNAVAILABLE'
        result['fwmini']=json.loads(self.samples['fwmini'].data) if self.fresh('fwmini') else None
        result['gps']=self.gps(result['pose'])
        return result
    def gps(self,robot_pose):
        ins=self.samples.get('ins');fix=self.samples.get('fix')
        if not ins or not fix:return dict(usable=False,reasons=['hardware_not_observed'],state='NOT_RUN')
        stamp=lambda msg:msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        st={key:getattr(ins,key) for key in ('rtk_fixed','position_status_valid','position_std_valid','east_std','north_std','up_std','position_type')}
        st.update(fix_stamp=stamp(fix),status_stamp=stamp(ins),imu_stamp=stamp(self.samples['ins_imu']) if self.fresh('ins_imu') else 0.)
        path=ROOT/'config/localization/gps_alignment.json';align={}
        if path.is_file():
            try:
                session=self.current() or {};align=load_alignment(path,session.get('map_sha256'),session.get('calibration_sha256'))
            except Exception:align={}
        return dict(observed=st,**prior(dict(latitude=fix.latitude,longitude=fix.longitude,altitude=fix.altitude,status=fix.status.status),st,align,self.get_clock().now().nanoseconds/1e9,robot_pose))

class Handler(socketserver.StreamRequestHandler):
    def handle(self):
        try:
            request=json.loads(self.rfile.readline(1000000));reply=queue.Queue(1)
            self.server.node.inbox.put((request['operation'],request.get('payload',{}),reply,time.monotonic()));value=reply.get(timeout=6)
        except Exception as e:value={'error':str(e)}
        self.wfile.write((json.dumps(value)+'\n').encode())

if __name__=='__main__':
    import fcntl
    (ROOT/'work').mkdir(exist_ok=True)
    owner=(ROOT/'work/ros_bridge.lock').open('a')
    try:fcntl.flock(owner,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError:raise SystemExit('ROS bridge already owns this workspace')
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO);node=Bridge();running=[True]
    signal.signal(signal.SIGINT,lambda *_:running.__setitem__(0,False));signal.signal(signal.SIGTERM,lambda *_:running.__setitem__(0,False))
    path=ROOT/'work/ros_bridge.sock';path.parent.mkdir(exist_ok=True)
    if path.exists():path.unlink()
    server=socketserver.ThreadingUnixStreamServer(str(path),Handler);server.node=node;os.chmod(path,0o600)
    threading.Thread(target=server.serve_forever,daemon=True).start()
    try:
        while running[0] and rclpy.ok():rclpy.spin_once(node,timeout_sec=.02)
    finally:
        node.stop();server.shutdown();server.server_close();path.unlink(missing_ok=True);node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
