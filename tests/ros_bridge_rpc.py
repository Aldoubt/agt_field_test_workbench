#!/usr/bin/env python3
"""Actual DDS + Unix RPC supervised jog lease, synthetic Bunker feedback only."""
import importlib.util,json,os,sys,tempfile,threading,time,unittest
from pathlib import Path
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import Twist,TransformStamped
from tf2_ros import StaticTransformBroadcaster
from rclpy.action import ActionServer
from nav2_msgs.action import NavigateToPose,FollowWaypoints
from lifecycle_msgs.srv import GetState
from lifecycle_msgs.msg import State
from bunker_msgs.msg import BunkerStatus
REPO=Path(__file__).resolve().parents[1];sys.path.insert(0,str(REPO))
from workbench.session import rpc
class RPCIntegration(unittest.TestCase):
 def test_expiring_jog_and_closed_lease_cannot_replay(self):
    spec=importlib.util.spec_from_file_location('field_bridge',REPO/'scripts/ros_bridge.py');module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    with tempfile.TemporaryDirectory() as td:
        module.ROOT=Path(td);root=module.ROOT;(root/'work').mkdir();(root/'runs/fixture').mkdir(parents=True)
        (root/'work/current_session.json').write_text(json.dumps(dict(id='fixture',robot='bunker',task='manual',checks={'synthetic':True},profile=dict(forward_mps=.15,reverse_mps=.08,lateral_mps=0.,angular_radps=.3))))
        rclpy.init();bridge=module.Bridge();observer=Node('synthetic_bridge_feedback');seen=[]
        observer.create_subscription(Twist,'/agt/hmi/cmd_vel',lambda msg:seen.append(msg.linear.x),10)
        pub=observer.create_publisher(BunkerStatus,'/bunker_status',10)
        status=BunkerStatus();status.vehicle_state=0;status.control_mode=1
        timer=observer.create_timer(.02,lambda:pub.publish(status))
        executor=SingleThreadedExecutor();executor.add_node(bridge);executor.add_node(observer)
        spinner=threading.Thread(target=executor.spin,daemon=True);spinner.start()
        server=module.socketserver.ThreadingUnixStreamServer(str(root/'work/ros_bridge.sock'),module.Handler);server.node=bridge
        threading.Thread(target=server.serve_forever,daemon=True).start()
        def wait(pred):
            end=time.monotonic()+5
            while not pred() and time.monotonic()<end:time.sleep(.02)
            self.assertTrue(pred())
        try:
            wait(lambda:bridge.physical()[0]);time.sleep(.2)
            p=dict(robot='bunker',lease='synthetic-lease',sequence=1,vx=.03)
            self.assertEqual(rpc(root,'jog',p)['state'],'LEASE_REFRESHED');wait(lambda:any(v>0 for v in seen))
            rpc(root,'stop',dict(lease=p['lease']));time.sleep(.08);self.assertEqual(seen[-1],0.)
            with self.assertRaises(ValueError):rpc(root,'jog',dict(p,sequence=2))
            self.assertEqual(rpc(root,'jog',dict(p,lease='fresh-lease'))['state'],'LEASE_REFRESHED')
            time.sleep(.3);self.assertEqual(seen[-1],0.)
            with self.assertRaises(ValueError):rpc(root,'jog',dict(p,lease='fresh-lease',sequence=2))
            with self.assertRaises(ValueError):rpc(root,'distance',dict(robot='bunker',map_id=None,distance_m=1))
            self.assertEqual(observer.count_publishers('/ctrl_cmd'),0)
            # Exercise real Humble action/lifecycle wire types with an explicit
            # mock physical gate. This is not Nav2 planning or field acceptance.
            import math
            received=[]
            def execute(handle):
                received.append(handle.request.pose);handle.succeed();return NavigateToPose.Result()
            action=ActionServer(observer,NavigateToPose,'/navigate_to_pose',execute)
            def active(request,response):response.current_state=State(id=3,label='active');return response
            services=[observer.create_service(GetState,'/'+name+'/get_state',active) for name in ('controller_server','bt_navigator')]
            tf=StaticTransformBroadcaster(observer);t=TransformStamped();t.header.frame_id='map';t.child_frame_id='base_link';t.header.stamp=observer.get_clock().now().to_msg();t.transform.translation.x=2.;t.transform.translation.y=3.;t.transform.rotation.z=math.sin(math.pi/4);t.transform.rotation.w=math.cos(math.pi/4);tf.sendTransform(t)
            wait(lambda:bridge.navigate.server_is_ready() and all(bridge.states.get(n)==3 for n in bridge.state_clients))
            wait(lambda:bridge.buffer.can_transform('map','base_link',rclpy.time.Time()))
            fixture=dict(id='fixture',robot='bunker',task='navigation',map_id='synthetic-map')
            (root/'work/current_session.json').write_text(json.dumps(fixture));bridge.check_navigation=lambda:fixture
            result=rpc(root,'distance',dict(robot='bunker',map_id='synthetic-map',distance_m=1.))
            self.assertEqual(result['state'],'REQUESTED');wait(lambda:len(received)==1)
            self.assertAlmostEqual(received[0].pose.position.x,2.);self.assertAlmostEqual(received[0].pose.position.y,4.)
            self.assertEqual(received[0].header.frame_id,'map')
            wait(lambda:not bridge.goals and not bridge.pending_goal)
            from workbench.maps import Maps
            from workbench.routes import Routes
            m=Maps(root)._save(20,20,bytes([254])*400,dict(resolution=1.,origin=[0.,0.,0.],occupied_thresh=.65,free_thresh=.196,negate=0),dict(parent=None,test_fixture=True))
            route=Routes(root).save(m['id'],[dict(x=2.,y=4.,yaw=0.),dict(x=3.,y=5.,yaw=0.)])
            fixture['map_id']=m['id'];(root/'work/current_session.json').write_text(json.dumps(fixture))
            waypoints=[]
            def execute_route(handle):
                waypoints.extend(handle.request.poses);handle.succeed();return FollowWaypoints.Result()
            route_action=ActionServer(observer,FollowWaypoints,'/follow_waypoints',execute_route)
            wait(bridge.waypoints.server_is_ready)
            rpc(root,'route',dict(robot='bunker',map_id=m['id'],route_id=route['id']))
            wait(lambda:len(waypoints)==2)
            self.assertEqual([p.pose.position.x for p in waypoints],[2.,3.])
            wait(lambda:not bridge.goals and not bridge.pending_goal)
            route_action.destroy();action.destroy()
            for service in services:observer.destroy_service(service)
        finally:
            server.shutdown();server.server_close();executor.shutdown();spinner.join(timeout=3);bridge.destroy_node();observer.destroy_node();rclpy.shutdown()
if __name__=='__main__':unittest.main(verbosity=2)
