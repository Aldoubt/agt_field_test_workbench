"""Real DDS startup services in domain 183; no hardware processes or task goals."""
import math
import os
import threading
import time
from types import SimpleNamespace

import pytest

pytestmark=pytest.mark.skipif(
    os.environ.get('ROS_DOMAIN_ID')!='183' or os.environ.get('ROS_LOCALHOST_ONLY')!='1',
    reason='requires isolated domain 183 on localhost')


@pytest.mark.parametrize('mode,manual,replies',[
    ('nav',False,'normal'),('distance',False,'normal'),('photo',False,'normal'),('nav',True,'normal'),
    ('distance',False,'lost_first'),('distance',False,'lost_all')])
def test_composite_startup_uses_real_services_and_keeps_task_trigger_external(tmp_path,monkeypatch,mode,manual,replies):
    import rclpy
    from rclpy.context import Context
    from rclpy.node import Node
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.parameter import Parameter
    from rclpy.action import ActionClient
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import TransformStamped
    from tf2_ros import TransformBroadcaster
    from lifecycle_msgs.srv import GetState
    from nav2_msgs.srv import ManageLifecycleNodes
    from rcl_interfaces.srv import GetParameters,SetParametersAtomically
    from std_srvs.srv import Trigger
    from agt_robot_interfaces.msg import LocalizationStatus
    from camera_gimbal_interfaces.msg import CapabilityHealth
    from workbench import field_start
    ctx=Context();rclpy.init(context=ctx)
    peer=Node('isolated_field_mode_peers',context=ctx)
    states={name:1 for name in ['map_server','controller_server','bt_navigator']}
    events=[];localized=[False];ready=[False];map_started=[None]
    for name in states:
        def state(_request,response,key=name):
            response.current_state.id=states[key];return response
        peer.create_service(GetState,'/'+name+'/get_state',state)
    for manager,names in [('map',['map_server']),('navigation',['controller_server','bt_navigator'])]:
        def activate(request,response,group=names,key=manager):
            assert request.command==request.STARTUP
            if key=='navigation':assert localized[0]
            events.append('activate_'+key)
            if key=='map':map_started[0]=time.monotonic()
            for name in group:states[name]=3
            response.success=True;return response
        peer.create_service(ManageLifecycleNodes,'/lifecycle_manager_'+manager+'/manage_nodes',activate)
    def budget(request,response):
        assert request.parameters[0].value.double_value==30.
        events.append('budget');response.result.successful=True;return response
    peer.create_service(SetParametersAtomically,'/agt_global_relocalization/set_parameters_atomically',budget)
    def relocalize(_request,response):
        events.append('relocalize');localized[0]=True;response.success=True;return response
    peer.create_service(Trigger,'/agt/localization/relocalize',relocalize)
    def forbidden_trigger(*_):
        pytest.fail('startup invoked external task start/cancel')
    for endpoint in ['/agt/rviz_patrol/start','/agt/task/cancel']:
        peer.create_service(Trigger,endpoint,forbidden_trigger)
    params={'nav_behavior_tree':str(field_start.REPO/'config/photo_navigate.xml') if mode=='photo' else '',
            'recenter_after_capture':mode=='photo','goal_checker_plugins':['general_goal_checker','photo_goal_checker'],
            'photo_goal_checker.xy_goal_tolerance':.35,'photo_goal_checker.yaw_goal_tolerance':math.pi}
    def parameters(request,response):
        response.values=[Parameter(key,value=params[key]).get_parameter_value() for key in request.names]
        return response
    sends=[]
    for endpoint in ['/mission_runtime/get_parameters','/controller_server/get_parameters']:
        service=peer.create_service(GetParameters,endpoint,parameters)
        if endpoint=='/mission_runtime/get_parameters' and replies!='normal':
            original_send=service.send_response
            def send(response,header):
                sends.append(header)
                if replies=='lost_all' or len(sends)==1:return
                original_send(response,header)
            monkeypatch.setattr(service,'send_response',send)
    if replies!='normal':
        original_read=field_start.read_parameters
        monkeypatch.setattr(field_start,'read_parameters',
            lambda *a,**k:original_read(*a,response_timeout=.2,**k))
    odom=peer.create_publisher(Odometry,'/agt/odometry/local',10)
    status=peer.create_publisher(LocalizationStatus,'/agt/localization/status',10)
    camera=peer.create_publisher(CapabilityHealth,'/camera_gimbal/health',10)
    tf=TransformBroadcaster(peer)
    def publish():
        if manual and map_started[0] is not None and time.monotonic()-map_started[0] > 1.5:
            localized[0]=True
        now=peer.get_clock().now().to_msg()
        o=Odometry();o.header.stamp=now;o.header.frame_id='odom';o.child_frame_id='base_link';o.pose.pose.orientation.w=1.;odom.publish(o)
        s=LocalizationStatus();s.stamp=now;s.state=3 if localized[0] else 2
        s.local_odom_fresh=True;s.global_correction_valid=localized[0];s.map_id='fixture_map';status.publish(s)
        transforms=[]
        for parent,child in [('map','odom'),('odom','base_link')]:
            t=TransformStamped();t.header.stamp=now;t.header.frame_id=parent;t.child_frame_id=child;t.transform.rotation.w=1.;transforms.append(t)
        tf.sendTransform(transforms)
        h=CapabilityHealth();h.header.stamp=now;h.state=h.STATE_READY
        h.camera_alive=h.gimbal_serial_connected=h.gimbal_feedback_alive=h.move_action_ready=True;camera.publish(h)
    peer.create_timer(.02,publish)
    executor=MultiThreadedExecutor(num_threads=3,context=ctx);executor.add_node(peer)
    spin=threading.Thread(target=executor.spin,daemon=True);spin.start()
    class FakeJobs:
        def __init__(self,*_):self.jobs={}
        def _new(self,kind,command,meta):
            ident=str(len(self.jobs));record={'id':ident,'log':'synthetic-only.log'}
            self.jobs[ident]={'process':SimpleNamespace(poll=lambda:None),'record':record}
            events.append(('launch',kind,command));return record
        def stop(self,ident):events.append(('stop',ident))
    original_check=field_start.ModeSession.check
    def check(session):
        if ready[0]:raise KeyboardInterrupt
        return original_check(session)
    original_write=field_start.write_json
    def write(path,value):
        original_write(path,value)
        if value.get('state')=='READY':ready[0]=True
    monkeypatch.setattr(field_start,'ROOT',tmp_path)
    monkeypatch.setattr(field_start,'Jobs',FakeJobs)
    monkeypatch.setattr(field_start.ModeSession,'check',check)
    monkeypatch.setattr(field_start,'write_json',write)
    monkeypatch.setattr(field_start.field_trial,'make_plan',lambda *a,**k:dict(map_id='fixture_map',global_backend='bbs'))
    monkeypatch.setattr(ActionClient,'send_goal_async',lambda *a,**k:pytest.fail('startup sent an Action goal'))
    try:
        args=[mode,'--no-clock-hold','--startup-timeout-sec','5',
              *(['--manual-localization'] if manual else [])]
        if replies=='lost_all':
            with pytest.raises(ValueError,match='3 次有界只读尝试'):field_start.main(args)
            assert not ready[0] and len(sends)==3
        else:
            assert field_start.main(args)==130
            assert ready[0]
            if replies=='lost_first':assert len(sends)==2
        assert events.count('relocalize')==(0 if manual else 1)
        assert events.index('activate_map')<events.index('activate_navigation')
        launched=[e for e in events if isinstance(e,tuple) and e[0]=='launch']
        assert len([e for e in launched if e[1]=='field_session'])==1
        assert len([e for e in events if isinstance(e,tuple) and e[0]=='stop'])==len(launched)
        if mode=='photo':
            assert [e[1] for e in launched]==['field_session','mode_camera','mode_runtime','mode_queue','mode_rviz']
        if mode=='distance':
            runtime=next(e[2] for e in launched if e[1]=='mode_runtime')
            assert 'nav_behavior_tree:=""' in runtime
    finally:
        executor.shutdown(timeout_sec=3.);spin.join(timeout=3.)
        peer.destroy_node();ctx.try_shutdown()
        assert not spin.is_alive()
