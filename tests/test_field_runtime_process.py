"""Run the exact distance-runtime argv on Humble in private DDS, without goals."""
import os
import signal
import subprocess
import time
import uuid

import pytest

from workbench import field_start


def test_real_distance_executor_starts_with_empty_tree_parameter(tmp_path,monkeypatch):
    if os.environ.get('ROS_DOMAIN_ID')!='183' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
        pytest.skip('requires isolated domain 183 on localhost')
    import rclpy
    from rclpy.parameter import parameter_value_to_python
    from rcl_interfaces.srv import GetParameters
    from rclpy.action import ActionClient
    from agt_robot_interfaces.action import ExecuteInspectionMission
    monkeypatch.setattr(field_start.field_trial,'make_plan',lambda *a,**k:dict(map_id='fixture',global_backend='bbs'))
    plan=field_start.make_mode_plan(field_start.parser().parse_args(['distance']))
    command=next(c['command'] for c in plan['components'] if c['name']=='runtime')
    assert 'nav_behavior_tree:=""' in command
    prefix='/distance_process_'+uuid.uuid4().hex[:8]
    command += ['-p','mission_action:='+prefix+'/mission',
                '-p','nav_action:='+prefix+'/nav', '-p','acquire_view_action:='+prefix+'/camera',
                '-p','local_odom_topic:='+prefix+'/odom',
                '-r','/tf:='+prefix+'/tf', '-r','/tf_static:='+prefix+'/tf_static']
    output=tmp_path/'runtime.log'
    with output.open('wb') as log:
        process=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    rclpy.init(args=[]);node=rclpy.create_node('distance_process_parameter_check')
    client=node.create_client(GetParameters,'/mission_runtime/get_parameters')
    action=ActionClient(node,ExecuteInspectionMission,prefix+'/mission')
    # Any submitted goal would fail the test. Only discover the action server.
    monkeypatch.setattr(action,'send_goal_async',lambda *a,**k:pytest.fail('test submitted a mission'))
    try:
        deadline=time.monotonic()+8.
        while time.monotonic()<deadline:
            assert process.poll() is None,output.read_text()
            rclpy.spin_once(node,timeout_sec=.05)
            if client.service_is_ready() and action.server_is_ready():break
        assert client.service_is_ready() and action.server_is_ready(),output.read_text()
        future=client.call_async(GetParameters.Request(names=['nav_behavior_tree','recenter_after_capture']))
        rclpy.spin_until_future_complete(node,future,timeout_sec=2.)
        assert future.done() and future.result() is not None
        values=[parameter_value_to_python(v) for v in future.result().values]
        assert values==['',False]
        assert process.poll() is None
    finally:
        if process.poll() is None:
            os.killpg(process.pid,signal.SIGINT)
        try:process.wait(timeout=5.)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=3.)
        action.destroy();node.destroy_node();rclpy.shutdown()
    assert process.returncode in (0,130,-signal.SIGINT),output.read_text()
