"""Read retries over actual localhost DDS; no hardware or mutating requests."""
import os
import threading
import time
import uuid

import pytest

pytestmark=pytest.mark.skipif(
    os.environ.get('ROS_DOMAIN_ID')!='183' or os.environ.get('ROS_LOCALHOST_ONLY')!='1',
    reason='requires isolated domain 183 on localhost')


@pytest.mark.parametrize('case',['lost_first','lost_all','bad_count','health_failure'])
def test_parameter_response_retries_are_bounded_and_clean_up(case,monkeypatch,capsys):
    import rclpy
    from rclpy.context import Context
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import Parameter
    from rcl_interfaces.srv import GetParameters
    from workbench.ros_parameters import read_parameters
    ctx=Context();rclpy.init(context=ctx);rclpy.init(args=[])
    name='parameter_retry_'+uuid.uuid4().hex[:8]
    peer=Node(name+'_server',context=ctx);node=Node(name+'_client')
    requests=[];delayed=[];clients=[]
    def parameters(request,response):
        requests.append(request)
        # A late first response must not be confused with the second request.
        value='expired' if len(requests)==1 and case=='lost_first' else ''
        response.values=[] if case=='bad_count' else [Parameter('nav_behavior_tree',value=value).get_parameter_value()]
        return response
    service=peer.create_service(GetParameters,'/'+name+'/get_parameters',parameters)
    original_send=service.send_response
    def send(response,header):
        if case in ('lost_all','health_failure'):return
        if case=='lost_first' and len(requests)==1:
            delayed.append((response,header));return
        if delayed:
            original_send(*delayed.pop())
        original_send(response,header)
    monkeypatch.setattr(service,'send_response',send)
    original_create=node.create_client
    def create(*args,**kwargs):
        client=original_create(*args,**kwargs);clients.append(client);return client
    monkeypatch.setattr(node,'create_client',create)
    executor=SingleThreadedExecutor(context=ctx);executor.add_node(peer)
    spin=threading.Thread(target=executor.spin,daemon=True);spin.start()
    def health():
        if case=='health_failure' and requests:raise ValueError('component exited')
    try:
        start=time.monotonic()
        if case=='lost_first':
            assert read_parameters(node,name,['nav_behavior_tree'],response_timeout=.2,
                                   health_check=health)=={'nav_behavior_tree':''}
            assert len(requests)==2
            assert '重试 2/3' in capsys.readouterr().out
        else:
            message={'lost_all':'3 次有界只读尝试','bad_count':'invalid parameter count',
                     'health_failure':'component exited'}[case]
            with pytest.raises(ValueError,match=message):
                read_parameters(node,name,['nav_behavior_tree'],response_timeout=.2,health_check=health)
            assert len(requests)==(3 if case=='lost_all' else 1)
        assert time.monotonic()-start<5.
        assert len(clients)==1 and not clients[0]._pending_requests
        assert not list(node.clients)
    finally:
        executor.shutdown(timeout_sec=3.);spin.join(timeout=3.)
        node.destroy_node();peer.destroy_node();ctx.try_shutdown();rclpy.shutdown()
        assert not spin.is_alive()
