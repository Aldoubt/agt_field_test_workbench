"""Distance submission checks against a real parameter service; never execute ros2 send_goal."""
import importlib.util
import os
from pathlib import Path
import threading

import pytest
import yaml

pytestmark=pytest.mark.skipif(
    os.environ.get('ROS_DOMAIN_ID')!='183' or os.environ.get('ROS_LOCALHOST_ONLY')!='1',
    reason='requires isolated ROS domain 183 on localhost')


@pytest.mark.parametrize('case',['precision','photo_runtime','stale_localization','starting','ready'])
def test_distance_execution_needs_ready_precision_runtime(tmp_path,monkeypatch,case):
    import rclpy
    from rclpy.context import Context
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import Parameter
    from rcl_interfaces.srv import GetParameters
    from workbench import field_trial
    from workbench.mode_state import publish_mode_state
    repo=Path(__file__).resolve().parents[1]
    spec=importlib.util.spec_from_file_location('distance_test_tasks',repo/'scripts/field_tasks.py')
    tasks=importlib.util.module_from_spec(spec);spec.loader.exec_module(tasks)
    monkeypatch.setattr(tasks,'ROOT',tmp_path)
    if case in ('starting','ready'):
        publish_mode_state(tmp_path,'distance','fixture_map',case.upper())
    if case=='starting':
        monkeypatch.setattr(rclpy,'init',lambda **_:pytest.fail('unready start initialized ROS'))
    (tmp_path/'config').mkdir();(tmp_path/'config/field_trial.yaml').write_text('map_id: fixture_map\n')
    book=dict(map_id='fixture_map',points=dict(home=dict(pose=dict(x=0.,y=0.,yaw=0.,frame_id='map'))))
    mission=tmp_path/'accuracy_5m.yaml';tasks.save_new(mission,tasks.accuracy_mission(book,5.))
    def navigation(node):
        if case=='stale_localization':raise ValueError('localization stale')
    monkeypatch.setattr(field_trial,'require_navigation_ready',navigation)
    executed=[]
    monkeypatch.setattr(tasks.os,'execvp',lambda command,args:executed.append((command,args)))
    if case=='starting':
        with pytest.raises(ValueError,match='本次未提交任务'):tasks.execute_distance(mission)
        assert not executed
        return
    ctx=Context();rclpy.init(context=ctx);peer=Node('isolated_distance_params',context=ctx)
    def parameters(request,response):
        assert request.names==['nav_behavior_tree']
        response.values=[Parameter('nav_behavior_tree',value='photo.xml' if case=='photo_runtime' else '').get_parameter_value()]
        return response
    peer.create_service(GetParameters,'/mission_runtime/get_parameters',parameters)
    executor=SingleThreadedExecutor(context=ctx);executor.add_node(peer)
    spin=threading.Thread(target=executor.spin,daemon=True);spin.start()
    try:
        if case in ('precision','ready'):
            tasks.execute_distance(mission)
            assert len(executed)==1
            _,command=executed[0]
            assert command[:4]==['ros2','action','send_goal','/agt/mission/execute']
            goal=yaml.safe_load(command[-2])
            assert goal==dict(mission_file=str(mission),resume=False)
        else:
            with pytest.raises(ValueError):tasks.execute_distance(mission)
            assert not executed
    finally:
        executor.shutdown(timeout_sec=3.);spin.join(timeout=3.)
        peer.destroy_node();ctx.try_shutdown()
        assert not spin.is_alive()
