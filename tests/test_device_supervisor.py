"""Camera is optional: it must not own or stop a robot field session."""
from pathlib import Path
from unittest.mock import patch
import pytest
from workbench import devices


class FakeJobs:
    def __init__(self, jobs=None):
        self.items=list(jobs or [])
        self.stopped=[]
    def list(self):return list(self.items)
    def _new(self, kind, cmd, meta):
        job={'id':'camera-1','kind':kind,'state':'RUNNING','command':cmd,**meta}
        self.items.append(job);return job
    def stop(self, id):
        self.stopped.append(id)
        job=next(j for j in self.items if j['id']==id)
        job['state']='CANCELLED'
        return job


def test_camera_launch_is_detached_from_navigation_and_uses_fixed_argv():
    sessions=[{'id':'nav-1','kind':'field_session','state':'RUNNING','enable_camera_gimbal':False}]
    jobs=FakeJobs(sessions)
    with patch.object(devices,'ros_ready',return_value=True),\
         patch.object(devices,'ros_call',return_value='prefix'),\
         patch.object(Path,'exists',return_value=True):
        started=devices.start_camera(jobs,{'device_path':'/dev/video0','port_name':'/dev/ttyUSB1'})
    assert started['kind']=='camera_gimbal'
    assert started['command'][-3:]==['device_path:=/dev/video0','port_name:=/dev/ttyUSB1','gui:=false']
    assert len([x for x in jobs.list() if x['kind']=='field_session'])==1
    devices.stop_camera(jobs)
    assert jobs.stopped==['camera-1']
    assert jobs.list()[0]['state']=='RUNNING'


def test_missing_hardware_and_duplicate_owner_fail_closed():
    with patch.object(devices,'ros_ready',return_value=True),\
         patch.object(Path,'exists',return_value=False):
        with pytest.raises(ValueError,match='not connected'):
            devices.start_camera(FakeJobs(),{'device_path':'/dev/video0','port_name':'/dev/ttyUSB1'})
    with patch.object(devices,'ros_ready',return_value=True),\
         patch.object(Path,'exists',return_value=True):
        with pytest.raises(ValueError,match='already'):
            devices.start_camera(FakeJobs([{'kind':'camera_gimbal','state':'RUNNING'}]),
                                  {'device_path':'/dev/video0','port_name':'/dev/ttyUSB1'})


def test_untrusted_path_rejected_without_shell_execution():
    with pytest.raises(ValueError,match='camera'):
        devices._device('/tmp/attacker.sh','camera')
    with pytest.raises(ValueError,match='gimbal'):
        devices._device('/tmp/ttyUSB0','gimbal')

def test_camera_action_creates_immutable_request_and_checks_health(tmp_path, monkeypatch):
    import json
    from workbench import devices

    class Jobs(FakeJobs):
        def __init__(self):
            super().__init__([{'id':'camera-1','kind':'camera_gimbal','state':'RUNNING',
                'capture_root':str(tmp_path/'data'/'captures'/'session-1'),
                'capture_session_id':'session-1'}])
    root=tmp_path
    (root/'data'/'captures'/'session-1').mkdir(parents=True)
    monkeypatch.setattr(devices,'ROOT',root)
    monkeypatch.setattr(devices,'REPO',root/'src'/'repo')
    monkeypatch.setattr(devices,'ROS_EXEC',root/'src'/'repo'/'scripts'/'ros_exec.sh')
    monkeypatch.setattr(devices,'_health',lambda:{'state':1,'state_name':'READY',
        'camera_alive':True,'gimbal_serial_connected':True,'gimbal_feedback_alive':True,
        'move_action_ready':True,'busy':False})
    jobs=Jobs()
    result=devices.start_capture(jobs,{'heading':-25.,'pitch':12.,'roll':0.,
        'tag':'P01_left','metadata':{'map_version':None,'pose':{'x':1.,'y':2.,'yaw':.5}}},save_image=True)
    assert result['kind']=='camera_capture'
    request=json.loads(Path(result['request_path']).read_text())
    assert request['heading']==-25.
    assert request['tag']=='P01_left'
    assert request['metadata']['pose']['x']==1.
    assert result['scope']=='C1_GIMBAL_ACTION_NO_BASE_MOTION'
    assert result['result_path'].endswith('result.json')
    assert '--capture-root' in result['command']


def test_camera_action_refuses_unhealthy_hardware_and_mechanical_limit(tmp_path,monkeypatch):
    from workbench import devices
    class Jobs(FakeJobs):
        def __init__(self):
            super().__init__([{'id':'camera-1','kind':'camera_gimbal','state':'RUNNING',
                'capture_root':str(tmp_path/'data'/'captures'/'session-1')}])
    root=tmp_path;(root/'data'/'captures'/'session-1').mkdir(parents=True)
    monkeypatch.setattr(devices,'ROOT',root)
    monkeypatch.setattr(devices,'REPO',root/'repo')
    monkeypatch.setattr(devices,'ROS_EXEC',root/'repo'/'ros_exec.sh')
    jobs=Jobs()
    monkeypatch.setattr(devices,'_health',lambda:{'state':3,'state_name':'DEGRADED',
        'camera_alive':True,'gimbal_serial_connected':False,'gimbal_feedback_alive':False,
        'move_action_ready':True,'busy':False})
    with pytest.raises(ValueError,match='not READY'):
        devices.start_capture(jobs,{'heading':0.,'pitch':0.})
    assert not any(j['kind']=='camera_capture' for j in jobs.list())
    monkeypatch.setattr(devices,'_health',lambda:{'state':1,'state_name':'READY',
        'camera_alive':True,'gimbal_serial_connected':True,'gimbal_feedback_alive':True,
        'move_action_ready':True,'busy':False})
    with pytest.raises(ValueError,match='heading'):
        devices.start_capture(jobs,{'heading':170.,'pitch':0.})
