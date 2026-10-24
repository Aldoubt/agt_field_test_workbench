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
