"""Startup ownership must block goals without gating the legacy split workflow."""
import os

import pytest

from workbench.mode_state import publish_mode_state, require_mode_ready
from workbench.runtime import _pid_start_ticks, write_json


@pytest.mark.parametrize('state',['STARTING','STOPPING','STOP_FAILED'])
def test_live_unready_supervisor_blocks_submission(tmp_path,state):
    publish_mode_state(tmp_path,'distance','fixture',state)
    with pytest.raises(ValueError,match='本次未提交任务'):
        require_mode_ready(tmp_path)


def test_ready_supervisor_permits_submission(tmp_path):
    publish_mode_state(tmp_path,'distance','fixture','READY')
    require_mode_ready(tmp_path)


@pytest.mark.parametrize('owner',['missing','dead','reused'])
def test_old_mode_state_leaves_split_workflow_available(tmp_path,owner):
    if owner!='missing':
        pid=os.getpid() if owner=='reused' else 2**30
        write_json(tmp_path/'work/field_mode_state.json',dict(
            state='STARTING',pid=pid,process_start_ticks=(_pid_start_ticks(pid) or 0)+1))
    require_mode_ready(tmp_path)


@pytest.mark.parametrize('value',['{','[]'])
def test_unreadable_state_does_not_submit(tmp_path,value):
    (tmp_path/'work').mkdir()
    (tmp_path/'work/field_mode_state.json').write_text(value)
    with pytest.raises(ValueError,match='本次未提交任务'):
        require_mode_ready(tmp_path)


def test_photo_start_is_blocked_before_ros_init(tmp_path,monkeypatch):
    import rclpy
    from workbench import field_photo
    publish_mode_state(tmp_path,'photo','fixture','STARTING')
    monkeypatch.setattr(field_photo,'ROOT',tmp_path)
    monkeypatch.setattr(rclpy,'init',lambda **_:pytest.fail('unready start initialized ROS'))
    with pytest.raises(ValueError,match='本次未提交任务'):
        field_photo.check_photo_runtime()
