"""Mode composition and ownership; subprocess fixtures never execute hardware commands."""
from pathlib import Path
from types import SimpleNamespace
import signal
import sys
import subprocess
import time

import pytest

from workbench import field_start
from workbench.runtime import Jobs


@pytest.fixture
def base(monkeypatch):
    monkeypatch.setattr(field_start.field_trial,'make_plan',
        lambda *a,**k:dict(map_id='test_map',global_backend='bbs'))


@pytest.mark.parametrize('mode,expected',[
    ('nav',['navigation_stack','runtime','rviz']),
    ('distance',['navigation_stack','runtime','rviz']),
    ('photo',['navigation_stack','camera','runtime','queue','rviz'])])
def test_three_modes_compose_existing_entrypoints_without_task_goals(base,mode,expected):
    args=field_start.parser().parse_args([mode])
    plan=field_start.make_mode_plan(args)
    assert [c['name'] for c in plan['components']]==expected
    assert plan['hold_system_clock'] and plan['clock_owner']=='foreground_mode'
    assert '--hold-system-clock' not in plan['components'][0]['command']
    commands=[c['command'] for c in plan['components']]
    assert commands[0][-2:]==['--planning-profile','distance' if mode=='distance' else 'navigation']
    assert all('send_goal' not in c and 'start' not in c[2:] for c in commands[1:])
    if mode=='photo':
        assert plan['external_task_trigger']=='./field_photo start'
        assert plan['components'][3]['command'][-2:]==['--map-id','test_map']
    elif mode in ('distance', 'nav'):
        assert 'nav_behavior_tree:=""' in commands[1]
        assert 'recenter_after_capture:=false' in commands[1]
    else:
        assert not any('mission_runtime' in c for c in commands)


def test_dry_run_never_creates_ros_nodes_jobs_or_locks(base,monkeypatch,tmp_path,capsys):
    monkeypatch.setattr(field_start,'ROOT',tmp_path)
    def forbidden(*a,**k):pytest.fail('dry-run touched ROS or subprocesses')
    monkeypatch.setattr(field_start,'Readiness',forbidden)
    monkeypatch.setattr(field_start,'Jobs',forbidden)
    for mode in ['nav','distance','photo']:
        assert field_start.main([mode,'--dry-run'])==0
    assert not (tmp_path/'work').exists()
    assert './field_photo start' in capsys.readouterr().out


def test_manual_headless_plan_keeps_localization_external(base):
    args=field_start.parser().parse_args(['photo','--manual-localization','--no-rviz','--no-clock-hold'])
    plan=field_start.make_mode_plan(args)
    assert not plan['automatic_relocalization']
    assert '--hold-system-clock' not in plan['components'][0]['command']
    assert 'rviz' not in [c['name'] for c in plan['components']]


@pytest.mark.parametrize('value',['0','-1','nan','inf'])
def test_invalid_startup_budgets_fail_before_start(base,value):
    with pytest.raises(SystemExit):
        field_start.parser().parse_args(['nav','--startup-timeout-sec',value])


@pytest.mark.parametrize('moving_at',[0,1])
def test_no_relocalization_request_until_stationary(moving_at):
    probe=object.__new__(field_start.Readiness)
    count=[0];calls=[]
    def stationary():
        count[0]+=1
        return count[0] <= moving_at
    probe.stationary=stationary
    probe.call=lambda typ,name,request: (calls.append(name) or
        SimpleNamespace(result=SimpleNamespace(successful=True)))
    with pytest.raises(ValueError,match='未请求重定位'):
        probe.relocalize(30.)
    assert '/agt/localization/relocalize' not in calls


def test_mode_cleanup_only_stops_owned_process_groups(tmp_path):
    # Two benign processes; the unrelated one must survive session.close().
    unrelated=subprocess.Popen([sys.executable,'-c','import time; time.sleep(30)'],start_new_session=True)
    command=[sys.executable,'-c','import time; time.sleep(30)']
    session=field_start.ModeSession(dict(mode='nav',components=[dict(name='navigation_stack',command=command)]),Jobs(tmp_path))
    try:
        session.start('navigation_stack')
        owned=session.jobs.jobs[session.owned[0][1]]['process']
        session.close()
        assert owned.poll() is not None
        assert unrelated.poll() is None
    finally:
        if unrelated.poll() is None:
            unrelated.terminate();unrelated.wait(timeout=3.)


def test_component_failure_does_not_restart_or_claim_ready(tmp_path):
    session=field_start.ModeSession(dict(mode='photo',components=[dict(name='camera',
        command=[sys.executable,'-c','raise SystemExit(9)'])]),Jobs(tmp_path))
    session.start('camera')
    process=session.jobs.jobs[session.owned[0][1]]['process']
    process.wait(timeout=3.)
    with pytest.raises(ValueError,match='camera exited'):
        session.check()
    session.close()
    assert len(session.owned)==1


def test_launch_zero_exit_surfaces_child_abort_before_peer_cleanup(tmp_path):
    session=field_start.ModeSession(dict(mode='nav',components=[dict(name='navigation_stack',
        command=[sys.executable,'-c',
            "print('[base] free(): invalid pointer'); "
            "print('[ERROR] [agt_bunker_base_node]: process has died, exit code -6'); "
            "print('peer process has finished cleanly\\n'*25)"])]),Jobs(tmp_path))
    session.start('navigation_stack')
    session.jobs.jobs[session.owned[0][1]]['process'].wait(timeout=3.)
    with pytest.raises(ValueError) as error:
        session.check()
    assert 'exited (0)' in str(error.value)
    assert 'free(): invalid pointer' in str(error.value)
    assert 'exit code -6' in str(error.value)
    session.close()


def test_existing_split_stack_is_left_running(base,monkeypatch,tmp_path):
    monkeypatch.setattr(field_start,'ROOT',tmp_path)
    def blocked(mode):raise ValueError('已有分开启动的组件')
    monkeypatch.setattr(field_start,'Readiness',lambda *_:SimpleNamespace(reject_existing=blocked,close=lambda:None))
    class NoProcesses:
        def __init__(self,*_):pass
        def _new(self,*_):pytest.fail('launched a duplicate component')
        def stop(self,*_):pytest.fail('stopped an existing component')
    monkeypatch.setattr(field_start,'Jobs',NoProcesses)
    with pytest.raises(ValueError,match='已有分开启动'):
        field_start.main(['photo'])


def test_relocalization_rechecks_stop_after_service_discovery():
    probe=object.__new__(field_start.Readiness)
    stopped=[True,True,False];calls=[]
    probe.stationary=lambda:stopped.pop(0)
    def call(typ,name,request,**kwargs):
        if kwargs.get('before_send'):kwargs['before_send']()
        calls.append(name)
        return SimpleNamespace(result=SimpleNamespace(successful=True))
    probe.call=call
    with pytest.raises(ValueError,match='服务发现期间车辆移动'):
        probe.relocalize(30.)
    assert '/agt/localization/relocalize' not in calls


def test_initialization_installs_missing_wrappers_without_overwriting_split_commands(tmp_path):
    source=field_start.REPO/'templates/bin'
    destination=tmp_path/'workspace';destination.mkdir()
    custom=destination/'field_trial';custom.write_text('# operator-owned entrypoint\n')
    subprocess.run(['cp','-an',str(source)+'/.',str(destination)+'/'],check=True)
    assert custom.read_text()=='# operator-owned entrypoint\n'
    for name in ['field_start','field_distance','field_photo']:
        path=destination/name
        assert path.is_file() and path.stat().st_mode & 0o111
        subprocess.run(['bash','-n',str(path)],check=True)
