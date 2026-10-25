"""Regression tests for reviewed camera results, immutable maps and component ownership.

These tests use fixtures/processes, never claim physical hardware verification.
"""
import base64
import importlib.util
import json
import math
from pathlib import Path
import signal
import subprocess
import sys
from unittest.mock import Mock
import pytest
import yaml

from workbench.maps import Maps, hash_file, read_pgm
from workbench.runtime import Jobs, write_json
from workbench.session import Sessions
from test_map_diff import make_map

REPO=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('camera_capture',REPO/'scripts/camera_capture.py')
camera=importlib.util.module_from_spec(spec);spec.loader.exec_module(camera)


def capture_document(tmp_path):
    image=tmp_path/'capture.png'
    from workbench.maps import png
    image.write_bytes(png(1,1,b'\xff'))
    return dict(success=True,action_status=4,error_code=0,save_image=True,image_path=str(image),
        actual_angles=dict(heading=20.,pitch=5.,roll=0.),requested_angles=dict(heading=20.,pitch=5.,roll=0.),
        reached_stamp={'nanoseconds':100},image_stamp={'nanoseconds':101})


def test_capture_validates_disk_hash_and_actual_angles(tmp_path):
    doc=capture_document(tmp_path);camera.validate_capture(doc,tmp_path)
    assert doc['image_sha256']==hash_file(Path(doc['image_path']))
    doc['actual_angles']['heading']=22.
    with pytest.raises(RuntimeError,match='tolerance'):camera.validate_capture(doc,tmp_path)


@pytest.mark.parametrize('field,value,reason',[
    ('action_status',6,'did not succeed'),('image_stamp',{'nanoseconds':99},'not newer'),
    ('image_path','/outside.png','missing/outside')])
def test_capture_rejects_false_success(tmp_path,field,value,reason):
    doc=capture_document(tmp_path);doc[field]=value
    with pytest.raises(RuntimeError,match=reason):camera.validate_capture(doc,tmp_path)


def test_move_still_requires_fresh_frame_but_not_disk(tmp_path):
    doc=capture_document(tmp_path);doc.update(save_image=False,image_path='')
    camera.validate_capture(doc,tmp_path)
    doc['image_stamp']['nanoseconds']=100
    with pytest.raises(RuntimeError,match='not newer'):camera.validate_capture(doc,tmp_path)


def test_camera_import_failure_leaves_result_and_audit(tmp_path):
    request=tmp_path/'request.json';result=tmp_path/'result.json'
    request.write_text(json.dumps(dict(heading=0.,pitch=0.,roll=0.)))
    # Host Python intentionally has no ROS overlay; this is a failure-path test.
    command=[sys.executable,str(REPO/'scripts/camera_capture.py'),'--request',str(request),
             '--result',str(result),'--capture-root',str(tmp_path)]
    completed=subprocess.run(command,capture_output=True,timeout=5)
    if completed.returncode==0:pytest.skip('requires Python without a ROS overlay')
    assert json.loads(result.read_text())['success'] is False
    assert (tmp_path/'actions.jsonl').is_file()


def test_map_diff_exact_statistics_and_review_protection(tmp_path):
    maps=Maps(tmp_path)
    base=make_map(maps,tmp_path,'base',[254,0,205,254,0,205],'ACCEPTED')
    new=make_map(maps,tmp_path,'new',[0,254,0,205,0,205],'DRAFT')
    before=Path(base['pgm']).read_bytes()
    result=maps.compare_and_propose(base['id'],new['id'])
    update=result['update_map'];counts=result['report']['counts']
    assert counts==dict(new_obstacle_cells=1,cleared_obstacle_cells=1,newly_observed_cells=1,
        coverage_lost_cells=1,stable_occupied_cells=1,stable_free_cells=0,changed_cells=4,unknown_both_cells=1)
    assert base64.b64decode(result['overlay_png_b64']).startswith(b'\x89PNG')
    assert maps.get(update['id'])['diff_report']==result['report']['report_path']
    with pytest.raises(ValueError,match='explicit review'):
        maps.revise(update['id'],base64.b64encode(read_pgm(Path(update['pgm']))[2]),'ACCEPTED')
    accepted=maps.set_status(update['id'],'ACCEPTED','synthetic fixture reviewer confirms known changes')
    assert accepted['review_required'] is False
    assert Path(base['pgm']).read_bytes()==before
    assert maps.get(base['id'])['status']=='ACCEPTED'


def test_map_requires_alignment_evidence_and_exact_rotated_offsets(tmp_path):
    maps=Maps(tmp_path)
    base=make_map(maps,tmp_path,'base',[254]*6,'ACCEPTED',origin=(0.,0.,math.pi/2))
    new=make_map(maps,tmp_path,'new',[254]*6,'DRAFT',origin=(0.,1.,math.pi/2))
    result=maps.compare_and_propose(base['id'],new['id'])
    assert result['report']['candidate_offset_cells_xy']==[1,0]
    manifest=maps.directory/new['id']/'manifest.json'
    record=json.loads(manifest.read_text());record.pop('coordinate_reference_id');write_json(manifest,record)
    with pytest.raises(ValueError,match='alignment evidence'):maps.compare_and_propose(base['id'],new['id'])
    alignment=dict(method='EXTERNAL_ALIGNMENT_REVIEWED',evidence='synthetic known transform identity',
        base_pcd_sha256=base['source_pcd_sha256'],candidate_pcd_sha256=new['source_pcd_sha256'])
    assert maps.compare_and_propose(base['id'],new['id'],alignment)['update_map']['status']=='DRAFT'


def test_map_yaml_tamper_and_fractional_translation_rejected(tmp_path):
    maps=Maps(tmp_path);base=make_map(maps,tmp_path,'base',[254]*6,'ACCEPTED')
    new=make_map(maps,tmp_path,'new',[254]*6,'DRAFT',origin=(.1,0.,0.))
    with pytest.raises(ValueError,match='cell-aligned'):maps.compare_and_propose(base['id'],new['id'])
    Path(base['yaml']).write_text(Path(base['yaml']).read_text()+'\n# tamper\n')
    with pytest.raises(ValueError,match='YAML SHA'):maps.validate_identity(base)


def test_diff_artifact_tamper_blocks_preview_and_acceptance(tmp_path):
    maps=Maps(tmp_path);base=make_map(maps,tmp_path,'base',[254]*6,'ACCEPTED')
    new=make_map(maps,tmp_path,'new',[0]*6,'DRAFT')
    result=maps.compare_and_propose(base['id'],new['id']);ident=result['update_map']['id']
    assert maps.diff_preview(ident)['update_map']['status']=='DRAFT'
    path=maps.directory/ident/'manifest.json';bound=maps.get(ident);legacy=dict(bound);legacy.pop('diff_report')
    write_json(path,legacy)
    with pytest.raises(ValueError,match='legacy diff candidate'):maps.set_status(ident,'ACCEPTED','reject unbound legacy report')
    write_json(path,bound)
    base_source=Path(base['source_pcd']);original=base_source.read_bytes();base_source.write_bytes(b'changed after comparison')
    with pytest.raises(ValueError,match='source PCD'):maps.set_status(ident,'ACCEPTED','reject changed base source')
    base_source.write_bytes(original)
    Path(result['report']['report_path']).write_text('{}')
    with pytest.raises(ValueError,match='SHA-256'):maps.diff_preview(ident)
    with pytest.raises(ValueError,match='report SHA'):maps.set_status(ident,'ACCEPTED','must reject tampered report')


def test_grid_pixel_edit_preserves_localization_assets(tmp_path):
    maps=Maps(tmp_path);base=make_map(maps,tmp_path,'base',[254]*6,'ACCEPTED')
    edited=maps.revise(base['id'],base64.b64encode(bytes([0,254,254,254,254,254])))
    assert edited['source_pcd']==base['source_pcd']
    assert edited['source_pcd_sha256']==base['source_pcd_sha256']
    assert edited['localization_geometry_valid'] is True
    assert edited['navigation_grid_edited'] is True
    accepted=maps.set_status(edited['id'],'ACCEPTED','operator reviewed planner grid edit')
    assert accepted['status']=='ACCEPTED'


class MemoryJobs:
    def __init__(self):self.items=[];self.stopped=[];self.fail_kind=None
    def list(self):return list(reversed(self.items))
    def _new(self,kind,command,meta):
        if kind==self.fail_kind:raise ValueError('synthetic startup failure')
        row=dict(id=str(len(self.items)+1),kind=kind,state='RUNNING',command=command,**meta)
        self.items.append(row);return row
    def stop(self,ident):
        row=next(j for j in self.items if j['id']==ident)
        row['state']='CANCELLED';self.stopped.append(ident);return row


@pytest.fixture
def sensors(tmp_path,monkeypatch):
    from workbench import session
    from agt_field_test_bringup import profiles,calibration
    jobs=MemoryJobs();manager=Sessions(jobs,tmp_path)
    profile=tmp_path/'robot.yaml';profile.write_bytes((REPO/'templates/config/robots/bunker.yaml').read_bytes())
    cal=tmp_path/'calibration.yaml';cal.write_bytes((REPO/'agt_field_test_bringup/config/bunker_calibration_candidate.yaml').read_bytes())
    limits=profiles.load(profile,'bunker');bundle=calibration.load(cal,'bunker')
    snapshot=dict(id='sensors',robot='bunker',task='sensors',profile=limits,profile_snapshot=str(profile),
        calibration=str(cal),calibration_sha256=calibration.digest(bundle),calibration_snapshot_sha256=hash_file(cal))
    write_json(tmp_path/'runs/sensors/session.json',snapshot)
    job=jobs._new('sensor_stack',[],dict(session_id='sensors',robot='bunker'))
    write_json(tmp_path/'work/current_session.json',dict(snapshot,job_id=job['id']))
    monkeypatch.setattr(manager,'bridge',lambda:dict(ready=True))
    monkeypatch.setattr(session,'ros_call',lambda *a,**k:'installed')
    monkeypatch.setattr(session,'rpc',lambda *a,**k:{'state':'STOP_REQUESTED'})
    return manager,jobs,snapshot


def test_chassis_start_failure_leaves_sensors_running(sensors):
    manager,jobs,snapshot=sensors;jobs.fail_kind='chassis_component'
    with pytest.raises(ValueError,match='startup failure'):
        manager.start(dict(robot='bunker',task='manual',checks={k:True for k in ('wiring','physical_estop','remote_takeover','clear_area','supervised')}))
    assert jobs.items[0]['state']=='RUNNING' and not jobs.stopped


def test_guard_start_failure_stops_only_new_chassis(sensors):
    manager,jobs,snapshot=sensors;jobs.fail_kind='control_guard'
    with pytest.raises(ValueError,match='startup failure'):
        manager.start(dict(robot='bunker',task='manual',checks={k:True for k in ('wiring','physical_estop','remote_takeover','clear_area','supervised')}))
    assert jobs.items[0]['state']=='RUNNING' and jobs.items[1]['state']=='CANCELLED'


def test_independent_chassis_start_stop_and_duplicate_start(sensors):
    manager,jobs,snapshot=sensors
    p=dict(robot='bunker',task='manual',checks={k:True for k in ('wiring','physical_estop','remote_takeover','clear_area','supervised')})
    first=manager.start(p);second=manager.start(p)
    assert first['id']==second['id']
    assert 'enable_mid360:=false' in first['command']
    assert len([j for j in jobs.items if j['kind']=='chassis_component'])==1
    with pytest.raises(ValueError,match='still used'):manager.stop_component('sensors')
    manager.stop_chassis()
    assert jobs.items[0]['state']=='RUNNING' and manager.current()['task']=='sensors'
    assert all(j['state']=='CANCELLED' for j in jobs.items[1:])


def test_stopping_navigation_restores_localization_without_sensor_restart(sensors):
    manager,jobs,snapshot=sensors
    loc=jobs._new('field_session',[],dict(robot='bunker',task='localization'))
    loc_snapshot=dict(snapshot,task='localization',job_id=loc['id'])
    nav=jobs._new('navigation_component',[],dict(robot='bunker'))
    write_json(manager.root/'work/current_session.json',dict(loc_snapshot,task='navigation',job_id=nav['id'],localization_session=loc_snapshot))
    with pytest.raises(ValueError,match='still uses'):manager.stop_component('localization')
    manager.stop()
    assert loc['state']=='RUNNING' and jobs.items[0]['state']=='RUNNING'
    assert manager.current()['task']=='localization' and nav['state']=='CANCELLED'


def test_navigation_reuses_driver_localization_and_transfers_guard(sensors,monkeypatch):
    from workbench import session
    manager,jobs,snapshot=sensors
    checks={k:True for k in ('wiring','physical_estop','remote_takeover','clear_area','supervised')}
    manager.start(dict(robot='bunker',task='manual',checks=checks))
    maps=Maps(manager.root);m=make_map(maps,manager.root,'base',[254]*6,'ACCEPTED')
    assets=Path(m['source_pcd']).parent/'relocalization'
    (assets/'voxelmaps_coords').mkdir(parents=True)
    (assets/'global_map_downsampled.pcd').write_bytes(b'downsampled')
    (assets/'polar_context.db').write_bytes(b'candidate-index')
    (assets/'asset_status.json').write_text(json.dumps({
        'state':'READY','bbs':'READY','source_sha256':m['source_pcd_sha256']}))
    localization=jobs._new('field_session',[],dict(robot='bunker',task='localization'))
    current=dict(manager.current(),task='localization',job_id=localization['id'],map_id=m['id'],
        map_sha256=m['sha256'],localization_pcd_sha256=m['source_pcd_sha256'],
        lio_backend='fastlio2',global_backend='bbs')
    write_json(manager.root/'work/current_session.json',current)
    # Scope: lifecycle wiring only. Physical evidence is deliberately mocked here.
    monkeypatch.setattr(session.calibration,'load',lambda *a:dict(status='VERIFIED'))
    monkeypatch.setattr(session,'report',lambda *a,**k:dict(path='/synthetic/physical-report.json'))
    job=manager.start(dict(robot='bunker',task='navigation',map_id=m['id'],global_backend='bbs',lio_backend='fastlio2',checks=checks))
    assert job['kind']=='navigation_component'
    assert 'external_localization:=true' in job['command'] and 'enable_chassis:=false' in job['command']
    assert 'enable_description:=false' in job['command'] and 'enable_camera_gimbal:=false' in job['command']
    assert len([j for j in jobs.items if j['kind']=='chassis_component'])==1
    assert len([j for j in jobs.items if j['kind']=='field_session'])==1
    assert not any(j['state']=='RUNNING' for j in jobs.items if j['kind']=='control_guard')
    manager.stop()
    assert localization['state']=='RUNNING'
    assert any(j['state']=='RUNNING' for j in jobs.items if j['kind']=='control_guard')


def test_fwmini_independent_driver_uses_its_profile_and_preserves_sensors(sensors):
    from agt_field_test_bringup import profiles
    from workbench.fwmini import load_profile,digest
    manager,jobs,snapshot=sensors
    profile=manager.root/'fwmini.yaml';profile.write_bytes((REPO/'templates/config/robots/fwmini.yaml').read_bytes())
    mount=manager.root/'mount.yaml';mount.write_bytes((REPO/'agt_field_test_bringup/config/fwmini_mid360_candidate.yaml').read_bytes())
    selected=load_profile(mount);geometry=manager.root/'geometry.yaml'
    geometry.write_text(yaml.safe_dump(dict(body=dict(selected['body'],measured=True))))
    snapshot.update(robot='fwmini',profile=profiles.load(profile,'fwmini'),profile_snapshot=str(profile),
        calibration=str(mount),calibration_sha256=digest(selected),calibration_snapshot_sha256=hash_file(mount))
    jobs.items[0]['robot']='fwmini';write_json(manager.root/'runs/sensors/session.json',snapshot)
    write_json(manager.root/'work/current_session.json',dict(snapshot,job_id=jobs.items[0]['id']))
    job=manager.start(dict(robot='fwmini',task='manual',geometry_file=str(geometry),
        checks={k:True for k in ('wiring','physical_estop','remote_takeover','clear_area','supervised')}))
    assert 'robot:=fwmini' in job['command'] and 'enable_mid360:=false' in job['command']
    assert 'fwmini_mode:=sensor_test' in job['command']
    assert profiles.guard(snapshot['profile'],False)['max_linear_y']>0
    manager.stop_chassis();assert jobs.items[0]['state']=='RUNNING'


def test_process_exit_and_restart_orphan_have_explicit_states(tmp_path):
    jobs=Jobs(tmp_path)
    child=jobs._new('camera_gimbal',[sys.executable,'-c','pass'],{})
    jobs.jobs[child['id']]['process'].wait(timeout=3)
    record=jobs.list()[0]
    assert record['state']=='FAILED' and 'unexpectedly' in record['reason']
    foreign=tmp_path/'runs/foreign/run.json'
    write_json(foreign,dict(id='foreign',kind='chassis_component',state='RUNNING',pid=__import__('os').getpid()))
    assert next(j for j in jobs.list() if j['id']=='foreign')['state']=='ORPHANED'
    with pytest.raises(ValueError,match='orphaned'):jobs._new('chassis_component',[sys.executable,'-c','pass'],{})


def test_crashed_supervisor_child_group_is_stopped_before_resource_reuse(tmp_path):
    jobs=Jobs(tmp_path)
    command=[sys.executable,'-c','import subprocess; subprocess.Popen(["sleep","20"])']
    job=jobs._new('control_guard',command,{})
    jobs.jobs[job['id']]['process'].wait(timeout=3)
    try:
        assert jobs.list()[0]['state']=='ORPHANED'
        with pytest.raises(ValueError,match='orphaned'):jobs._new('control_guard',command,{})
    finally:
        jobs.stop(job['id'])
    assert jobs.list()[0]['state']=='CANCELLED'


def test_stop_failure_retains_resource_ownership(tmp_path,monkeypatch):
    jobs=Jobs(tmp_path);process=Mock();process.poll.return_value=None
    process.wait.side_effect=subprocess.TimeoutExpired('fixture',1);process.pid=123456789
    row=dict(id='one',kind='control_guard',state='RUNNING')
    jobs.jobs['one']=dict(process=process,record=row,kind='control_guard',state='RUNNING')
    monkeypatch.setattr('workbench.runtime.os.killpg',lambda *a:None)
    with pytest.raises(ValueError,match='stop failed'):jobs.stop('one')
    assert row['state']=='STOP_FAILED'
    with pytest.raises(ValueError,match='orphaned'):jobs._new('control_guard',[],{})


def test_shell_resource_locks_allow_independent_components_but_reject_duplicate(tmp_path):
    import time
    scripts=tmp_path/'src/workbench/scripts';scripts.mkdir(parents=True)
    script=scripts/'run_field_test.sh';script.write_bytes((REPO/'scripts/run_field_test.sh').read_bytes())
    (scripts/'ros_exec.sh').write_text('#!/bin/bash\nexec sleep 20\n')
    env=__import__('os').environ.copy();env['AGT_FIELD_TEST_WS']=str(tmp_path)
    sensor=subprocess.Popen(['bash',str(script)],env=env,start_new_session=True)
    chassis=None
    try:
        deadline=time.monotonic()+2
        while not (tmp_path/'work/sensor.lock').exists() and time.monotonic()<deadline:time.sleep(.01)
        assert sensor.poll() is None
        chassis_args=['enable_mid360:=false','enable_description:=false','enable_chassis:=true','enable_guard:=false']
        chassis=subprocess.Popen(['bash',str(script),*chassis_args],env=env,start_new_session=True)
        deadline=time.monotonic()+2
        while not (tmp_path/'work/chassis.lock').exists() and time.monotonic()<deadline:time.sleep(.01)
        while subprocess.run(['flock','-n',str(tmp_path/'work/chassis.lock'),'true']).returncode==0 and time.monotonic()<deadline:time.sleep(.01)
        duplicate=subprocess.run(['bash',str(script),*chassis_args],env=env,capture_output=True,timeout=2)
        assert duplicate.returncode==3 and b'chassis' in duplicate.stderr
        assert sensor.poll() is None and chassis.poll() is None
    finally:
        for process in (chassis,sensor):
            if process and process.poll() is None:
                __import__('os').killpg(process.pid,signal.SIGTERM);process.wait(timeout=3)
