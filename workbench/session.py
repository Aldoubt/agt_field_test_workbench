"""Fixed field-session operations and immutable selected configuration snapshots."""
import json
from pathlib import Path
import shutil
import socket
import time
from .runtime import ROOT,REPO,ROS_EXEC,write_json,ros_ready
from .maps import Maps,_new_id,hash_file,relocalization_assets
from .fwmini import candidate_path,resolve_profile,load_profile,digest
from .safety import setup,report
from agt_field_test_bringup import profiles,calibration


def rpc(root,operation,payload=None):
    path=Path(root)/'work/ros_bridge.sock'
    with socket.socket(socket.AF_UNIX,socket.SOCK_STREAM) as client:
        client.settimeout(8);client.connect(str(path));client.sendall((json.dumps(dict(operation=operation,payload=payload or {}))+'\n').encode())
        chunks=[]
        while True:
            data=client.recv(65536)
            if not data:break
            chunks.append(data)
            if b'\n' in data:break
    response=json.loads(b''.join(chunks))
    if 'error' in response:raise ValueError(response['error'])
    return response

class Sessions:
    def __init__(self,jobs,root=ROOT):self.jobs=jobs;self.root=Path(root)
    def bridge(self):
        if not ros_ready():raise ValueError('ROS Humble runtime unavailable; start the workbench in the deployed Humble workspace')
        try:return rpc(self.root,'ping')
        except (OSError,ValueError):
            if any(j['kind']=='ros_bridge' and j['state']=='RUNNING' for j in self.jobs.list()):raise ValueError('ROS bridge starting; retry shortly')
            self.jobs._new('ros_bridge',['bash',str(ROS_EXEC),'python3',str(REPO/'scripts/ros_bridge.py')],{})
            # A GUI action should not require a second click merely because the
            # bridge process needed a normal startup interval.
            deadline=time.monotonic()+6
            while time.monotonic()<deadline:
                try:return rpc(self.root,'ping')
                except (OSError,ValueError):time.sleep(.1)
            raise ValueError('ROS bridge did not become ready within 6 seconds')
    def current(self):
        path=self.root/'work/current_session.json'
        return json.loads(path.read_text()) if path.is_file() else None
    def start(self,p):
        robot=p.get('robot');task=p.get('task','sensors')
        if robot not in ('bunker','fwmini') or task not in ('sensors','manual','localization','navigation'):raise ValueError('unknown robot/task')
        if any(j['kind']=='mapping_live' and j['state']=='RUNNING' for j in self.jobs.list()):raise ValueError('finish live mapping before changing runtime; start a manual session before the next live run')
        active=[j for j in self.jobs.list() if j['kind']=='field_session' and j['state'] in ('RUNNING','ORPHANED')]
        if active:raise ValueError('stop the current field session before changing robot/configuration')
        self.bridge()
        limits=profiles.load(profiles.selected_path(self.root,robot),robot)
        ident=_new_id();directory=self.root/'runs'/ident;directory.mkdir(parents=True)
        profile=directory/'robot.yaml';shutil.copyfile(limits['profile_path'],profile)
        checks=setup(p.get('checks')) if task in ('manual','navigation') else {}
        selected=None;cal_sha=None;bundle=None
        if robot=='bunker':
            selected=Path(p.get('calibration_bundle') or self.root/'calibration/bunker/bunker_calibration_candidate.yaml').resolve()
            if not selected.is_relative_to((self.root/'calibration/bunker').resolve()):raise ValueError('Bunker calibration must be a workspace version')
            bundle=calibration.load(selected,robot);cal_sha=calibration.digest(bundle)
        else:
            selected=resolve_profile(self.root,p.get('mount_profile'));mount=load_profile(selected);cal_sha=digest(mount)
        copied=directory/'calibration.yaml';shutil.copyfile(selected,copied)
        map_record=Maps(self.root).get(p['map_id']) if p.get('map_id') else None
        if task in ('localization','navigation') and (not map_record or not map_record.get('source_pcd')):raise ValueError('map with source PCD required')
        acceptance=None
        if task=='navigation':
            if map_record['status']!='ACCEPTED':raise ValueError('formal navigation requires ACCEPTED map')
            if (bundle or mount)['status']!='VERIFIED':raise ValueError('formal navigation requires VERIFIED calibration')
            acceptance=report(p.get('safety_report',''),robot,limits['profile_sha256'],map_record['sha256'],cal_sha)
        args=['robot:='+robot,'robot_profile_file:='+str(profile),'enable_chassis:='+str(task in ('manual','navigation')).lower(),
              'enable_localization:='+str(task in ('localization','navigation')).lower(),'enable_navigation:='+str(task=='navigation').lower(),
              'enable_mid360:='+str(bool(p.get('enable_sensor',True))).lower()]
        if p.get('livox_config'):args+=['livox_config:='+str(Path(p['livox_config']).resolve())]
        if robot=='bunker':args+=['calibration_bundle:='+str(copied)]
        else:
            args+=['fwmini_mount_file:='+str(copied),'fwmini_mode:='+('autonomous_navigation' if task=='navigation' else 'supervised_localization' if task=='localization' else 'sensor_test'),'supervised:=true']
            if p.get('geometry_file'):args+=['fwmini_geometry_file:='+str(Path(p['geometry_file']).resolve())]
        if p.get('gps_alignment_file'):args+=['gps_alignment_file:='+str(Path(p['gps_alignment_file']).resolve())]
        if map_record:args+=['global_map:='+map_record['source_pcd'],'map_yaml:='+map_record['yaml']]
        if acceptance:
            args+=['safety_report:='+acceptance['path']]
            if robot=='fwmini':
                import yaml
                runtime_mount=yaml.safe_load(copied.read_text())
                runtime_mount['validation'].update(localization='PASS',motion_safety='PASS')
                copied.write_text(yaml.safe_dump(runtime_mount,sort_keys=False))
        backend=p.get('global_backend','bbs')
        if backend not in ('bbs','kiss'):raise ValueError('unknown global localization backend')
        lio_backend=p.get('lio_backend','batch_lio')
        if lio_backend not in ('batch_lio','fastlio2'):raise ValueError('unknown local odometry backend')
        args+=['global_backend:='+backend,'lio_backend:='+lio_backend]
        assets=relocalization_assets(map_record) if map_record else ''
        if assets:args+=['relocalization_assets:='+assets]
        if task=='navigation':
            args+=['enable_inspection:=true','enable_camera_gimbal:='+str(bool(p.get('enable_camera_gimbal',True))).lower()]
            for key in ('camera_device_path','gimbal_port_name'):
                if p.get(key):args+=[key+':='+str(p[key])]
        snapshot=dict(id=ident,robot=robot,task=task,profile=limits,profile_snapshot=str(profile),calibration=str(copied),
            calibration_sha256=cal_sha,calibration_snapshot_sha256=hash_file(copied),checks=checks,map_id=map_record['id'] if map_record else None,
            map_sha256=map_record['sha256'] if map_record else None,
            map_yaml_sha256=hash_file(map_record['yaml']) if map_record else None,
            localization_pcd=map_record.get('source_pcd') if map_record else None,
            localization_pcd_sha256=map_record.get('source_pcd_sha256') if map_record else None,
            global_backend=backend,lio_backend=lio_backend,relocalization_assets=assets,
            enable_camera_gimbal=bool(p.get('enable_camera_gimbal',True)) if task=='navigation' else False,
            safety_report=acceptance)
        write_json(directory/'session.json',snapshot)
        job_meta=dict(snapshot);job_meta['session_id']=job_meta.pop('id')
        job=self.jobs._new('field_session',['bash',str(REPO/'scripts/run_field_test.sh'),*args],job_meta)
        write_json(self.root/'work/current_session.json',dict(snapshot,job_id=job['id']))
        return job
    def stop(self):
        try:rpc(self.root,'stop')
        except (OSError,ValueError):pass
        result=[]
        for job in self.jobs.list():
            if job['kind']=='field_session' and job['state']=='RUNNING':result.append(self.jobs.stop(job['id']))
        (self.root/'work/current_session.json').unlink(missing_ok=True)
        return result
    def stop_all(self):
        """Stop every process owned by this Workbench, including recorder and bridge."""
        self.stop()
        self.jobs.stop_all()
        return self.jobs.list()
