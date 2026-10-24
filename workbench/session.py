"""Fixed field-session operations and immutable selected configuration snapshots."""
import json
from pathlib import Path
import shutil
import socket
import time
from .runtime import ROOT,REPO,ROS_EXEC,write_json,ros_ready,ros_call
from .maps import Maps,_new_id,hash_file,relocalization_assets
from .fwmini import candidate_path,resolve_profile,load_profile,digest
from .safety import setup,report
from agt_field_test_bringup import profiles,calibration
from agt_field_test_bringup.livox import validate_mid360s_config


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
            deadline=time.monotonic()+6
            while time.monotonic()<deadline:
                try:return rpc(self.root,'ping')
                except (OSError,ValueError):time.sleep(.1)
            raise ValueError('ROS bridge did not become ready within 6 seconds')

    def current(self):
        path=self.root/'work/current_session.json'
        return json.loads(path.read_text()) if path.is_file() else None

    def _jobs(self):return self.jobs.list()

    def _find_job(self,kind,states=('RUNNING',)):
        return next((j for j in self._jobs() if j.get('kind')==kind and j.get('state') in states),None)

    def _sensor_stack(self,robot=None):
        job=self._find_job('sensor_stack')
        if not job:return None,None
        if robot and job.get('robot')!=robot:
            raise ValueError('sensor stack belongs to '+str(job.get('robot'))+'; stop all components before switching robot')
        session_id=job.get('session_id')
        if not session_id:raise ValueError('sensor stack has no immutable session identity; stop and restart it')
        path=self.root/'runs'/session_id/'session.json'
        if not path.is_file():raise ValueError('sensor stack session snapshot is missing: '+str(path))
        snapshot=json.loads(path.read_text());snapshot['job_id']=job['id']
        return job,snapshot

    def _start_manual_guard(self,sensor):
        current=self._find_job('control_guard')
        if current:return current
        if any(j.get('kind')=='control_guard' and j.get('state')=='ORPHANED' for j in self._jobs()):
            raise ValueError('manual control guard is ORPHANED; inspect its process log before restarting')
        try:ros_call(['ros2','pkg','prefix','agt_base_control'],timeout=4)
        except Exception as exc:raise ValueError('agt_base_control/cmd_vel_guard is not installed in this ROS overlay') from exc
        import yaml
        profile=sensor.get('profile') or profiles.load(sensor['profile_snapshot'],sensor['robot'])
        config=Path(sensor['profile_snapshot']).resolve().parent/'guard_manual.yaml'
        config.write_text(yaml.safe_dump({'agt_cmd_vel_guard':{'ros__parameters':profiles.guard(profile,False)}},sort_keys=False))
        return self.jobs._new('control_guard',
            ['bash',str(ROS_EXEC),'ros2','run','agt_base_control','cmd_vel_guard','--ros-args','--params-file',str(config)],
            {'robot':sensor['robot'],'mode':'manual','session_id':sensor['id'],
             'profile_sha256':profile['profile_sha256'],'config_path':str(config),
             'scope':'INDEPENDENT_MANUAL_CMD_VEL_GUARD'})

    def _stop_kind(self,kind):
        stopped=[]
        for job in self._jobs():
            if job.get('kind')==kind and job.get('state') in ('RUNNING','ORPHANED'):
                stopped.append(self.jobs.stop(job['id']))
        return stopped

    def _restore_sensor_or_manual(self):
        sensor_job,sensor=self._sensor_stack()
        path=self.root/'work/current_session.json'
        if not sensor_job:
            path.unlink(missing_ok=True);return None
        chassis=self._find_job('chassis_component')
        snapshot=dict(sensor);snapshot['job_id']=sensor_job['id']
        if chassis:
            guard=self._start_manual_guard(sensor)
            snapshot.update(task='manual',chassis_enabled=True,chassis_job_id=chassis['id'],
                guard_job_id=guard['id'],sensor_stack_job_id=sensor_job['id'],checks=chassis.get('checks',{}))
        else:
            snapshot.update(task='sensors',chassis_enabled=False,sensor_stack_job_id=sensor_job['id'])
            snapshot.pop('chassis_job_id',None);snapshot.pop('guard_job_id',None)
        write_json(path,snapshot);return snapshot

    def start_chassis(self,p):
        robot=p.get('robot')
        if robot not in ('bunker','fwmini'):raise ValueError('unknown robot')
        if any(j['kind']=='mapping_live' and j['state']=='RUNNING' for j in self._jobs()):
            raise ValueError('finish live mapping before changing chassis runtime')
        active=[j for j in self._jobs() if j['kind']=='field_session' and j['state'] in ('RUNNING','ORPHANED')]
        if active:raise ValueError('stop localization/navigation session before starting or stopping the independent chassis')
        self.bridge()
        sensor_job,sensor=self._sensor_stack(robot)
        if not sensor_job:raise ValueError('start the matching MID360S/IMU sensor session before connecting the chassis')
        current=self.current()
        if current and current.get('task') not in ('sensors','manual'):
            raise ValueError('stop the active localization/navigation session first')
        existing=self._find_job('chassis_component')
        if existing:
            if existing.get('robot')!=robot:raise ValueError('another robot chassis component is already running')
            return dict(existing,session_task='manual',guard_job_id=(self._find_job('control_guard') or {}).get('id'))
        if any(j.get('kind')=='chassis_component' and j.get('state')=='ORPHANED' for j in self._jobs()):
            raise ValueError('chassis component is ORPHANED; inspect its process log before restarting')
        checks=setup(p.get('checks'))
        profile_path=Path(sensor['profile_snapshot']).resolve()
        calibration_path=Path(sensor['calibration']).resolve()
        if not profile_path.is_file() or hash_file(profile_path)!=sensor['profile']['profile_sha256']:
            raise ValueError('sensor session robot profile changed; restart sensors from the selected immutable profile')
        if not calibration_path.is_file() or hash_file(calibration_path)!=sensor['calibration_snapshot_sha256']:
            raise ValueError('sensor session calibration snapshot changed; restart sensors before enabling chassis')
        limits=profiles.load(profile_path,robot)
        if robot=='bunker':
            bundle=calibration.load(calibration_path,robot)
            if calibration.digest(bundle)!=sensor['calibration_sha256']:raise ValueError('sensor and chassis calibration SHA differ')
            mount=None
        else:
            mount=load_profile(calibration_path)
            if digest(mount)!=sensor['calibration_sha256']:raise ValueError('sensor and chassis mount-profile SHA differ')
        geometry_file=p.get('geometry_file') or sensor.get('geometry_file')
        if robot=='fwmini':
            if not geometry_file or not Path(geometry_file).is_file():
                raise ValueError('FW-mini chassis requires the measured geometry file')
            import yaml
            geometry=yaml.safe_load(Path(geometry_file).read_text())
            if geometry.get('body',{}).get('measured') is not True or any(
                abs(float(geometry['body'][key])-float(mount['body'][key]))>1e-6
                for key in ('length_m','width_m','height_m')):
                raise ValueError('measured FW-mini geometry must match the selected mount profile')
        for package in (('agt_bunker_base',) if robot=='bunker' else
                        ('yhs_can_control','fwmini_chassis_adapter','fwmini_odom_adapter')):
            try:ros_call(['ros2','pkg','prefix',package],timeout=4)
            except Exception as exc:raise ValueError('required chassis package is not installed: '+package) from exc
        args=['robot:='+robot,'robot_profile_file:='+str(profile_path),
              'enable_chassis:=true','enable_mid360:=false','enable_localization:=false',
              'enable_navigation:=false','enable_description:=false','enable_guard:=false',
              'external_chassis:=false','enable_inspection:=false','enable_camera_gimbal:=false']
        if robot=='bunker':args+=['calibration_bundle:='+str(calibration_path)]
        else:args+=['fwmini_mount_file:='+str(calibration_path),'fwmini_mode:=sensor_test','supervised:=true',
                    'fwmini_geometry_file:='+str(Path(geometry_file).resolve())]
        driver=self.jobs._new('chassis_component',
            ['bash',str(REPO/'scripts/run_field_test.sh'),*args],
            {'robot':robot,'task':'manual','session_id':sensor['id'],'profile_sha256':limits['profile_sha256'],
             'calibration_sha256':sensor['calibration_sha256'],'checks':checks,
             'sensor_stack_job_id':sensor_job['id'],'scope':'INDEPENDENT_CHASSIS_DRIVER_NO_SENSOR_RESTART'})
        try:guard=self._start_manual_guard(sensor)
        except Exception:
            self.jobs.stop(driver['id']);raise
        snapshot=dict(sensor,task='manual',job_id=sensor_job['id'],chassis_enabled=True,
            sensor_stack_job_id=sensor_job['id'],chassis_job_id=driver['id'],guard_job_id=guard['id'],checks=checks)
        write_json(self.root/'work/current_session.json',snapshot)
        return dict(driver,session_task='manual',guard_job_id=guard['id'])

    def stop_chassis(self):
        current=self.current()
        if current and current.get('task') in ('localization','navigation'):
            raise ValueError('stop the active localization/navigation session before stopping its chassis')
        try:rpc(self.root,'stop')
        except (OSError,ValueError):pass
        stopped=self._stop_kind('control_guard')+self._stop_kind('chassis_component')
        restored=self._restore_sensor_or_manual()
        return {'state':'STOPPED','stopped':stopped,'restored_session':restored}

    def start(self,p):
        robot=p.get('robot');task=p.get('task','sensors')
        if robot not in ('bunker','fwmini') or task not in ('sensors','manual','localization','navigation'):
            raise ValueError('unknown robot/task')
        if task=='manual':return self.start_chassis(p)
        if any(j['kind']=='mapping_live' and j['state']=='RUNNING' for j in self._jobs()):
            raise ValueError('finish live mapping before changing runtime; start a manual session before the next live run')
        active=[j for j in self._jobs() if j['kind']=='field_session' and j['state'] in ('RUNNING','ORPHANED')]
        if active:raise ValueError('stop the current localization/navigation session before changing runtime configuration')
        self.bridge()
        sensor_job,sensor=self._sensor_stack()
        if task=='sensors':
            if sensor_job:
                if sensor_job.get('robot')!=robot:raise ValueError('stop the old sensor stack before switching robot')
                return sensor_job
            if self._find_job('chassis_component') or any(j.get('kind')=='chassis_component' and j.get('state')=='ORPHANED' for j in self._jobs()):
                raise ValueError('stop the independent chassis before starting/replacing the sensor stack')
        else:
            sensor_job,sensor=self._sensor_stack(robot)
            if not sensor_job or not sensor:raise ValueError('start the matching independent sensor stack before localization/navigation')
            current=self.current()
            if current and current.get('task') not in ('sensors','manual'):
                raise ValueError('a localization/navigation session is already selected')
            if current and current.get('robot')!=robot:raise ValueError('sensor session robot differs from requested robot')
            if task=='navigation' and not self._find_job('chassis_component'):
                raise ValueError('start and verify the independent chassis component before navigation')
        if task in ('localization','navigation') and sensor:
            profile_source=Path(sensor['profile_snapshot']).resolve()
            if not profile_source.is_file() or hash_file(profile_source)!=sensor['profile']['profile_sha256']:
                raise ValueError('sensor session profile changed; restart sensors before localization/navigation')
            limits=profiles.load(profile_source,robot);selected=Path(sensor['calibration']).resolve()
            if not selected.is_file() or hash_file(selected)!=sensor['calibration_snapshot_sha256']:
                raise ValueError('sensor session calibration changed; restart sensors before localization/navigation')
        else:
            limits=profiles.load(profiles.selected_path(self.root,robot),robot);selected=None
        ident=_new_id();directory=self.root/'runs'/ident;directory.mkdir(parents=True)
        profile=directory/'robot.yaml';shutil.copyfile(limits['profile_path'],profile)
        checks=setup(p.get('checks')) if task=='navigation' else {}
        bundle=None;mount=None;cal_sha=None
        if task in ('localization','navigation') and sensor:
            bundle=calibration.load(selected,robot) if robot=='bunker' else None
            mount=load_profile(selected) if robot=='fwmini' else None
            cal_sha=calibration.digest(bundle) if bundle else digest(mount)
            if cal_sha!=sensor['calibration_sha256']:raise ValueError('selected calibration differs from the running sensor stack')
            calibration_source=sensor.get('calibration_source',str(selected))
        elif robot=='bunker':
            selected=Path(p.get('calibration_bundle') or self.root/'calibration/bunker/bunker_calibration_candidate.yaml').resolve()
            if not selected.is_relative_to((self.root/'calibration/bunker').resolve()):
                raise ValueError('Bunker calibration must be a workspace version')
            bundle=calibration.load(selected,robot);cal_sha=calibration.digest(bundle);calibration_source=str(selected)
        else:
            selected=resolve_profile(self.root,p.get('mount_profile'));mount=load_profile(selected);cal_sha=digest(mount);calibration_source=str(selected)
        copied=directory/'calibration.yaml';shutil.copyfile(selected,copied)
        map_record=Maps(self.root).get(p['map_id']) if p.get('map_id') else None
        if task in ('localization','navigation') and (not map_record or not map_record.get('source_pcd')):
            raise ValueError('map with source PCD required')
        acceptance=None
        if task=='navigation':
            if map_record['status']!='ACCEPTED':raise ValueError('formal navigation requires ACCEPTED map')
            if (bundle or mount)['status']!='VERIFIED':raise ValueError('formal navigation requires VERIFIED calibration')
            acceptance=report(p.get('safety_report',''),robot,limits['profile_sha256'],map_record['sha256'],cal_sha)
        sensor_enabled=bool(p.get('enable_sensor',True)) if task=='sensors' else False
        livox_identity=None
        if sensor_enabled:
            livox_source=Path(p.get('livox_config') or REPO/'agt_field_test_bringup/config/MID360s_config.json').expanduser().resolve()
            validate_mid360s_config(livox_source)
            livox_snapshot=directory/'livox_mid360s.json';shutil.copyfile(livox_source,livox_snapshot)
            livox_identity=dict(validate_mid360s_config(livox_snapshot),source_config_path=str(livox_source))
        elif sensor:livox_identity=sensor.get('livox')
        args=['robot:='+robot,'robot_profile_file:='+str(profile),
              'enable_chassis:=false','enable_localization:='+str(task in ('localization','navigation')).lower(),
              'enable_navigation:='+str(task=='navigation').lower(),'enable_mid360:='+str(sensor_enabled).lower(),
              'enable_description:='+str(task=='sensors').lower(),'enable_guard:=false',
              'external_chassis:='+str(task=='navigation').lower()]
        if sensor_enabled:args+=['livox_config:='+str(livox_snapshot)]
        if robot=='bunker':args+=['calibration_bundle:='+str(copied)]
        else:
            args+=['fwmini_mount_file:='+str(copied),
                   'fwmini_mode:='+('autonomous_navigation' if task=='navigation' else 'supervised_localization' if task=='localization' else 'sensor_test'),
                   'supervised:=true']
            geometry_file=p.get('geometry_file') or (sensor.get('geometry_file') if sensor else None)
            if geometry_file:args+=['fwmini_geometry_file:='+str(Path(geometry_file).resolve())]
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
        if task=='navigation':args+=['enable_inspection:=true','enable_camera_gimbal:=false']
        else:args+=['enable_inspection:=false','enable_camera_gimbal:=false']
        snapshot=dict(id=ident,robot=robot,task=task,profile=limits,profile_snapshot=str(profile),calibration=str(copied),
            calibration_source=calibration_source,calibration_sha256=cal_sha,
            calibration_snapshot_sha256=hash_file(copied),geometry_file=p.get('geometry_file') or (sensor.get('geometry_file') if sensor else None),
            checks=checks,map_id=map_record['id'] if map_record else None,
            map_sha256=map_record['sha256'] if map_record else None,
            map_yaml_sha256=hash_file(map_record['yaml']) if map_record else None,
            localization_pcd=map_record.get('source_pcd') if map_record else None,
            localization_pcd_sha256=map_record.get('source_pcd_sha256') if map_record else None,
            global_backend=backend,lio_backend=lio_backend,relocalization_assets=assets,livox=livox_identity,
            enable_camera_gimbal=False,camera_launch_owner='camera_gimbal_job',
            sensor_stack_job_id=sensor_job['id'] if sensor_job else None,
            chassis_job_id=(self._find_job('chassis_component') or {}).get('id'),
            safety_report=acceptance)
        write_json(directory/'session.json',snapshot)
        job_meta=dict(snapshot);job_meta['session_id']=job_meta.pop('id')
        if task=='navigation':self._stop_kind('control_guard')
        kind='sensor_stack' if task=='sensors' else 'field_session'
        try:job=self.jobs._new(kind,['bash',str(REPO/'scripts/run_field_test.sh'),*args],job_meta)
        except Exception:
            if task=='navigation' and sensor and self._find_job('chassis_component'):
                try:self._start_manual_guard(sensor)
                except Exception:pass
            raise
        write_json(self.root/'work/current_session.json',dict(snapshot,job_id=job['id']))
        return job

    def stop(self):
        current=self.current()
        if current and current.get('task')=='manual':return self.stop_chassis()
        if current and current.get('task')=='sensors' and self._find_job('chassis_component'):
            raise ValueError('stop the independent chassis first; sensor/TF ownership is still required')
        try:rpc(self.root,'stop')
        except (OSError,ValueError):pass
        stopped=[]
        if current and current.get('task') in ('localization','navigation'):
            stopped.extend(self._stop_kind('field_session'))
            self._restore_sensor_or_manual()
            return stopped
        if current and current.get('task')=='sensors':
            stopped.extend(self._stop_kind('sensor_stack'))
            (self.root/'work/current_session.json').unlink(missing_ok=True)
            return stopped
        stopped.extend(self._stop_kind('field_session'))
        return stopped

    def stop_all(self):
        """Stop every process owned by this Workbench, including recorder and bridge."""
        try:rpc(self.root,'stop')
        except (OSError,ValueError):pass
        self.jobs.stop_all()
        (self.root/'work/current_session.json').unlink(missing_ok=True)
        return self.jobs.list()
