"""Real ROS CLI observations and narrowly scoped child-process orchestration."""
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import threading
import time
import uuid

import yaml
from .fwmini import resolve_profile, load_profile, require_mode, digest

ROOT = Path(__file__).resolve().parents[3]
REPO = ROOT / 'src/agt_field_test_workbench'
ROS_EXEC = REPO / 'scripts/ros_exec.sh'
MAPPING = ROOT / 'src/agt-lio-pgo-mapping/scripts/run_mid360_mapping.sh'
ROS_SETUP = Path('/opt/ros/humble/setup.bash')
INSTALL = ROOT / 'install/setup.bash'

TEMPLATES = {
    'motion_diagnostic':['/cmd_vel_nav','/cmd_vel','/cmd_vel_smoothed','/agt/hmi/cmd_vel','/mux/cmd_vel',
        '/fwmini/cmd_vel_guarded','/ctrl_cmd','/chassis_info_fb','/fwmini/io_fb','/fwmini/control_status','/bunker_status',
        '/wheel/odom','/agt/odometry/local','/tf','/tf_static', '/field/map_pose', '/field/aligned/lidar', '/field/aligned/imu','/livox/lidar','/livox/imu','/agt/livox/points',
        '/agt/localization/status','/agt/localization/metrics','/agt/map_tracking/status','/agt/map_tracking/pose','/field/gps/candidate_status',
        '/local_costmap/costmap','/local_costmap/published_footprint','/plan','/ins/navsatfix','/ins/status','/ins/imu'],
    'calibration': ['/livox/lidar', '/livox/imu', '/tf', '/tf_static'],
    'mapping': ['/livox/lidar', '/livox/imu', '/tf', '/tf_static', '/field/map_pose', '/field/aligned/lidar', '/field/aligned/imu', '/wheel/odom'],
    'localization': ['/agt/odometry/local', '/agt/localization/status', '/agt/localization/metrics',
                     '/agt/map_tracking/status', '/agt/map_tracking/pose', '/tf', '/tf_static'],
    'chassis_motion': ['/wheel/odom', '/cmd_vel', '/mux/cmd_vel', '/ctrl_cmd', '/chassis_info_fb',
                       '/diagnostics', '/tf'],
    'full_field_test': ['/livox/lidar', '/livox/imu', '/wheel/odom', '/bunker_status',
                        '/agt/odometry/local', '/agt/localization/status', '/agt/localization/metrics',
                        '/agt/map_tracking/status', '/agt/global_relocalization/status',
                        '/agt/relocalization/status', '/cmd_vel_nav', '/cmd_vel', '/cmd_vel_smoothed',
                        '/agt/hmi/cmd_vel', '/mux/cmd_vel', '/ctrl_cmd', '/chassis_info_fb',
                        '/diagnostics', '/tf', '/tf_static'],
}

REQUIRED_TOPIC_TYPES = {
    'full_field_test': {'/livox/lidar': 'livox_ros_driver2/msg/CustomMsg'},
}


def sha256_file(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def utc_now():
    return datetime.now(timezone.utc).isoformat()


def write_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + '.tmp')
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n')
    temporary.replace(path)


def source_versions(root=ROOT):
    lock=yaml.safe_load((Path(root)/'src/agt_field_test_workbench/dependencies/field_test.repos').read_text())['repositories']
    result={}
    for name in ['agt_field_test_workbench',*lock]:
        path=Path(root)/'src'/name
        result[name]=subprocess.run(['git','-c','safe.directory='+str(path),'-C',str(path),'rev-parse','HEAD'],capture_output=True,text=True).stdout.strip() or 'UNAVAILABLE'
    return result


def ros_ready():
    return ROS_SETUP.is_file() and INSTALL.is_file()


def ros_call(args, timeout=5, domain=None):
    if not ros_ready():
        raise RuntimeError('ROS 2 Humble or unified install/setup.bash is unavailable')
    env = os.environ.copy()
    if domain is not None:
        env['ROS_DOMAIN_ID'] = str(domain)
    return subprocess.run(['bash', str(ROS_EXEC), *args], env=env, text=True,
                          capture_output=True, timeout=timeout, check=True).stdout


def topics():
    output = ros_call(['ros2', 'topic', 'list', '-t'], timeout=6)
    found = {}
    for line in output.splitlines():
        name, sep, type_part = line.partition(' [')
        if sep and type_part.endswith(']'):
            found[name] = type_part[:-1]
    return found


def validate_recording_topics(items):
    """Optional exact topic list; an omitted/empty list retains template behavior."""
    if items is None or items == []:return None
    if not isinstance(items,list) or len(items)>128:raise ValueError('custom topics must be a list of at most 128 names')
    result=[]
    for item in items:
        if not isinstance(item,str):raise ValueError('custom topic names must be strings')
        topic=item.strip()
        if len(topic)>255 or not re.fullmatch(r'/[A-Za-z0-9_]+(?:/[A-Za-z0-9_]+)*',topic):
            raise ValueError('invalid absolute ROS topic: '+topic[:80])
        if topic not in result:result.append(topic)
    return result or None


def status():
    result = {'observed_at': utc_now(), 'ros_domain_id': os.environ.get('ROS_DOMAIN_ID', '0'),
              'mode': 'LIVE_ROS' if ros_ready() else 'ROS_UNAVAILABLE', 'topics': {},
              'checks': {}, 'observations': {}}
    if ros_ready():
        try:
            result['topics'] = topics()
        except (subprocess.CalledProcessError, subprocess.TimeoutExpired, RuntimeError) as exc:
            result['error'] = str(exc)
        for name, topic in {'tracker': '/agt/map_tracking/status',
                            'manager': '/agt/relocalization/status',
                            'metrics': '/agt/localization/metrics'}.items():
            if topic not in result['topics']:
                continue
            try:
                sample = yaml.safe_load(ros_call(['ros2', 'topic', 'echo', '--once', topic], timeout=2))
                if isinstance(sample, dict) and isinstance(sample.get('data'), str):
                    try:
                        sample = json.loads(sample['data'])
                    except ValueError:
                        pass
                result['observations'][name] = sample
            except (subprocess.CalledProcessError, subprocess.TimeoutExpired,
                    RuntimeError, yaml.YAMLError):
                result['observations'][name] = {'state': 'NO_SAMPLE'}
    for name, required in {
        'sensor': ['/livox/lidar', '/livox/imu'],
        'local_odometry': ['/agt/odometry/local'],
        'localization': ['/agt/localization/status'],
        'tracking': ['/agt/map_tracking/status'],
        'chassis_feedback': ['/wheel/odom'],
        'nav2': ['/plan', '/local_costmap/costmap'],
    }.items():
        result['checks'][name] = {'state': 'OBSERVED' if all(t in result['topics'] for t in required)
                                   else 'NOT_OBSERVED', 'required_topics': required,
                                   'missing': [t for t in required if t not in result['topics']]}
    return result


class Jobs:
    def __init__(self, root=ROOT):
        self.root = Path(root)
        self.lock = threading.RLock()
        self.jobs = {}

    def _new(self, kind, command, meta, env=None):
        if set(meta)&{'id','kind','state','pid','log','command','started_at'}:raise ValueError('reserved job metadata field')
        with self.lock:
            history = self.list()
            if any(item['state'] == 'ORPHANED' and
                   (item['kind'] == kind or
                    (kind.startswith('mapping') and item['kind'].startswith('mapping')))
                   for item in history):
                raise ValueError('an orphaned prior job may still own the resource; inspect its PID and log')
            if any(item['kind'] == kind and item['state'] == 'RUNNING' for item in self.jobs.values()):
                raise ValueError(f'{kind} already running')
            if kind.startswith('mapping') and any(item['kind'].startswith('mapping') and
                                                  item['state'] == 'RUNNING' for item in self.jobs.values()):
                raise ValueError('another mapping session already owns ROS Domain 89')
            if kind == 'recorder' and any(item['kind'] == 'mapping_live' and item['state'] == 'RUNNING'
                                          for item in self.jobs.values()):
                raise ValueError('live mapping already owns raw rosbag recording')
            if kind == 'mapping_live' and any(item['kind'] == 'recorder' and item['state'] == 'RUNNING'
                                              for item in self.jobs.values()):
                raise ValueError('stop recorder before live mapping to avoid duplicate raw bag')
            run_id = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ') + '-' + uuid.uuid4().hex[:8]
            directory = self.root / 'runs' / run_id
            directory.mkdir(parents=True, exist_ok=False)
            log = (directory / 'process.log').open('wb')
            child_env = os.environ.copy()
            child_env.update(env or {})
            try:
                process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                           start_new_session=True, env=child_env, cwd=self.root)
            finally:
                log.close()
            record = {'id': run_id, 'kind': kind, 'state': 'RUNNING', 'started_at': utc_now(),
                      'pid': process.pid, 'log': str(directory / 'process.log'),
                      'command': command, **meta}
            if kind == 'recorder' and 'configuration_snapshot' in record:
                configuration_path = directory / 'recording_config.json'
                write_json(configuration_path, record['configuration_snapshot'])
                record['configuration_path'] = str(configuration_path)
                record['configuration_sha256'] = sha256_file(configuration_path)
            self.jobs[run_id] = {'process': process, 'record': record, 'kind': kind, 'state': 'RUNNING'}
            write_json(directory / 'run.json', record)
            return record

    def list(self):
        with self.lock:
            result = []
            for item in self.jobs.values():
                process, record = item['process'], item['record']
                code = process.poll()
                if code is not None and record['state'] == 'RUNNING':
                    record['state'] = 'PASS' if code == 0 else 'FAILED'
                    record['exit_code'] = code
                    record['ended_at'] = utc_now()
                    if item['kind'] == 'recorder':
                        record['integrity'] = check_bag(Path(record['output']), record.get('topics'))
                        if record['integrity']['state'] != 'PASS':
                            record['state'] = 'INCOMPLETE'
                    elif item['kind'].startswith('mapping'):
                        result_path = Path(record['output']) / 'field_test_result.json'
                        if result_path.is_file():
                            record['result'] = json.loads(result_path.read_text())
                    write_json(self.root / 'runs' / record['id'] / 'run.json', record)
                item['state'] = record['state']
                current = dict(record, duration_sec=round(time.time() - process_start(record), 1)
                                   if record['state'] == 'RUNNING' else None)
                if item['kind'].startswith('mapping') and record.get('output'):
                    stage_path = Path(record['output']) / 'save_status.json'
                    if stage_path.is_file():
                        try: current['save_status'] = json.loads(stage_path.read_text())
                        except (OSError, ValueError): pass
                result.append(current)
            for manifest in sorted((self.root / 'runs').glob('*/run.json'), reverse=True):
                if manifest.parent.name in self.jobs:
                    continue
                try:
                    record = json.loads(manifest.read_text())
                except (OSError, ValueError):
                    continue
                if record.get('state') == 'RUNNING':
                    pid = record.get('pid')
                    try:
                        os.kill(pid, 0)
                        state = 'ORPHANED'
                    except (OSError, TypeError):
                        state = 'INCOMPLETE'
                    record['state'] = state
                    record['reason'] = 'web server restarted before this job reached a terminal state'
                    write_json(manifest, record)
                result.append(record)
            return sorted(result, key=lambda record: record['id'], reverse=True)

    def finish_mapping(self, run_id):
        with self.lock:
            item=self.jobs.get(run_id)
            if not item or item['kind']!='mapping_live' or item['process'].poll() is not None:
                raise ValueError('running live mapping required')
            os.kill(item['process'].pid,signal.SIGUSR1)
            item['record']['save_requested_at']=utc_now()
            write_json(self.root/'runs'/run_id/'run.json',item['record'])
            return dict(item['record'],finishing=True)

    def stop_all(self):
        for item in self.list():
            if item['id'] in self.jobs and item['state'] == 'RUNNING':
                self.stop(item['id'])

    def stop(self, run_id):
        with self.lock:
            item = self.jobs.get(run_id)
            if not item:
                raise ValueError('unknown job')
            process, record = item['process'], item['record']
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=12)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGTERM)
                    process.wait(timeout=5)
            record['state'] = 'INCOMPLETE' if item['kind'] == 'recorder' else 'CANCELLED'
            if item['kind'] == 'recorder':
                record['integrity'] = check_bag(Path(record['output']), record.get('topics'))
                if record['integrity']['state'] == 'PASS':
                    record['state'] = 'PASS'
            record['ended_at'] = utc_now()
            record['exit_code'] = process.returncode
            item['state'] = record['state']
            write_json(self.root / 'runs' / run_id / 'run.json', record)
            return record

    def start_mapping(self, bag, robot, calibration=None, live=False, livox_config=None, mount_profile=None, backend="fastlio", calibration_bundle=None, map_name=''):
        if not ros_ready():
            raise ValueError('ROS Humble unified overlay not built')
        if robot not in ('bunker', 'fwmini'):
            raise ValueError('unknown robot')
        if backend not in ('fastlio','pgo_experiment'):raise ValueError('unknown mapping backend')
        if live and backend=='pgo_experiment':raise ValueError('PGO experiment is offline-only; use FAST-LIO for live mapping')
        source = Path(bag).expanduser().resolve() if bag else None
        if not live and (source is None or not (source / 'metadata.yaml').is_file()):
            raise ValueError('existing rosbag2 directory with metadata.yaml required')
        if calibration:
            config = calibration_config(self.root, robot, calibration)
            calibration_sha = json.loads((self.root / 'calibration' / robot /
                                          (calibration + '.json')).read_text())['sha256']
        else:
            config = None
            calibration_sha = None
        session_path=self.root/'work/current_session.json'
        current=json.loads(session_path.read_text()) if session_path.is_file() else None
        live_session=next((j for j in self.list() if j['kind']=='field_session' and j['state']=='RUNNING'),None)
        if live and not live_session:raise ValueError('start a sensors/manual session in Robot Setup before live mapping')
        if live and live_session and (current['robot']!=robot or current['task'] in ('localization','navigation')):
            raise ValueError('live mapping needs the same robot in sensors/manual mode; stop localization first')
        run_id = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ') + '-' + uuid.uuid4().hex[:8]
        output = self.root / 'maps' / run_id
        cmd = ['python3', str(REPO / 'scripts/mapping_pipeline.py'),
               '--robot', robot, '--output', str(output), '--backend', backend,
               '--map-name', str(map_name or '').strip()[:120]]
        mount_metadata = None
        if robot == 'fwmini':
            mount = resolve_profile(self.root, mount_profile)
            profile = load_profile(mount)
            require_mode(profile, 'sensor_mapping' if live else 'offline_mapping')
            if live and live_session and digest(profile)!=current['calibration_sha256']:raise ValueError('FW-mini mount differs from running TF')
            cmd += ['--mount-profile', str(mount)]
            mount_metadata = {'path': str(mount), 'calibration_sha256': digest(profile), 'status': profile['status']}
        if live:
            if not (live_session and live) and (not livox_config or not Path(livox_config).is_file()):
                raise ValueError('live sensor-only mapping requires an existing Livox IP config for this robot')
            # This upstream profile starts MID360 only. It does not start either
            # chassis, avoiding the legacy Bunker hardware launch dependency.
            cmd += ['--live', '--livox-config', str(Path(livox_config).resolve()) if livox_config else '']
        else:
            cmd += ['--bag', str(source)]
        if calibration:
            cmd += ['--calibration', calibration]
        env = {'AGT_FIELD_TEST_ROBOT': robot + '_sensor_only'} if live else {}
        if live and live_session:
            env.update(FIELD_REUSE_RUNTIME='1',FIELD_MAPPING_DOMAIN=os.environ.get('ROS_DOMAIN_ID','0'),ROS_DOMAIN_ID=os.environ.get('ROS_DOMAIN_ID','0'))
        if robot=='bunker':
            from agt_field_test_bringup import calibration as bundle_cal
            bundle_path=Path(calibration_bundle or self.root/'calibration/bunker/bunker_calibration_candidate.yaml').resolve()
            if not bundle_path.is_relative_to((self.root/'calibration/bunker').resolve()):raise ValueError('workspace Bunker calibration required')
            bundle=bundle_cal.load(bundle_path,robot)
            if live and live_session and bundle_cal.digest(bundle)!=current['calibration_sha256']:raise ValueError('live mapping calibration differs from active TF; restart session')
            parameters=yaml.safe_load((self.root/'src/agt-lio-pgo-mapping/bringup/agt_mapping_bringup/config/fastlio2_mid360.yaml').read_text())
            parameters.update(r_il=bundle['internal']['fastlio2']['rotation'],t_il=bundle['internal']['fastlio2']['translation'],esti_il=False)
            if config:
                override=yaml.safe_load(config.read_text())
                if live and live_session and (override['r_il']!=parameters['r_il'] or override['t_il']!=parameters['t_il']):raise ValueError('internal calibration differs from running TF')
                parameters=override
            folder=self.root/'runs'/('mapping_config_'+run_id);folder.mkdir(parents=True)
            config=folder/'fastlio.yaml';config.write_text(yaml.safe_dump(parameters,sort_keys=False))
            calibration_sha=bundle_cal.digest(bundle)
            env.update(FIELD_CALIBRATION_BUNDLE=str(bundle_path),FIELD_IMU_OFFSET=str(bundle['time_sync']['imu_offset_sec']),FIELD_LIDAR_OFFSET=str(bundle['time_sync']['lidar_offset_sec']))
        if config:
            env['AGT_FIELD_TEST_FASTLIO_CONFIG'] = str(config)
        return self._new('mapping_live' if live else 'mapping_bag', cmd,
                         {'robot': robot, 'bag': str(source) if source else None,
                          'output': str(output), 'calibration': calibration,
                          'calibration_sha256': calibration_sha,
                          'mount_profile': mount_metadata,
                          'versions': source_versions(self.root),
                          'ros_domain_id': int(env.get('FIELD_MAPPING_DOMAIN','89')), 'sensor_only': live and not bool(live_session), 'reuse_runtime':bool(live and live_session), 'backend':backend,
                          'raw_bag': str(output / 'raw_bag') if live else None,
                          'map_name': str(map_name or '').strip()[:120]}, env)

    def start_recorder(self, template, robot, calibration=None, map_version=None, label='', mount_profile=None, requested_topics=None):
        if template not in TEMPLATES or robot not in ('bunker', 'fwmini'):
            raise ValueError('unknown template or robot')
        custom_topics=validate_recording_topics(requested_topics)
        if not ros_ready():
            raise ValueError('ROS Humble unified overlay not built')
        mount_metadata = None
        if robot == 'fwmini':
            mount = resolve_profile(self.root, mount_profile)
            profile = load_profile(mount)
            require_mode(profile, 'calibration_recording')
            # Recorder observes an already running sensor profile; never adds a
            # second RSP to a live robot's domain.
            ros_call(['ros2', 'run', 'agt_field_test_bringup', 'fwmini_tf_contract.py',
                      '--profile', str(mount), '--preflight'], timeout=13)
            mount_metadata = {'path': str(mount), 'calibration_sha256': digest(profile), 'status': profile['status']}
        free = shutil.disk_usage(self.root).free
        if free < 2 * 1024**3:
            raise ValueError(f'only {free // 1024**3} GiB free; require at least 2 GiB')
        actual = topics()
        desired = custom_topics if custom_topics is not None else [
            t for t in TEMPLATES[template] if robot == 'fwmini' or t not in ('/ctrl_cmd', '/chassis_info_fb')]
        selected = [t for t in desired if t in actual]
        if not selected:
            raise ValueError('none of the requested topics currently exists' if custom_topics is not None
                             else 'none of the template topics currently exists')
        required_types = {name: typ for name, typ in REQUIRED_TOPIC_TYPES.get(template, {}).items()
                          if name in desired}
        type_mismatches = {name: {'expected': typ, 'observed': actual.get(name)}
                           for name, typ in required_types.items()
                           if name in actual and actual[name] != typ}
        if type_mismatches:
            raise ValueError('required topic type mismatch: ' + json.dumps(type_mismatches, sort_keys=True))
        missing_topics = [t for t in desired if t not in actual]
        run_id = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ') + '-' + uuid.uuid4().hex[:8]
        output = self.root / 'bags' / run_id
        qos_path = self.root / 'config/recording/qos_overrides.yaml'
        command = ['bash', str(ROS_EXEC), 'ros2', 'bag', 'record', '-o', str(output),
                   '--qos-profile-overrides-path', str(qos_path), *selected]
        versions = source_versions(self.root)
        qos = {}
        for topic in selected:
            try:
                qos[topic] = ros_call(['ros2', 'topic', 'info', '--verbose', topic], timeout=3)
            except (subprocess.CalledProcessError, subprocess.TimeoutExpired):
                qos[topic] = 'UNAVAILABLE'
        snapshot = {'created_at': utc_now(), 'template': template, 'robot': robot,
                    'topic_source': 'custom' if custom_topics is not None else 'template',
                    'requested_topics': list(desired), 'selected_topics': selected,
                    'topic_types': {t: actual[t] for t in selected},
                    'required_topic_types': required_types, 'type_mismatches': type_mismatches,
                    'missing_topics': missing_topics, 'offered_qos': qos,
                    'qos_overrides_path': str(qos_path),
                    'qos_overrides_sha256': sha256_file(qos_path) if qos_path.is_file() else None,
                    'calibration': calibration, 'map_version': map_version,
                    'mount_profile': mount_metadata, 'label': label[:100], 'versions': versions,
                    'ros_domain_id': os.environ.get('ROS_DOMAIN_ID', '0')}
        return self._new('recorder', command, {'template': template, 'robot': robot,
                         'topic_source': snapshot['topic_source'], 'output': str(output),
                         'topics': snapshot['topic_types'], 'offered_qos': qos,
                         'requested_topics': list(desired), 'missing_topics': missing_topics,
                         'required_topic_types': required_types, 'type_mismatches': type_mismatches,
                         'calibration': calibration, 'map_version': map_version,
                         'mount_profile': mount_metadata, 'configuration_snapshot': snapshot,
                         'label': label[:100], 'versions': versions,
                         'ros_domain_id': snapshot['ros_domain_id']})


def process_start(record):
    return datetime.fromisoformat(record['started_at']).timestamp()


def _bag_storage_files(root, entries):
    """Normalize Humble string entries and supported {path: ...} mappings."""
    if not isinstance(entries, list) or not entries:
        raise ValueError('relative_file_paths must be a non-empty list')
    root = root.resolve()
    result = []
    seen = set()
    for item in entries:
        relative = item if isinstance(item, str) else item.get('path') if isinstance(item, dict) else None
        if not isinstance(relative, str) or not relative.strip():
            raise ValueError('unsupported relative_file_paths entry')
        relative = relative.strip()
        candidate = Path(relative)
        if candidate.is_absolute():
            raise ValueError('absolute bag storage path is not allowed')
        resolved = (root / candidate).resolve()
        try:
            resolved.relative_to(root)
        except ValueError as exc:
            raise ValueError('bag storage path escapes the bag directory') from exc
        normalized = resolved.relative_to(root).as_posix()
        if normalized in seen:
            raise ValueError('duplicate bag storage path: ' + normalized)
        seen.add(normalized)
        result.append((normalized, resolved))
    return result


def check_bag(path, expected_types=None):
    path = Path(path).expanduser()
    metadata = path / 'metadata.yaml'
    if not metadata.is_file():
        return {'state': 'INCOMPLETE', 'reason': 'metadata.yaml missing'}
    evidence = {}
    try:
        metadata_bytes = metadata.read_bytes()
        evidence = {'metadata_sha256': hashlib.sha256(metadata_bytes).hexdigest()}
        document = yaml.safe_load(metadata_bytes.decode('utf-8'))
        if not isinstance(document, dict):
            raise ValueError('metadata.yaml must contain a mapping')
        info = document['rosbag2_bagfile_information']
        if not isinstance(info, dict):
            raise ValueError('rosbag2_bagfile_information must be a mapping')
        entries = info.get('topics_with_message_count', [])
        if not isinstance(entries, list):
            raise ValueError('topics_with_message_count must be a list')
        counts, types = {}, {}
        for entry in entries:
            topic = entry['topic_metadata']
            name, typ = topic['name'], topic['type']
            if not isinstance(name, str) or not name or not isinstance(typ, str) or not typ:
                raise ValueError('invalid topic metadata')
            if name in counts:
                raise ValueError('duplicate topic metadata: ' + name)
            message_count = int(entry['message_count'])
            if message_count < 0:
                raise ValueError('negative message count: ' + name)
            counts[name], types[name] = message_count, typ
        count = sum(counts.values())
        storage = _bag_storage_files(path, info.get('relative_file_paths'))
        files = []
        for relative, storage_path in storage:
            item = {'path': relative, 'sha256': None, 'size_bytes': 0}
            if storage_path.is_file():
                item['size_bytes'] = storage_path.stat().st_size
                if item['size_bytes'] > 0:
                    item['sha256'] = sha256_file(storage_path)
            files.append(item)
        evidence.update(counts=counts, types=types, messages=count, files=files,
                        data_sha256={item['path']: item['sha256'] for item in files})
        if not count:
            return dict(evidence, state='INCOMPLETE', reason='zero messages')
        missing = [item['path'] for item in files if item['sha256'] is None]
        if missing:
            return dict(evidence, state='INCOMPLETE', reason='missing or empty bag storage files',
                        missing_or_empty_files=missing)
        mismatched = {name: {'expected': typ, 'recorded': types.get(name)}
                      for name, typ in (expected_types or {}).items() if types.get(name) != typ}
        if mismatched:
            return dict(evidence, state='INCOMPLETE', reason='topic types missing or changed',
                        type_mismatches=mismatched)
        return dict(evidence, state='PASS')
    except (KeyError, ValueError, TypeError, UnicodeError, OSError, yaml.YAMLError) as exc:
        return dict(evidence, state='INCOMPLETE', reason=str(exc))


def calibration_config(root, robot, version):
    if not version or any(c not in 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789-_' for c in version):
        raise ValueError('invalid calibration version')
    path = root / 'calibration' / robot / (version + '.json')
    if not path.is_file():
        raise ValueError('calibration version not found')
    data = json.loads(path.read_text())
    if data.get('robot') != robot:
        raise ValueError('calibration robot mismatch')
    digest = hashlib.sha256(json.dumps({key: value for key, value in data.items() if key != 'sha256'},
                                    sort_keys=True).encode()).hexdigest()
    if digest != data.get('sha256'):
        raise ValueError('calibration SHA-256 mismatch')
    source = root / 'src/agt-lio-pgo-mapping/bringup/agt_mapping_bringup/config/fastlio2_mid360.yaml'
    params = yaml.safe_load(source.read_text())
    params['r_il'] = data['internal']['rotation_row_major']
    params['t_il'] = data['internal']['translation_m']
    if len(params['r_il']) != 9 or len(params['t_il']) != 3:
        raise ValueError('invalid LiDAR/IMU extrinsics')
    if any(not isinstance(value, (int, float)) or not math.isfinite(value)
           for value in params['r_il'] + params['t_il']):
        raise ValueError('non-finite calibration parameter')
    target = root / 'calibration' / robot / (version + '.fastlio.yaml')
    content = yaml.safe_dump(params, sort_keys=False)
    if target.exists() and target.read_text() != content:
        raise ValueError('existing derived FAST-LIO2 config differs; preserved')
    if not target.exists():
        target.write_text(content)
    return target
