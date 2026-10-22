"""Real ROS CLI observations and narrowly scoped child-process orchestration."""
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import threading
import time
import uuid

import yaml

ROOT = Path(__file__).resolve().parents[3]
REPO = ROOT / 'src/agt_field_test_workbench'
ROS_EXEC = REPO / 'scripts/ros_exec.sh'
MAPPING = ROOT / 'src/agt-lio-pgo-mapping/scripts/run_mid360_mapping.sh'
ROS_SETUP = Path('/opt/ros/humble/setup.bash')
INSTALL = ROOT / 'install/setup.bash'

TEMPLATES = {
    'calibration': ['/livox/lidar', '/livox/imu', '/tf', '/tf_static'],
    'mapping': ['/livox/lidar', '/livox/imu', '/tf', '/tf_static', '/wheel/odom'],
    'localization': ['/agt/odometry/local', '/agt/localization/status', '/agt/localization/metrics',
                     '/agt/map_tracking/status', '/agt/map_tracking/pose', '/tf', '/tf_static'],
    'chassis_motion': ['/wheel/odom', '/cmd_vel', '/mux/cmd_vel', '/ctrl_cmd', '/chassis_info_fb',
                       '/diagnostics', '/tf'],
    'full_field_test': ['/livox/lidar', '/livox/imu', '/wheel/odom', '/agt/odometry/local',
                        '/agt/localization/status', '/agt/localization/metrics', '/agt/map_tracking/status',
                        '/cmd_vel', '/mux/cmd_vel', '/ctrl_cmd', '/chassis_info_fb',
                        '/diagnostics', '/tf', '/tf_static'],
}


def utc_now():
    return datetime.now(timezone.utc).isoformat()


def write_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + '.tmp')
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n')
    temporary.replace(path)


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
        with self.lock:
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
                    write_json(self.root / 'runs' / record['id'] / 'run.json', record)
                item['state'] = record['state']
                result.append(dict(record, duration_sec=round(time.time() - process_start(record), 1)
                                   if record['state'] == 'RUNNING' else None))
            return result

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

    def start_mapping(self, bag, robot, calibration=None, live=False, livox_config=None):
        if not ros_ready():
            raise ValueError('ROS Humble unified overlay not built')
        if robot not in ('bunker', 'fwmini'):
            raise ValueError('unknown robot')
        source = Path(bag).expanduser().resolve() if bag else None
        if not live and (source is None or not (source / 'metadata.yaml').is_file()):
            raise ValueError('existing rosbag2 directory with metadata.yaml required')
        if calibration:
            config = calibration_config(self.root, robot, calibration)
        else:
            config = None
        run_id = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ') + '-' + uuid.uuid4().hex[:8]
        output = self.root / 'maps' / run_id
        cmd = ['python3', str(REPO / 'scripts/mapping_pipeline.py'),
               '--robot', robot, '--output', str(output)]
        if live:
            if not livox_config or not Path(livox_config).is_file():
                raise ValueError('live sensor-only mapping requires an existing Livox IP config for this robot')
            # This upstream profile starts MID360 only. It does not start either
            # chassis, avoiding the legacy Bunker hardware launch dependency.
            cmd += ['--live', '--livox-config', str(Path(livox_config).resolve())]
        else:
            cmd += ['--bag', str(source)]
        if calibration:
            cmd += ['--calibration', calibration]
        env = {'AGT_FIELD_TEST_ROBOT': robot + '_sensor_only'} if live else {}
        if config:
            env['AGT_FIELD_TEST_FASTLIO_CONFIG'] = str(config)
        return self._new('mapping_live' if live else 'mapping_bag', cmd,
                         {'robot': robot, 'bag': str(source) if source else None,
                          'output': str(output), 'calibration': calibration,
                          'ros_domain_id': 89, 'sensor_only': live,
                          'raw_bag': str(output / 'raw_bag') if live else None}, env)

    def start_recorder(self, template, robot, calibration=None, map_version=None, label=''):
        if template not in TEMPLATES or robot not in ('bunker', 'fwmini'):
            raise ValueError('unknown template or robot')
        if not ros_ready():
            raise ValueError('ROS Humble unified overlay not built')
        free = shutil.disk_usage(self.root).free
        if free < 2 * 1024**3:
            raise ValueError(f'only {free // 1024**3} GiB free; require at least 2 GiB')
        actual = topics()
        desired = [t for t in TEMPLATES[template] if robot == 'fwmini' or t not in ('/ctrl_cmd', '/chassis_info_fb')]
        selected = [t for t in desired if t in actual]
        if not selected:
            raise ValueError('none of the template topics currently exists')
        run_id = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ') + '-' + uuid.uuid4().hex[:8]
        output = self.root / 'bags' / run_id
        command = ['bash', str(ROS_EXEC), 'ros2', 'bag', 'record', '-o', str(output),
                   '--qos-profile-overrides-path', str(self.root / 'config/recording/qos_overrides.yaml'),
                   *selected]
        versions = {p.name: subprocess.run(['git', '-C', str(p), 'rev-parse', 'HEAD'],
                                           capture_output=True, text=True).stdout.strip()
                    for p in (self.root / 'src').iterdir() if (p / '.git').exists()}
        qos = {}
        for topic in selected:
            try:
                qos[topic] = ros_call(['ros2', 'topic', 'info', '--verbose', topic], timeout=3)
            except (subprocess.CalledProcessError, subprocess.TimeoutExpired):
                qos[topic] = 'UNAVAILABLE'
        return self._new('recorder', command, {'template': template, 'robot': robot,
                         'output': str(output), 'topics': {t: actual[t] for t in selected},
                         'offered_qos': qos,
                         'missing_topics': [t for t in desired if t not in actual],
                         'calibration': calibration, 'map_version': map_version,
                         'label': label[:100], 'versions': versions,
                         'ros_domain_id': os.environ.get('ROS_DOMAIN_ID', '0')})


def process_start(record):
    return datetime.fromisoformat(record['started_at']).timestamp()


def check_bag(path, expected_types=None):
    metadata = path / 'metadata.yaml'
    if not metadata.is_file():
        return {'state': 'INCOMPLETE', 'reason': 'metadata.yaml missing'}
    try:
        info = yaml.safe_load(metadata.read_text())['rosbag2_bagfile_information']
        entries = info.get('topics_with_message_count', [])
        counts = {entry['topic_metadata']['name']: int(entry['message_count']) for entry in entries}
        types = {entry['topic_metadata']['name']: entry['topic_metadata']['type'] for entry in entries}
        count = sum(counts.values())
        files = [path / item['path'] for item in info.get('relative_file_paths', [])]
        if not count or not files or not all(p.is_file() for p in files):
            return {'state': 'INCOMPLETE', 'reason': 'zero messages or missing bag storage files',
                    'counts': counts}
        mismatched = {name: {'expected': typ, 'recorded': types.get(name)}
                      for name, typ in (expected_types or {}).items() if types.get(name) != typ}
        if mismatched:
            return {'state': 'INCOMPLETE', 'reason': 'topic types missing or changed',
                    'type_mismatches': mismatched, 'counts': counts}
        return {'state': 'PASS', 'counts': counts, 'types': types, 'messages': count}
    except (KeyError, ValueError, TypeError, yaml.YAMLError) as exc:
        return {'state': 'INCOMPLETE', 'reason': str(exc)}


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
