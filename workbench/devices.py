"""Independent, supervised C1 camera/gimbal and capture-session jobs."""
import json
import math
import re
from pathlib import Path

import yaml

from .runtime import ROS_EXEC, REPO, ROOT, ros_ready, ros_call, write_json
from .maps import Maps, _new_id


def _device(value, kind):
    if not isinstance(value, str) or not value or len(value) > 240:
        raise ValueError('invalid ' + kind + ' device path')
    path = Path(value)
    permitted = ('/dev/video', '/dev/v4l/by-id/') if kind == 'camera' else ('/dev/tty', '/dev/serial/by-id/')
    if not path.is_absolute() or not any(value.startswith(prefix) for prefix in permitted):
        raise ValueError(kind + ' must be a configured /dev camera or serial device')
    if not path.exists():
        raise ValueError(kind + ' device not connected: ' + value)
    return value


def _health():
    try:
        value = yaml.safe_load(ros_call(['ros2', 'topic', 'echo', '--once', '/camera_gimbal/health'], timeout=2.5))
    except Exception as exc:
        raise ValueError('camera capability health unavailable: ' + str(exc)[:160]) from exc
    if not isinstance(value, dict):
        raise ValueError('camera capability returned an invalid health message')
    names = {0:'NOT_READY',1:'READY',2:'BUSY',3:'DEGRADED',4:'ERROR'}
    value['state_name'] = names.get(value.get('state'), 'UNKNOWN')
    return value


def start_camera(jobs, request):
    if not ros_ready():
        raise ValueError('ROS 2 Humble workspace unavailable')
    if not isinstance(request, dict):
        raise ValueError('camera configuration required')
    camera = _device(request.get('device_path', '/dev/video0'), 'camera')
    gimbal = _device(request.get('port_name', ''), 'gimbal')
    current = jobs.list()
    if any(job['kind'] == 'camera_gimbal' and job['state'] in ('RUNNING', 'ORPHANED') for job in current):
        raise ValueError('camera already owned by this workbench (or orphaned); inspect job')
    if any(job['kind'] == 'field_session' and job['state'] == 'RUNNING'
           and job.get('enable_camera_gimbal') for job in current):
        raise ValueError('legacy navigation session already owns camera')
    for package in ('autolabor_c1_bringup', 'camera_gimbal_interfaces',
                    'camera_gimbal_capability', 'pantilt_camera_serial'):
        try:
            ros_call(['ros2', 'pkg', 'prefix', package], timeout=4)
        except Exception as exc:
            raise ValueError('required C1 ROS package is not installed/sourced: ' + package) from exc
    session_id = _new_id()
    capture_root = (Path(ROOT) / 'data' / 'captures' / session_id).resolve()
    capture_root.mkdir(parents=True, exist_ok=False)
    return jobs._new('camera_gimbal',
        ['bash', str(ROS_EXEC), 'ros2', 'launch', 'autolabor_c1_bringup',
         'autolabor_c1.launch.py', 'device_path:='+camera, 'port_name:='+gimbal,
         'capture_output_root:='+str(capture_root), 'gui:=false'],
        {'device_path':camera, 'port_name':gimbal, 'capture_root':str(capture_root),
         'capture_session_id':session_id,
         'readiness':'WAITING_FOR_CAMERA_GIMBAL_HEALTH'})


def stop_camera(jobs):
    # Stop one-shot clients first so a camera shutdown cannot strand an action request.
    for job in jobs.list():
        if job['kind'] == 'camera_capture' and job['state'] == 'RUNNING':
            jobs.stop(job['id'])
    active = [job for job in jobs.list()
              if job['kind'] == 'camera_gimbal' and job['state'] == 'RUNNING']
    if not active:
        return {'state':'STOPPED', 'detail':'no managed camera process running'}
    return jobs.stop(active[0]['id'])


def camera_status(jobs, probe=False):
    camera = next((j for j in jobs.list() if j['kind'] == 'camera_gimbal'), None)
    if camera is None:
        return {'state':'STOPPED','readiness':'UNVERIFIED','capture_root':None}
    result = {'state':camera['state'], 'job_id':camera['id'],
              'log':camera.get('log'), 'readiness':camera.get('readiness','UNVERIFIED'),
              'device_path':camera.get('device_path'), 'port_name':camera.get('port_name'),
              'capture_root':camera.get('capture_root'),
              'capture_session_id':camera.get('capture_session_id')}
    if probe and camera['state'] == 'RUNNING':
        try:
            health = _health()
            result['health'] = health
            result['readiness'] = health['state_name']
            result['ready'] = bool(health.get('state') == 1 and health.get('camera_alive')
                and health.get('gimbal_serial_connected') and health.get('gimbal_feedback_alive')
                and health.get('move_action_ready') and not health.get('busy'))
        except ValueError as exc:
            result.update(readiness='HEALTH_UNAVAILABLE', ready=False, health_error=str(exc))
    else:
        result['ready'] = False
    return result


def _finite_angle(request, name, low, high):
    value = request.get(name, 0.0)
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(name + ' must be finite degrees')
    if value < low or value > high:
        raise ValueError(f'{name} must be in [{low}, {high}] degrees')
    return float(value)


def start_capture(jobs, request, save_image=True):
    if not isinstance(request, dict):
        raise ValueError('camera request must be an object')
    camera = next((j for j in jobs.list() if j['kind'] == 'camera_gimbal'), None)
    if not camera or camera['state'] != 'RUNNING':
        raise ValueError('start the independent C1 camera/gimbal first')
    if any(j['kind']=='camera_capture' and j['state']=='RUNNING' for j in jobs.list()):
        raise ValueError('another gimbal/capture action is still running')
    health = _health()
    if not (health.get('state') == 1 and health.get('camera_alive')
            and health.get('gimbal_serial_connected') and health.get('gimbal_feedback_alive')
            and health.get('move_action_ready') and not health.get('busy')):
        raise ValueError('C1 is not READY: ' + str(health.get('state_name')) + ' · ' +
                         str(health.get('last_error','check camera image, serial and gimbal feedback'))[:160])
    heading = _finite_angle(request, 'heading', -160.0, 160.0)
    pitch = _finite_angle(request, 'pitch', -90.0, 90.0)
    roll = _finite_angle(request, 'roll', -40.0, 40.0)
    tag = request.get('tag') or ('capture_' + _new_id().replace('-', '_'))
    if not isinstance(tag, str) or not re.fullmatch(r'[A-Za-z0-9_.-]{1,64}', tag):
        raise ValueError('tag must contain 1..64 letters, digits, dot, underscore or hyphen')
    metadata = request.get('metadata') or {}
    if not isinstance(metadata, dict) or len(metadata) > 24:
        raise ValueError('capture metadata must be a small JSON object')
    pose = metadata.get('pose')
    if pose is not None:
        if not isinstance(pose, dict) or set(pose)-{'x','y','yaw'}:
            raise ValueError('pose metadata must contain x/y/yaw only')
        if any(isinstance(pose.get(k),bool) or not isinstance(pose.get(k),(int,float))
               or not math.isfinite(pose[k]) for k in ('x','y','yaw')):
            raise ValueError('pose metadata x/y/yaw must be finite')
    map_id = metadata.get('map_version')
    if map_id:
        Maps(ROOT).get(map_id)
    capture_root = Path(camera.get('capture_root','')).resolve()
    allowed_root = (Path(ROOT)/'data'/'captures').resolve()
    if not capture_root.is_relative_to(allowed_root):
        raise ValueError('camera capture root is outside workspace data/captures')
    request_id = _new_id()
    request_dir = Path(ROOT)/'runs'/('camera_request_'+request_id)
    request_dir.mkdir(parents=True, exist_ok=False)
    request_path = request_dir/'request.json'
    result_path = request_dir/'result.json'
    payload = {'heading':heading, 'pitch':pitch, 'roll':roll, 'tag':tag,
               'save_image':bool(save_image), 'metadata':metadata,
               'camera_session_id':camera.get('capture_session_id'),
               'requested_at':__import__('datetime').datetime.now(__import__('datetime').timezone.utc).isoformat()}
    write_json(request_path,payload)
    command = ['bash',str(ROS_EXEC),'python3',str(REPO/'scripts/camera_capture.py'),
               '--request',str(request_path),'--result',str(result_path),
               '--capture-root',str(capture_root)]
    return jobs._new('camera_capture', command,
        {'capture_id':request_id,'capture_root':str(capture_root),'request_path':str(request_path),
         'result_path':str(result_path),'requested_angles':{'heading':heading,'pitch':pitch,'roll':roll},
         'save_image':bool(save_image),'tag':tag,'map_version':map_id,
         'scope':'C1_GIMBAL_ACTION_NO_BASE_MOTION'})


def list_captures(jobs):
    camera = next((j for j in jobs.list() if j['kind']=='camera_gimbal'),None)
    if not camera or not camera.get('capture_root'):
        return {'state':'NO_CAPTURE_SESSION','capture_root':None,'captures':[]}
    root=Path(camera['capture_root']).resolve()
    allowed=(Path(ROOT)/'data'/'captures').resolve()
    if not root.is_relative_to(allowed) or not root.is_dir():
        return {'state':'CAPTURE_DIRECTORY_UNAVAILABLE','capture_root':str(root),'captures':[]}
    captures=[]
    for path in sorted(root.rglob('*')):
        if path.is_file() and path.suffix.lower() in ('.png','.jpg','.jpeg'):
            captures.append({'name':path.name,'path':str(path),'size_bytes':path.stat().st_size})
    return {'state':'READY','capture_root':str(root),'capture_session_id':camera.get('capture_session_id'),
            'count':len(captures),'captures':captures}
