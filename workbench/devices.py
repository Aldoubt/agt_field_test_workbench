"""Independent optional C1 camera launcher.

Only owns a camera job. It never stops mapping, ROS bridge, sensors, Nav2,
chassis, or a recorder; no automatic restart/motion on lost hardware.
"""
from pathlib import Path
from .runtime import ROS_EXEC, ros_ready, ros_call


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
        raise ValueError('existing legacy navigation session already owns camera')
    try:
        ros_call(['ros2', 'pkg', 'prefix', 'autolabor_c1_bringup'], timeout=5)
    except Exception as exc:
        raise ValueError('C1 ROS driver not installed or not sourced: autolabor_c1_bringup') from exc
    # Construct an argv vector, never a shell command from operator input.
    return jobs._new('camera_gimbal',
                     ['bash', str(ROS_EXEC), 'ros2', 'launch', 'autolabor_c1_bringup',
                      'autolabor_c1.launch.py', 'device_path:='+camera,
                      'port_name:='+gimbal, 'gui:=false'],
                     {'device_path':camera, 'port_name':gimbal,
                      'readiness':'UNVERIFIED_UNTIL_IMAGE_AND_GIMBAL_FEEDBACK'})


def stop_camera(jobs):
    active = [job for job in jobs.list() if job['kind'] == 'camera_gimbal' and job['state'] == 'RUNNING']
    if not active:
        return {'state':'STOPPED', 'detail':'no managed camera process running'}
    return jobs.stop(active[0]['id'])


def camera_status(jobs):
    camera = next((j for j in jobs.list() if j['kind'] == 'camera_gimbal'), None)
    if camera is None:
        return {'state':'STOPPED','readiness':'UNVERIFIED'}
    return {'state':camera['state'],'job_id':camera['id'],
            'log':camera.get('log'), 'readiness':camera.get('readiness','UNVERIFIED'),
            'device_path':camera.get('device_path'),'port_name':camera.get('port_name')}
