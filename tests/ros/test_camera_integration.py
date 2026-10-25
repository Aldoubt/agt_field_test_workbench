"""Real Humble messages + C1 capability + deterministic software device fixture.

Run explicitly with AGT_RUN_ROS_INTEGRATION=1 in a built Humble overlay.
This does not validate USB, serial, CAN or physical motion.
"""
import os
import json
from pathlib import Path
import signal
import subprocess
import sys
import time
import pytest

pytestmark=pytest.mark.skipif(os.environ.get('AGT_RUN_ROS_INTEGRATION')!='1',reason='requires built ROS Humble/C1 overlay')
REPO=Path(__file__).resolve().parents[2]
C1=REPO.parent/'Autolabor-C1-ROS2'


@pytest.fixture
def camera_stack(tmp_path):
    processes=[]
    def start(**params):
        fake=[sys.executable,str(C1/'src/autolabor_c1_bringup/scripts/fake_gimbal_camera.py'),
              '--ros-args','-r','__ns:=/pantilt_camera_serial0']
        for name,value in params.items():fake+=['-p',name+':='+str(value).lower()]
        capability=['ros2','run','camera_gimbal_capability','camera_gimbal_capability','--ros-args',
            '-r','__ns:=/camera_gimbal','-p','output_root:='+str(tmp_path),
            '-p','move_action:=/pantilt_camera_serial0/move_pantilt']
        for index,command in enumerate((fake,capability)):
            with (tmp_path/f'node{index}.log').open('w') as log:
                processes.append(subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True))
        import rclpy
        from rclpy.node import Node
        from camera_gimbal_interfaces.msg import CapabilityHealth
        rclpy.init();node=Node('workbench_health_integration');samples=[]
        node.create_subscription(CapabilityHealth,'/camera_gimbal/health',samples.append,10)
        deadline=time.monotonic()+8
        while time.monotonic()<deadline and (not samples or samples[-1].state!=1):rclpy.spin_once(node,timeout_sec=.05)
        assert samples and samples[-1].state==1, 'C1 fixture did not become READY'
        node.destroy_node();rclpy.shutdown()
    yield start
    for process in processes:
        if process.poll() is None:
            os.killpg(process.pid,signal.SIGINT)
            try:process.wait(timeout=5)
            except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=3)


def invoke(tmp_path,**request):
    payload=dict(heading=25.,pitch=-10.,roll=0.,tag='integration',save_image=True,
        metadata={'task_point':'P001','map_version':'synthetic-map-fixture'},**request)
    path=tmp_path/'request.json';path.write_text(json.dumps(payload));result=tmp_path/'result.json'
    process=subprocess.run([sys.executable,str(REPO/'scripts/camera_capture.py'),'--request',str(path),
        '--result',str(result),'--capture-root',str(tmp_path)],capture_output=True,text=True,timeout=24)
    return process,json.loads(result.read_text())


def test_real_acquire_view_interface_and_image_manifest(camera_stack,tmp_path):
    camera_stack()
    process,result=invoke(tmp_path)
    assert process.returncode==0,process.stderr
    assert result['success'] and result['actual_angles']==dict(heading=25.,pitch=-10.,roll=0.)
    assert Path(result['image_path']).is_file() and result['image_sha256']
    assert result['image_stamp']['nanoseconds']>result['reached_stamp']['nanoseconds']
    assert result['metadata']['task_point']=='P001'
    assert Path(result['manifest_path']).is_file() and (tmp_path/'feedback.json').is_file()
    assert result['pose_valid'] is False  # no robot TF in this camera-only fixture


def test_real_image_timeout_is_a_recorded_failure(camera_stack,tmp_path):
    camera_stack(drop_images_on_motion=True)
    process,result=invoke(tmp_path)
    assert process.returncode!=0 and result['success'] is False
    assert result['error_code']==301
    assert not list(tmp_path.rglob('*.png'))
    assert json.loads((tmp_path/'actions.jsonl').read_text().splitlines()[-1])['success'] is False


def test_real_action_deadline_requests_cancellation(camera_stack,tmp_path):
    camera_stack(move_delay=30.)
    started=time.monotonic();process,result=invoke(tmp_path)
    assert process.returncode!=0 and result['success'] is False
    assert time.monotonic()-started<22
    assert 'timed out' in result['message'].lower() or 'timeout' in result['message'].lower() or result.get('error_code')==202
