import os
os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
import threading
import time
from types import SimpleNamespace as NS
import numpy as np
import pytest
from PyQt5.QtWidgets import QApplication
from PyQt5.QtCore import QTimer
from PyQt5.QtTest import QTest
from agt_robot_studio.visualization import LatestFrames, cloud_snapshot, transform_xyz
from agt_robot_studio.ui import GridView, Studio, DEFAULT_CONFIG
from agt_robot_studio.tri_views import TriViewWorkbench
from agt_robot_studio.ros_worker import RosWorker
from agt_robot_studio.workbench_client import ApiWorker


@pytest.fixture(scope='module')
def app():
    return QApplication.instance() or QApplication([])


def test_latest_snapshots_bounded_and_shared(app):
    mailbox=LatestFrames()
    for i in range(1000):mailbox.put('cloud',i);mailbox.put('grid',i)
    assert mailbox.take()==dict(cloud=999,grid=999)
    assert mailbox.take()=={}
    frame=cloud_snapshot(np.array([[0,0,0],[1,2,3]]),'map','fixture')
    view=TriViewWorkbench(GridView(True));view.set_cloud(frame)
    assert view.top.snapshot is view.side.snapshot is frame
    pose=dict(x=1.,y=2.,z=3.,yaw=0.,frame_id='map')
    view.set_robot(pose);view.set_robot(None)
    assert view.top.pose is pose and view.top.pose_stale and view.grid.robot is pose
    assert not frame['xyz'].flags.writeable
    view.top.scale=3.;assert view.side.scale==1.
    view.expand(1);assert view.expanded==1 and view.frames[0].isHidden()
    view.expand(1);assert view.expanded is None and not view.frames[0].isHidden()
    view.close()


def test_grid_refresh_preserves_zoom_and_yaw_aware_center(app):
    view=GridView(True);view.resize(600,400)
    info=dict(width=10,height=8,data=[0]*80,resolution=.5,origin=(10.,20.,np.pi/2),frame_id='map')
    view.set_grid(info);view.scale=2.;view.pan_x=21.
    view.set_grid(info);assert view.scale==2. and view.pan_x==21.
    view.set_robot(dict(x=9.,y=22.,z=0.,yaw=0.,frame_id='map'));view.center_robot()
    point=view.world_screen(9.,22.)
    assert abs(point.x()-300)<1e-6 and abs(point.y()-200)<1e-6
    view.close()


def test_transform_does_not_mix_frames():
    tf=NS(translation=NS(x=1.,y=2.,z=3.),rotation=NS(x=0.,y=0.,z=np.sin(np.pi/4),w=np.cos(np.pi/4)))
    assert np.allclose(transform_xyz(np.array([[1.,0.,0.]]),tf),[[1.,3.,3.]])


def test_callback_queues_bounded_and_bad_grid_retains_last_frame(app):
    worker=RosWorker(DEFAULT_CONFIG)
    for i in range(100):worker.last_cloud=0.;worker._on_cloud(i);worker._on_grid(i)
    assert worker._decode_queue.qsize()==worker._grid_queue.qsize()==1
    worker.frames.put('grid',dict(valid=True))
    worker._decode_grid(NS(header=NS(frame_id='odom')))
    assert worker.frames.take()=={'grid':dict(valid=True)}
    assert '栅格' in worker.display_error
    worker.shutdown()


def test_api_wait_does_not_block_gui_event_loop(app):
    entered=threading.Event();release=threading.Event()
    class Client:
        def request(self,*args,**kwargs):entered.set();release.wait(2);return {}
    worker=ApiWorker(Client());worker.start();worker.call('camera','POST','/api/devices/camera/capture',{})
    assert entered.wait(1)
    ticks=[];timer=QTimer();timer.timeout.connect(lambda:ticks.append(1));timer.start(10)
    QTest.qWait(80)
    release.set();worker.shutdown();timer.stop()
    assert len(ticks)>=3


def test_stale_cloud_paints_and_invalid_snapshot_does_not_replace_it(app):
    frame=cloud_snapshot(np.array([[0,0,0],[1,2,3]]),'map','fixture',received=time.monotonic()-10)
    view=TriViewWorkbench(GridView(True));view.set_cloud(frame);view.resize(900,500);view.show()
    app.processEvents();assert not view.grab().isNull()
    assert view.top.snapshot['received']==frame['received']
    with pytest.raises(ValueError):cloud_snapshot(np.empty((0,3)),'map','invalid')
    assert view.side.snapshot is frame
    view.close()


def test_stop_request_bypasses_long_asset_http_request(app):
    entered=threading.Event();release=threading.Event();stopped=threading.Event()
    class Client:
        def request(self,method,path,*a,**k):
            if path=='/api/maps/diff':entered.set();release.wait(3)
            if path=='/api/control/stop':stopped.set()
            return {}
    window=Studio(DEFAULT_CONFIG,demo=True)
    window.demo=False
    window.api=ApiWorker(Client());window.control_api=ApiWorker(Client())
    window.api.start();window.control_api.start()
    try:
        window.api.call('asset_wait','POST','/api/maps/diff',{})
        assert entered.wait(1)
        window.stop_goal()
        assert stopped.wait(1) and window.api.active_request=='asset_wait'
    finally:
        release.set();window.api.shutdown();window.control_api.shutdown()
        window.api=None;window.control_api=None;window.close()


def test_two_real_backend_choices_and_sc_database_payload(app):
    window=Studio(DEFAULT_CONFIG,demo=True)
    try:
        assert [window.global_backend_box.itemData(i) for i in range(window.global_backend_box.count())]==['sc_kiss','bbs']
        assert window._session_payload('localization','map-1')['global_backend']=='sc_kiss'
        window.global_backend_box.setCurrentIndex(0);window.sc_database_edit.setText('/data/agri/scan_context')
        payload=window._session_payload('localization','map-1')
        assert payload['global_backend']=='sc_kiss' and payload['sc_database']=='/data/agri/scan_context'
    finally:window.close()
