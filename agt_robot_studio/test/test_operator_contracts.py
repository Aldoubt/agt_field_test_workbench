import os
os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
from types import SimpleNamespace as NS
import pytest
from PyQt5.QtWidgets import QApplication
from agt_robot_studio.ui import Studio, DEFAULT_CONFIG


@pytest.fixture
def studio():
    app=QApplication.instance() or QApplication([])
    window=Studio(DEFAULT_CONFIG,demo=True);window.timer.stop();window.display_timer.stop()
    calls=[];window.api=NS(call=lambda *args:calls.append(args))
    try:yield window,calls
    finally:window.api=None;window.control_api=None;window.telemetry_api=None;window.stop_api=None;window.close()


def test_three_themes_preserve_editable_route_and_never_call_robot(studio,tmp_path,monkeypatch):
    window,calls=studio;window.load_demo_scene()
    original=window.waypoint_editor.points()
    appearance=tmp_path/'appearance.json';monkeypatch.setattr('agt_robot_studio.ui.APPEARANCE_CONFIG',appearance)
    for name in ('industrial','daylight','field'):
        window.set_theme(name,persist=True)
        assert window.theme==name and window.waypoint_editor.points()==original
        assert not window.submit_goal_btn.isEnabled()
    assert calls==[] and 'field' in appearance.read_text()


def test_telemetry_reads_do_not_wait_in_asset_or_control_lane(studio):
    window,_=studio;window.demo=False;assets=[];control=[];telemetry=[];stops=[]
    window.api=NS(call=lambda *args:assets.append(args))
    window.control_api=NS(call=lambda *args:control.append(args))
    window.telemetry_api=NS(call=lambda *args:telemetry.append(args))
    window.stop_api=NS(call=lambda *args:stops.append(args))
    window._api('cloud_preview','GET','/api/clouds/a/preview')
    window._api('refresh_live','GET','/api/live')
    window._api('stop_goal','POST','/api/control/stop',{})
    window._api('goal','POST','/api/control/goal',{})
    assert [row[0] for row in assets]==['cloud_preview']
    assert [row[0] for row in telemetry]==['refresh_live'] and telemetry[0][-1]==2.
    assert [row[0] for row in control]==['goal']
    assert [row[0] for row in stops]==['stop_goal'] and stops[0][-1]==2.


def test_camera_completion_must_match_current_job_even_when_jobs_are_out_of_order(studio):
    window,calls=studio;window.demo=False
    window._camera_request(save_image=False)
    tag=calls[-1][3]['tag'];window._camera_request(save_image=False)
    assert len(calls)==1
    window._on_api('camera_move',True,{'id':'current','tag':tag})
    old={'kind':'camera_capture','id':'old','tag':'old','state':'PASS','save_image':True,
         'capture_result':{'image_path':'old.jpg'}}
    window._on_api('refresh_jobs',True,[old])
    assert window.camera_request_waiting and 'old.jpg' not in window.capture_status_label.text()
    current={'kind':'camera_capture','id':'current','tag':tag,'state':'PASS','save_image':False}
    window._on_api('refresh_jobs',True,[old,current])
    assert not window.camera_request_waiting and '本次未拍照' in window.capture_status_label.text()


def test_diagnostics_keep_service_health_separate_from_hardware_permission(studio):
    window,_=studio;window.demo=False;window.live_status={'state':'ROS_BRIDGE_UNAVAILABLE','reason':'offline'}
    import time
    window.live_seen=time.monotonic();window._update_diagnostics()
    links={link['key']:link for link in window.diagnostics.snapshot['links']}
    assert links['workbench']['state']=='READY'
    assert links['ros_bridge']['state']=='OFFLINE'
    assert links['motion_permission']['state']!='READY'


def test_old_bridge_protocol_blocks_new_motion_with_upgrade_explanation(studio):
    window,calls=studio;window.demo=False
    import time
    window.live_seen=time.monotonic()
    window.live_status={'session':{'robot':'bunker','task':'navigation','map_id':'map-1'},'localized':True}
    ready,reason=window._navigation_precheck()
    assert not ready and '回执协议' in reason and not calls
