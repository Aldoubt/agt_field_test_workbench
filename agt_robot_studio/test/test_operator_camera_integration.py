"""Offline operator regressions: real Studio widgets, fake API, no robot I/O."""
import os
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from types import SimpleNamespace as NS

import pytest
from PyQt5.QtGui import QImage
from PyQt5.QtWidgets import QApplication

from agt_robot_studio.ui import DEFAULT_CONFIG, Studio
from agt_robot_studio.visualization import LatestFrames


@pytest.fixture(scope='module')
def app():
    return QApplication.instance() or QApplication([])


@pytest.fixture
def studio(app, monkeypatch):
    clock = [100.0]
    monkeypatch.setattr('agt_robot_studio.ui.time.monotonic', lambda: clock[0])
    window = Studio(dict(DEFAULT_CONFIG, camera_image_topic='/camera/custom/image'), demo=True)
    window.timer.stop()
    window.display_timer.stop()
    window.camera_preview.timer.stop()
    calls = []
    window.demo = False
    window.api = NS(call=lambda *args: calls.append(args))
    window.nav_map_box.addItem('Map', 'map-1')
    window.displayed_map_id = 'map-1'
    window.live_status = {
        'session': {'id': 'session-1', 'robot': 'bunker', 'task': 'navigation', 'map_id': 'map-1'},
        'localized': True, 'chassis': {'enabled': True, 'online': True, 'state': 0},
        'feedback_fresh': [True, ''],
        'nav2_states': {'controller_server': 3, 'bt_navigator': 3},
    }
    window.live_seen = clock[0]
    try:
        yield window, calls, clock
    finally:
        window.worker = None
        window.api = None
        window.mapping_job_id = None
        window.mapping_saving = False
        window.close()


def test_configured_camera_topic_and_mailbox_are_wired(studio):
    window, calls, clock = studio
    assert window.camera_preview.topic == '/camera/custom/image'
    frames = LatestFrames()
    window.worker = NS(frames=frames)
    image = QImage(2, 2, QImage.Format_RGB888)
    image.fill(0xff00ff)
    frame = dict(image=image, received=clock[0], topic='/camera/custom/image', encoding='rgb8')
    frames.put('camera', frame)
    window._on_state({'connected': True})
    window._drain_display()
    assert window.camera_preview.frame is frame
    assert window.camera_preview.preview_state == 'LIVE'
    assert not frames.take()
    assert calls == []
    clock[0] += 2.1
    window.camera_preview.refresh_staleness()
    assert window.camera_preview.preview_state == 'STALE'
    window._on_state({'connected': False})
    assert window.camera_preview.preview_state == 'DISCONNECTED'
    assert window.camera_preview.canvas.image is image


def test_frozen_display_worker_and_old_backend_cannot_look_live(studio):
    window, _, clock = studio
    window._on_state({'connected':True,'rates':{'lidar':10.,'imu':100.,'odom':50.}})
    window._update_live()
    assert '已定位' in window.localization_label.text()
    clock[0] += 4.1
    window._drain_display()
    assert not window.ros_display_connected
    assert '过期' in window.localization_label.text()
    assert '过期' in window.pose_label.text()


def test_decoder_error_is_visible_without_replacing_last_image(studio):
    window, _, clock = studio
    image = QImage(2, 2, QImage.Format_RGB888)
    image.fill(0)
    window.camera_preview.set_frame(dict(image=image, received=clock[0]))
    window._on_state({'connected': True, 'camera_image_error': '图像时间戳缺失或过期'})
    assert window.camera_preview.preview_state == 'ERROR'
    assert '时间戳' in window.camera_preview.status_label.text()
    assert window.camera_preview.canvas.image is image


def test_actual_angles_expire_and_cannot_be_copied_to_waypoint(studio):
    window, calls, clock = studio
    window.route_points = [{'x': 1., 'y': 2., 'yaw': 0., 'photo': True,
                            'heading': 0., 'pitch': 0., 'roll': 0., 'photo_required': True}]
    window.waypoint_editor.set_points(window.route_points, 0)
    window._on_state({'connected': True, 'gimbal_angles': {'heading': 20., 'pitch': -10., 'roll': 0.}})
    assert window.copy_angles_btn.isEnabled()
    window.copy_actual_angles()
    assert window.route_points[0]['heading'] == 20.
    clock[0] += 2.1
    window._drain_display()
    assert window.actual_gimbal is None
    assert not window.copy_angles_btn.isEnabled()
    assert '过期' in window.actual_angles_label.text()
    window.copy_actual_angles()
    assert '需要新鲜实测角度' in window.capture_status_label.text()
    assert calls == []


def test_cached_gimbal_state_does_not_restart_actual_feedback_age(studio):
    window, _, clock = studio
    window._on_state({'connected': True, 'gimbal_age_s': 1.4,
                      'gimbal_angles': {'heading': 20., 'pitch': -10., 'roll': 0.}})
    assert window.actual_gimbal_seen == pytest.approx(clock[0] - 1.4)
    clock[0] += .7
    window._drain_display()
    assert window.actual_gimbal is None
    assert not window.copy_angles_btn.isEnabled()


def test_move_pass_does_not_claim_photo_saved(studio):
    window, _, _ = studio
    window._on_api('refresh_jobs', True, [{
        'id': 'move-1', 'kind': 'camera_capture', 'state': 'PASS', 'save_image': False,
        'capture_result': {'success': True},
    }])
    assert '已到位' in window.capture_status_label.text()
    assert '未拍照' in window.capture_status_label.text()
    assert '拍照成功' not in window.capture_status_label.text()


def test_saved_capture_reports_the_returned_file(studio):
    window, _, _ = studio
    window._on_api('refresh_jobs', True, [{
        'id': 'capture-1', 'kind': 'camera_capture', 'state': 'PASS', 'save_image': True,
        'capture_result': {'success': True, 'image_path': '/captures/p1.png', 'image_sha256': 'abc'},
    }])
    assert '拍照成功' in window.capture_status_label.text()
    assert '/captures/p1.png' in window.capture_status_label.text()


def test_capture_pass_without_image_evidence_does_not_claim_photo_saved(studio):
    window, _, _ = studio
    window._on_api('refresh_jobs', True, [{
        'id': 'capture-1', 'kind': 'camera_capture', 'state': 'PASS', 'save_image': True,
        'capture_result': {'success': True},
    }])
    assert '拍照成功' not in window.capture_status_label.text()


def test_old_action_feedback_is_labeled_as_historical(studio):
    window, _, _ = studio
    window._on_api('refresh_jobs', True, [{
        'id': 'capture-1', 'kind': 'camera_capture', 'state': 'RUNNING',
        'feedback': {'stage': 'MOVING', 'actual_angles': {'heading': 15., 'pitch': -10., 'roll': 0.},
                     'stable_count': 0, 'observed_at': '2000-01-01T00:00:00+00:00'},
    }])
    assert any(word in window.capture_status_label.text() for word in ('过期', '上次', '历史'))
    assert window.actual_gimbal is None


def test_backend_status_expiration_removes_online_chassis_claim(studio):
    window, _, clock = studio
    window._update_global_status()
    assert '通讯在线' in window.chassis_state_label.text()
    clock[0] += 4.1
    window._drain_display()
    assert '通讯在线' not in window.chassis_state_label.text()
    assert '已定位' not in window.mode_state_label.text()
    assert not window._navigation_precheck()[0]


@pytest.mark.parametrize('mutation', ['missing_navigator', 'wrong_robot'])
def test_navigation_precheck_requires_both_nodes_and_current_robot(studio, mutation):
    window, _, _ = studio
    assert window._navigation_precheck()[0]
    if mutation == 'missing_navigator':
        window.live_status['nav2_states'].pop('bt_navigator')
    else:
        window.robot_box.setCurrentText('FW-mini')
    assert not window._navigation_precheck()[0]


def test_navigation_waits_for_the_selected_map_to_finish_loading(studio):
    window, calls, _ = studio
    window.displayed_map_id = 'old-map'
    assert not window._navigation_precheck()[0]
    window.send_goal({'x': 3., 'y': 4., 'yaw': .2})
    assert window.draft_goal is None
    assert '尚未加载' in window.draft_label.text()
    assert calls == []


def test_goal_selection_is_draft_and_repeated_submit_is_bounded(studio):
    window, calls, _ = studio
    window.send_goal({'x': 3., 'y': 4., 'yaw': .2})
    assert calls == []
    assert window.submit_goal_btn.isEnabled()
    window.submit_goal()
    window.submit_goal()
    assert len(calls) == 1
    assert calls[0][:4] == ('goal', 'POST', '/api/control/goal', {
        'robot': 'bunker', 'map_id': 'map-1', 'pose': {'x': 3., 'y': 4., 'yaw': .2}})
    assert not window.submit_goal_btn.isEnabled()
    window._on_api('goal', True, {'accepted': True})
    assert window.draft_goal is None
    assert not window.goal_pending

