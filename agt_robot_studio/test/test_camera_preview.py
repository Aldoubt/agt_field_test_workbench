import os
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from types import SimpleNamespace as NS

import pytest
from PyQt5.QtGui import QImage
from PyQt5.QtWidgets import QApplication

from agt_robot_studio.camera_preview import CameraPreview, decode_image
from agt_robot_studio.ros_worker import RosWorker


def message(data, *, encoding='bgr8', width=2, height=2, step=8, sec=100):
    return NS(width=width, height=height, step=step, encoding=encoding, data=data,
              header=NS(frame_id='camera_optical', stamp=NS(sec=sec, nanosec=0)))


@pytest.fixture(scope='module')
def app():
    return QApplication.instance() or QApplication([])


def test_bgr_padding_and_owned_image():
    # Two BGR pixels per row followed by two bytes of row padding.
    raw = bytearray([0, 0, 255, 0, 255, 0, 99, 99,
                     255, 0, 0, 255, 255, 255, 88, 88])
    frame = decode_image(message(raw), received=100.0, topic='/camera')
    image = frame['image']
    assert image.pixelColor(0, 0).getRgb()[:3] == (255, 0, 0)
    assert image.pixelColor(1, 0).getRgb()[:3] == (0, 255, 0)
    assert image.pixelColor(0, 1).getRgb()[:3] == (0, 0, 255)
    assert image.pixelColor(1, 1).getRgb()[:3] == (255, 255, 255)
    raw[:] = b'\x00'*len(raw)
    assert image.pixelColor(0, 0).getRgb()[:3] == (255, 0, 0)
    assert frame['received'] == 100.0 and frame['stamp_sec'] == 100.0


@pytest.mark.parametrize('encoding,data,expected', [
    ('rgb8', bytes([10, 20, 30]), (10, 20, 30)),
    ('mono8', bytes([81]), (81, 81, 81)),
    ('rgba8', bytes([11, 22, 33, 255]), (11, 22, 33)),
    ('bgra8', bytes([33, 22, 11, 255]), (11, 22, 33)),
])
def test_supported_encodings(encoding, data, expected):
    frame = decode_image(message(data, encoding=encoding, width=1, height=1, step=len(data)))
    assert frame['image'].pixelColor(0, 0).getRgb()[:3] == expected


@pytest.mark.parametrize('msg,reason', [
    (message(bytes(16), step=5), 'step'),
    (message(bytes(15)), '长度'),
    (message(bytes(16), encoding='16UC1'), '编码'),
    (message(b'', width=0, height=0, step=0), '尺寸'),
    (message(bytes(16), sec=-1), '时间戳'),
])
def test_bad_frame_is_rejected(msg, reason):
    with pytest.raises(ValueError, match=reason):
        decode_image(msg)


def test_preview_downscales_without_changing_source_dimensions():
    frame = decode_image(message(bytes(1920*1080), encoding='mono8', width=1920, height=1080, step=1920))
    assert frame['image'].size().width() == 960
    assert frame['image'].size().height() == 540
    assert (frame['width'], frame['height']) == (1920, 1080)


def test_visible_stale_and_disconnected_states_keep_last_image(app):
    view = CameraPreview(timeout_sec=2.0)
    try:
        view.resize(400, 260)
        view.show()
        view.refresh_staleness(now=100)
        assert view.preview_state == 'WAITING'
        frame = decode_image(message(bytes(16)), received=100.0)
        view.set_frame(frame)
        view.refresh_staleness(now=101)
        assert view.preview_state == 'LIVE' and view.canvas.overlay == ''
        view.refresh_staleness(now=102.5)
        assert view.preview_state == 'STALE' and '过期' in view.canvas.overlay
        assert view.canvas.image is frame['image']
        view.set_connection(False)
        assert view.preview_state == 'DISCONNECTED'
        assert '保留最后有效画面' in view.canvas.overlay
        assert not view.grab().isNull()
        view.set_connection(True)
        view.set_error('不支持的编码')
        assert view.preview_state == 'ERROR'
        assert '不支持的编码' in view.status_label.text()
        assert view.canvas.image is frame['image']
    finally:
        view.close()


def test_bad_widget_payload_does_not_replace_good_image(app):
    view = CameraPreview()
    try:
        frame = decode_image(message(bytes(16)))
        view.set_frame(frame)
        view.set_frame({'image': QImage(), 'received': 1.0})
        assert view.frame is frame and view.preview_state == 'ERROR'
    finally:
        view.close()


def test_preview_age_includes_source_transport_delay(app):
    view = CameraPreview(timeout_sec=2.0)
    try:
        frame = decode_image(message(bytes(16)), received=100.0)
        frame['source_age_s'] = 1.6
        view.set_frame(frame)
        view.refresh_staleness(now=100.5)
        assert view.preview_state == 'STALE'
    finally:
        view.close()


def test_worker_uses_bounded_latest_mailbox_and_rejects_old_source(app):
    worker = RosWorker({})
    worker._ros_node = NS(get_clock=lambda: NS(now=lambda: NS(nanoseconds=100_500_000_000)))
    try:
        for i in range(100):
            worker.last_image = 0.0
            worker._on_image(i)
        assert worker._image_queue.qsize() == 1
        assert worker._image_queue.get_nowait()[0] == 99
        good = message(bytes(16))
        worker._decode_image(good, 200.0)
        old = worker.frames.take()['camera']
        assert worker.camera_image_seen == 200.0
        worker.frames.put('camera', old)
        worker._decode_image(message(bytes(16), sec=98), 201.0)
        assert worker.frames.take()['camera'] is old
        assert worker.camera_image_seen == 200.0
        assert '过期' in worker.camera_image_error
        worker._decode_image(good, 202.0)
        assert worker.frames.take()['camera']['received'] == 202.0
        assert worker.camera_image_error is None
    finally:
        worker.shutdown()


def test_worker_throttles_images_before_decoding(app, monkeypatch):
    clock = [100.0]
    monkeypatch.setattr('agt_robot_studio.ros_worker.time.monotonic', lambda: clock[0])
    worker = RosWorker({'camera_preview_hz': 5.0})
    try:
        worker._on_image('first')
        clock[0] = 100.05
        worker._on_image('too-fast')
        assert worker._image_queue.get_nowait()[0] == 'first'
        clock[0] = 100.25
        worker._on_image('next')
        assert worker._image_queue.get_nowait()[0] == 'next'
    finally:
        worker.shutdown()


def test_worker_reports_gimbal_source_age_and_expires_angles(app, monkeypatch):
    clock = [100.0]
    monkeypatch.setattr('agt_robot_studio.ros_worker.time.monotonic', lambda: clock[0])
    worker = RosWorker({})
    worker._map_robot = lambda: None
    states = []
    worker.state.connect(states.append)
    try:
        worker._emit_state()
        assert states[-1]['gimbal_age_s'] is None
        worker.gimbal_angles = {'heading': 20., 'pitch': -10., 'roll': 0.}
        worker.gimbal_seen = 100.0
        clock[0] = 101.4
        worker._emit_state()
        assert states[-1]['gimbal_angles'] == worker.gimbal_angles
        assert states[-1]['gimbal_age_s'] == pytest.approx(1.4)
        clock[0] = 101.8
        worker._emit_state()
        assert states[-1]['gimbal_angles'] is None
        assert states[-1]['gimbal_age_s'] == pytest.approx(1.8)
    finally:
        worker.shutdown()
