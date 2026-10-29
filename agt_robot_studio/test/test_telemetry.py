"""The diagnostic model is pure and requires neither ROS nor Qt."""
import copy

from agt_robot_studio.telemetry import normalize_links


def by_key(api=None, ros=None, **ages):
    return {link.key:link for link in normalize_links(api, ros, **ages)}


def api_snapshot():
    return {'localized': True, 'feedback_fresh': [False, 'remote_control_mode'],
            'chassis': {'online': True, 'enabled': True, 'state': 0}}


def ros_snapshot():
    return {'connected': True,
            'topic_status': {
                'lidar': {'topic': '/custom/lidar', 'age_s': .1, 'rate_hz': 10.},
                'camera': {'topic': '/custom/image', 'age_s': .05, 'rate_hz': 30.},
                'gimbal': {'topic': '/angles', 'age_s': .1, 'rate_hz': 20.}},
            'camera_image_age_s': .4, 'gimbal_age_s': .2,
            'camera_source': {'width': 1920, 'height': 1080, 'encoding': 'bgr8',
                              'frame_id': 'camera_optical', 'stamp_sec': 123.45},
            'map_robot': {'x': 0., 'y': 0., 'source_age_s': .2}}


def test_http_success_with_unavailable_ros_bridge_never_claims_hardware_ready():
    links = by_key({'state': 'ROS_BRIDGE_UNAVAILABLE', 'reason': 'socket absent'}, api_age_s=.2)
    assert links['workbench'].state == 'READY'
    assert links['ros_bridge'].state == 'OFFLINE'
    assert links['ros_bridge'].error == 'socket absent'
    assert links['chassis'].state == links['motion_permission'].state == 'OFFLINE'


def test_online_chassis_is_distinct_from_physical_permission():
    links = by_key(api_snapshot(), api_age_s=.1)
    assert links['workbench'].state == links['ros_bridge'].state == 'READY'
    assert links['chassis'].state == 'READY'
    assert links['motion_permission'].state == 'BLOCKED'
    assert links['motion_permission'].error == 'remote_control_mode'


def test_raw_camera_rate_valid_frame_age_and_source_metadata_are_independent():
    links = by_key(None, ros_snapshot(), ros_age_s=.3)
    assert links['camera'].state == 'READY'
    assert links['camera'].age_s == .7
    assert links['camera'].rate_hz == 30.
    assert links['camera'].source == '/custom/image'
    assert '1920×1080 bgr8' in links['camera'].detail
    assert 'stamp=123.45' in links['camera'].detail
    assert links['map_tf'].age_s == .5


def test_frozen_snapshots_age_even_without_another_callback():
    api, ros = api_snapshot(), ros_snapshot()
    original = copy.deepcopy((api, ros))
    links = by_key(api, ros, api_age_s=5., ros_age_s=3.)
    assert links['workbench'].state == links['ros_bridge'].state == 'STALE'
    assert links['chassis'].state == 'STALE'
    assert all(links[key].state == 'STALE' for key in ('ros_display', 'lidar', 'camera', 'gimbal', 'map_tf'))
    assert links['camera'].age_s == 3.4
    assert (api, ros) == original


def test_api_transport_failure_keeps_last_age_but_invalidates_cached_hardware_status():
    api = api_snapshot()
    api['transport_error'] = 'connection refused'
    links = by_key(api, ros_snapshot(), api_age_s=.4, ros_age_s=.1)
    assert links['workbench'].state == 'ERROR'
    assert links['workbench'].age_s == .4
    assert links['workbench'].error == 'connection refused'
    assert links['ros_bridge'].state == links['chassis'].state == 'ERROR'
    assert links['camera'].state == 'READY'  # Independent read-only ROS path.


def test_one_received_message_does_not_require_a_computable_frequency():
    ros = ros_snapshot()
    ros['topic_status']['lidar']['rate_hz'] = None
    link = by_key(None, ros, ros_age_s=0.)['lidar']
    assert link.state == 'READY' and link.rate_hz is None
    assert '未验证传感器源时钟' in link.detail


def test_received_invalid_images_show_decode_error_not_green_rate():
    ros = ros_snapshot()
    ros['camera_image_age_s'] = None
    ros['camera_image_error'] = '不支持的编码'
    link = by_key(None, ros, ros_age_s=.1)['camera']
    assert link.state == 'ERROR' and link.rate_hz == 30.
    assert link.age_s is None
    assert '尚无有效样本' in link.detail


def test_old_adapters_without_source_age_and_malformed_ages_are_unknown():
    ros = {'connected': True, 'rates': {'lidar': 10., 'imu': float('nan')},
           'camera_image_age_s': float('inf'), 'gimbal_age_s': -1.}
    links = by_key(None, ros, ros_age_s=0.)
    assert links['lidar'].state == links['camera'].state == links['gimbal'].state == 'WAITING'
    assert links['imu'].rate_hz is None
    assert by_key(api_snapshot(), ros, api_age_s=float('nan'), ros_age_s=True)['workbench'].state == 'OFFLINE'


def test_source_age_thresholds_still_apply_to_fresh_ros_adapter():
    ros = ros_snapshot()
    ros['camera_image_age_s'] = 1.9
    ros['gimbal_age_s'] = 1.4
    ros['map_robot']['source_age_s'] = .7
    links = by_key(None, ros, ros_age_s=.3)
    assert links['ros_display'].state == 'READY'
    assert links['camera'].state == links['gimbal'].state == links['map_tf'].state == 'STALE'
