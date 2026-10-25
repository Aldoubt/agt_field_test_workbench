"""Explain actual backend states without promoting them to readiness."""
from agt_robot_studio.workflow import localization_progress, session_error_detail


def live(reason, **extra):
    return dict(session={'task': 'localization'}, localized=False,
                localization=dict(fresh=True, local_odom_fresh=True, reason=reason), **extra)


def test_startup_and_missing_bridge_have_distinct_explanations():
    assert '定位进程未启动' in localization_progress({})
    assert 'ROS 桥接未就绪' in localization_progress({'state': 'ROS_BRIDGE_UNAVAILABLE'})


def test_odom_wait_and_manual_global_request_are_distinct():
    assert '等待有效本地里程计' in localization_progress(live('waiting_local_odom'))
    assert '点击“全局重定位”' in localization_progress(live('waiting_global_pose'))


def test_current_manager_overrides_cached_matcher():
    cached = {'state': 'WAIT_STATIONARY', 'stationary_odom_topic': '/wheel/odom'}
    assert '点击“全局重定位”' in localization_progress(
        live('waiting_global_pose', global_relocalization=cached))
    assert 'map_tracking_hold' in localization_progress(
        live('map_tracking_hold', global_relocalization=cached))


def test_stationary_wait_explains_feedback_and_survives_matcher_cache_expiry():
    data = live('global_relocalization:wait_stationary',
                global_relocalization={'state': 'WAIT_STATIONARY',
                                       'stationary_odom_topic': '/wheel/odom'})
    assert '/wheel/odom' in localization_progress(data)
    assert '检查轮速反馈' in localization_progress(data)
    data.update(wheel_odometry_fresh=True, global_relocalization=None)
    assert '等待静止确认' in localization_progress(data)


def test_stale_or_failed_status_cannot_appear_localized():
    data = live('tracking', global_relocalization={'state': 'SUCCESS'})
    data['localization']['fresh'] = False
    assert '状态已过期' in localization_progress(data)
    data['localized'] = True
    assert localization_progress(data) == '已定位'
    data = live('global_relocalization_rejected:weak query',
                global_relocalization={'state': 'SUCCESS'})
    assert 'rejected:weak query' in localization_progress(data)


def test_session_rejection_preserves_backend_reason():
    for error in ('formal navigation requires VERIFIED calibration',
                  'navigation requires matching PHYSICAL_NAVIGATION evidence and all safety checks PASS',
                  'fresh reliable localization required'):
        assert '后台原因：'+error in session_error_detail(error)
    assert session_error_detail('unknown failure') == 'unknown failure'
