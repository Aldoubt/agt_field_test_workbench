"""Read-only diagnostic snapshots; none of these indicators authorize motion.

``api_age_s`` and ``ros_age_s`` are the caller's monotonic elapsed time since the
last successful API snapshot / ROS state callback. Source ages inside a snapshot
are advanced by that elapsed time, so frozen callbacks cannot stay green.
This module imports neither Qt nor ROS and never performs I/O.
"""
from dataclasses import dataclass
import math
from collections.abc import Mapping
from typing import Optional


@dataclass(frozen=True)
class LinkStatus:
    key: str
    label: str
    source: str
    state: str
    rate_hz: Optional[float] = None
    age_s: Optional[float] = None
    detail: str = ''
    error: Optional[str] = None


def _mapping(value):
    return value if isinstance(value, Mapping) else {}


def _number(value):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    value = float(value)
    return value if math.isfinite(value) and value >= 0 else None


def _age(source_age, snapshot_age):
    age, elapsed = _number(source_age), _number(snapshot_age)
    return age + elapsed if age is not None and elapsed is not None else None


def _text(value):
    return str(value)[:400] if value else ''


def normalize_links(api_state=None, ros_state=None, *, api_age_s=None, ros_age_s=None):
    """Normalize service/data/permission links without inferring one from another.

    States are READY, WAITING, STALE, ERROR, OFFLINE or BLOCKED. Rate is measured
    packet arrival frequency, never a configured rate or a motion-readiness hint.
    Missing ages stay unknown; this deliberately supports old adapters without
    claiming that their last known rates establish live hardware status.
    """
    api, ros = _mapping(api_state), _mapping(ros_state)
    api_age, ros_age = _number(api_age_s), _number(ros_age_s)
    transport_error = _text(api.get('transport_error')) or None
    api_fresh = api_age is not None and api_age <= 4. and not transport_error
    ros_fresh = ros_age is not None and ros_age <= 1.5
    bridge_missing = api.get('state') == 'ROS_BRIDGE_UNAVAILABLE'
    bridge_known = 'feedback_fresh' in api or 'localized' in api
    bridge_ready = api_fresh and not bridge_missing and bridge_known
    display_ready = ros_fresh and ros.get('connected') is True
    display_error = _text(ros.get('adapter_error') or ros.get('callback_error')) or None
    result = [
        LinkStatus('workbench', 'Workbench API', '/api/live',
                   'ERROR' if transport_error else 'READY' if api_fresh else 'STALE' if api_age is not None else 'OFFLINE',
                   age_s=api_age, detail='HTTP 回复新鲜度；不代表 ROS 桥或硬件就绪', error=transport_error),
        LinkStatus('ros_bridge', 'ROS 控制桥服务', 'Workbench → ros_bridge.sock',
                   'OFFLINE' if bridge_missing else 'ERROR' if transport_error else
                   'READY' if bridge_ready else 'STALE' if api_age is not None and not api_fresh else
                   'OFFLINE' if api_age is None else 'WAITING',
                   age_s=api_age, detail='只表示状态 RPC 可用；运动许可单独判定',
                   error=(_text(api.get('reason')) or None) if bridge_missing else transport_error),
        LinkStatus('ros_display', 'ROS 显示适配器', 'RosWorker / rclpy',
                   'STALE' if ros_age is not None and not ros_fresh else 'ERROR' if display_error else
                   'READY' if display_ready else 'OFFLINE',
                   age_s=ros_age, detail='只读订阅；没有速度或执行控制发布器',
                   error=display_error),
    ]
    topic_status = _mapping(ros.get('topic_status'))
    defaults = {
        'lidar': ('雷达数据', '/livox/lidar', 1.),
        'imu': ('IMU 数据', '/livox/imu', 1.),
        'odom': ('局部里程计', '/agt/odometry/local', 1.5),
        'camera': ('相机图像', '/cv_camera0/image_raw', 2.),
        'gimbal': ('云台实测角度', '/pantilt_camera_serial0/pantilt_angle_info', 1.5),
    }
    for key, (label, default_topic, limit) in defaults.items():
        raw = _mapping(topic_status.get(key))
        rate = _number(raw.get('rate_hz', _mapping(ros.get('rates')).get(key)))
        arrival_age = _age(raw.get('age_s'), ros_age)
        valid_age_key = {'camera':'camera_image_age_s', 'odom':'odom_age_s', 'gimbal':'gimbal_age_s'}.get(key)
        age = _age(ros.get(valid_age_key), ros_age) if valid_age_key else arrival_age
        error = _text(raw.get('error') or ros.get({'camera':'camera_image_error', 'odom':'odom_error',
                                                'gimbal':'gimbal_error'}.get(key, ''))) or None
        if not display_ready:
            state = 'STALE' if ros_age is not None and not ros_fresh else 'OFFLINE'
        elif error:
            state = 'ERROR'
        elif age is None:
            state = 'WAITING'
        elif age > limit:
            state = 'STALE'
        else:
            state = 'READY'
        detail = '频率为回调收到的原始消息频率；不代表数据内容或运动许可'
        if key in ('lidar', 'imu'):
            detail += '；age 为本机接收龄，未验证传感器源时钟'
        elif key == 'gimbal':
            detail = 'age 为最后有效实测角度接收龄；反馈消息未提供源时间戳'
        elif key == 'odom':
            detail = 'age 包含有效位姿源时间延迟；旧位姿不会当作当前位置'
        elif key == 'camera':
            source = _mapping(ros.get('camera_source'))
            detail = 'age 为最后有效图像源时间龄；原始帧率与预览限频分别计算'
            if source:
                detail += ' · {}×{} {} · frame={} · stamp={}'.format(
                    source.get('width', '?'), source.get('height', '?'), source.get('encoding', '?'),
                    source.get('frame_id', '?'), source.get('stamp_sec', '?'))
        if arrival_age is not None and age is None:
            detail += f'；最后消息接收于 {arrival_age:.1f}s 前，尚无有效样本'
        result.append(LinkStatus(key, label, _text(raw.get('topic')) or default_topic,
                                 state, rate, age, detail, error))
    chassis = _mapping(api.get('chassis'))
    feedback = api.get('feedback_fresh')
    permitted = isinstance(feedback, (list, tuple)) and bool(feedback) and feedback[0] is True
    reason = _text(feedback[1]) if isinstance(feedback, (list, tuple)) and len(feedback)>1 and not permitted else ''
    missing_state = 'ERROR' if transport_error else 'STALE' if api_age is not None and not api_fresh else 'OFFLINE'
    result.extend([
        LinkStatus('chassis', '底盘通讯', '/api/live.chassis',
                   ('READY' if chassis.get('online') is True else 'WAITING') if bridge_ready else missing_state,
                   age_s=api_age,
                   detail='桥快照龄；通讯在线不等于运动许可 · state=' + _text(chassis.get('state', '未知'))),
        LinkStatus('motion_permission', '底盘物理许可反馈', '/api/live.feedback_fresh',
                   ('READY' if permitted else 'BLOCKED') if bridge_ready else missing_state,
                   age_s=api_age,
                   detail='物理反馈检查；提交动作时后端仍复核地图、定位、标定与完整运动许可',
                   error=reason or None),
        LinkStatus('localization', '全局定位状态', '/api/live.localization',
                   ('READY' if api.get('localized') is True else 'WAITING') if bridge_ready else missing_state,
                   age_s=api_age, detail=_text(_mapping(api.get('localization')).get('reason'))),
    ])
    pose = _mapping(ros.get('map_robot'))
    tf_age = _age(pose.get('source_age_s'), ros_age)
    tf_error = _text(ros.get('map_robot_error')) or None
    map_frame = _text(ros.get('map_frame') or pose.get('frame_id')) or 'map'
    result.append(LinkStatus('map_tf', '机器人地图坐标', f'TF: {map_frame} → base_link',
                             ('ERROR' if tf_error else 'WAITING' if not pose or tf_age is None else
                              'STALE' if tf_age > .8 else 'READY') if display_ready else
                             'STALE' if ros_age is not None and not ros_fresh else 'OFFLINE',
                             age_s=tf_age, detail='已验证的 map → base_link TF 源时间龄', error=tf_error))
    return result
