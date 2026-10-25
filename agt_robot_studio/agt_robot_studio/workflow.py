"""Operator explanations of backend observations; never grants readiness or motion."""


def localization_progress(live):
    """Describe the current blocking stage without inferring localization success."""
    if live.get('state') == 'ROS_BRIDGE_UNAVAILABLE':
        return 'ROS 桥接未就绪；先启动传感器，若已启动请检查会话日志'
    session = live.get('session') or {}
    if session.get('task') not in ('localization', 'navigation'):
        return '定位进程未启动：启动传感器，选择地图后点击“启动定位”'
    if live.get('localized'):
        return '已定位'
    manager = live.get('localization') or {}
    if manager and not manager.get('fresh'):
        return '定位管理器状态已过期；检查定位进程和日志'
    reason = str(manager.get('reason') or '')
    if reason == 'waiting_local_odom' or (manager and not manager.get('local_odom_fresh')):
        return '等待有效本地里程计：检查输入、时间戳和里程计适配器'
    # Current manager state takes precedence over a previous request's cached result.
    if reason == 'waiting_global_pose':
        return '本地里程计已就绪；停车后点击“全局重定位”，启动定位不会自动匹配地图'
    matcher = live.get('global_relocalization') or {}
    state = (reason.split(':', 1)[1].upper() if reason.startswith('global_relocalization:')
             else str(matcher.get('state') or ''))
    if reason and not reason.startswith('global_relocalization:'):
        return '定位等待：'+reason
    if state == 'WAIT_STATIONARY':
        topic = matcher.get('stationary_odom_topic') or '底盘轮速'
        action = ('保持停车，等待静止确认' if live.get('wheel_odometry_fresh')
                  else '在设备页连接底盘，检查轮速反馈并保持停车')
        return f'等待静止反馈（{topic}）：{action}'
    if state:
        detail = matcher.get('detail') if matcher.get('state') == state else ''
        return '全局重定位：'+state+' · '+str(detail or '')[:180]
    return '定位进程已启动，等待有效里程计与定位管理器状态'


def session_error_detail(message):
    """Add an actionable explanation while retaining the backend rejection text."""
    message = str(message)
    hints = {
        'formal navigation requires VERIFIED calibration':
            '当前标定尚未完成实车验证。正式导航要求 VERIFIED 标定；地图已验收不等于标定已验证。',
        'start and verify the independent chassis component before navigation':
            '请先在设备页连接并核对底盘反馈；“启动导航”不会自动连接底盘。',
        'start the matching independent sensor stack before localization/navigation':
            '请先在设备页启动当前机器人的 MID360S / IMU，再启动定位。',
        'navigation requires matching PHYSICAL_NAVIGATION evidence':
            '缺少匹配当前机器人、地图和标定的实车导航验收报告；仅勾选现场确认不能代替报告。',
        'fresh reliable localization required':
            '尚未获得新鲜可靠的全局定位；请先停车并完成“全局重定位”。',
    }
    for fragment, hint in hints.items():
        if fragment in message:
            return hint+'\n\n后台原因：'+message
    return message
