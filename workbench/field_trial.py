"""CLI/RViz field trials using the existing guarded ROS launch chain."""
import argparse
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import shlex
import shutil
import time
import uuid

import yaml
from agt_field_test_bringup import calibration, profiles
from agt_field_test_bringup.livox import validate_mid360s_config
from agt_field_test_bringup.navigation_policy import require_navigation_calibration
from .maps import Maps, hash_file, scan_context_database, relocalization_assets
from .relocalization import validate_sc_database
from .runtime import REPO, ROOT, TEMPLATES


def make_plan(root, config_path, map_id=None, global_backend=None, planning_profile='navigation'):
    """Validate selected inputs without starting ROS nodes or changing acceptance."""
    root = Path(root).resolve()
    if planning_profile not in ('navigation','distance'):
        raise ValueError('invalid planning profile')
    cfg = yaml.safe_load(Path(config_path).read_text())
    resolve = lambda value: (root/Path(value).expanduser()).resolve()
    selected_map = Maps(root).get(map_id or cfg['map_id'])
    Maps(root).validate_identity(selected_map)
    if selected_map['status'] != 'ACCEPTED' or selected_map.get('localization_geometry_valid') is False:
        raise ValueError('select an ACCEPTED map with valid localization geometry')
    if not selected_map.get('source_pcd') or not selected_map.get('source_pcd_sha256'):
        raise ValueError('selected map requires a matching localization PCD')
    if hash_file(Path(selected_map['source_pcd'])) != selected_map['source_pcd_sha256']:
        raise ValueError('localization PCD changed')
    cal_path = resolve(cfg['calibration_bundle'])
    bundle = calibration.load(cal_path, 'bunker')
    require_navigation_calibration('field_test', True, 'bunker', bundle)
    profile_path = resolve(cfg['robot_profile'])
    profile = profiles.load(profile_path, 'bunker')
    livox_path = resolve(cfg['livox_config'])
    validate_mid360s_config(livox_path)
    backend = global_backend or cfg.get('global_backend', 'sc_kiss')
    if backend == 'sc_kiss':
        database = validate_sc_database(resolve(cfg['sc_database']) if cfg.get('sc_database')
                                        else scan_context_database(selected_map), selected_map)
        backend_args = ['sc_database:='+database]
        strategy = 'Scan Context → KISS → GICP'
    elif backend == 'bbs':
        assets = relocalization_assets(selected_map)
        if not assets:
            raise ValueError('BBS assets missing or not associated with selected PCD; build matching assets first')
        backend_args = ['relocalization_assets:='+assets]
        strategy = ('Polar Context → candidate 3D-BBS → GICP' if (Path(assets)/'polar_context.db').is_file()
                    else 'whole-map 3D-BBS → GICP (no Polar Context candidates)')
    else:
        raise ValueError('global_backend must be bbs or sc_kiss')
    args = ['robot:=bunker', 'navigation_mode:=field_test', 'supervised:=true',
            'navigation_autostart:=false', 'planning_profile:='+planning_profile,
            'enable_mid360:=true', 'enable_chassis:=true', 'enable_description:=true',
            'enable_localization:=true', 'enable_navigation:=true', 'enable_guard:=true',
            'enable_inspection:=false', 'enable_camera_gimbal:=false',
            'lio_backend:=fastlio2', 'global_backend:='+backend, *backend_args,
            'global_map:='+selected_map['source_pcd'], 'map_yaml:='+selected_map['yaml'],
            'robot_profile_file:='+str(profile_path), 'calibration_bundle:='+str(cal_path),
            'livox_config:='+str(livox_path)]
    return dict(mode='field_test', global_backend=backend, relocalization_strategy=strategy,
        planning_profile=planning_profile,
        map_id=selected_map['id'], map_name=selected_map.get('name'),
        map_size_m=[round(selected_map[key]*selected_map['resolution'], 2) for key in ('width', 'height')],
        calibration_status=bundle['status'], calibration_sha256=calibration.digest(bundle),
        calibration_file_sha256=hash_file(cal_path), profile_sha256=profile['profile_sha256'],
        livox_config_sha256=hash_file(livox_path),
        calibration_acceptance=cfg.get('calibration_acceptance', 'operator-selected calibration for supervised field test'),
        limits=dict(forward_mps=profile['forward_mps'], reverse_mps=profile['reverse_mps'],
                    angular_radps=profile['angular_radps']),
        launch_args=args, inputs=dict(calibration_bundle=str(cal_path),
            robot_profile_file=str(profile_path), livox_config=str(livox_path)))


def snapshot_plan(root, plan):
    """Freeze configuration before starting; never rewrite original calibration."""
    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    directory = Path(root)/'runs'/('field_trial_'+stamp+'_'+uuid.uuid4().hex[:8])
    directory.mkdir(parents=True)
    frozen = dict(plan, launch_args=list(plan['launch_args']))
    for key, source in plan['inputs'].items():
        target = directory/(key+Path(source).suffix)
        shutil.copyfile(source, target)
        expected = plan[{'calibration_bundle': 'calibration_file_sha256',
                         'robot_profile_file': 'profile_sha256',
                         'livox_config': 'livox_config_sha256'}[key]]
        if hash_file(target) != expected:
            raise ValueError('configuration changed during startup: '+source)
        prefix = key+':='
        frozen['launch_args'] = [prefix+str(target) if item.startswith(prefix) else item
                                 for item in frozen['launch_args']]
    (directory/'session.json').write_text(json.dumps(frozen, ensure_ascii=False, indent=2)+'\n')
    return directory, frozen


def matcher_log_status(root=ROOT, proc_root=Path('/proc'), log_root=None):
    """Recover a one-shot result only from a currently running matcher in this workspace."""
    executable = str(Path(root)/'install/agt_global_relocalization/lib/agt_global_relocalization/global_relocalization')
    processes = []
    for process in Path(proc_root).glob('[0-9]*'):
        try:
            if executable in (process/'cmdline').read_bytes().decode().split('\0'):
                processes.append(process.name)
        except (OSError, UnicodeError):
            continue
    if len(processes) != 1:
        return None
    directory = Path(log_root or os.environ.get('ROS_LOG_DIR') or
                     Path(os.environ.get('ROS_HOME', str(Path.home()/'.ros')))/'log')
    logs = sorted(directory.glob('python3_'+processes[0]+'_*.log'), key=lambda p: p.stat().st_mtime)
    if not logs:
        return None
    try:
        with logs[-1].open('rb') as stream:
            stream.seek(max(0, logs[-1].stat().st_size-65536))
            lines = stream.read().decode(errors='replace').splitlines()
        for line in reversed(lines):
            if '] [agt_global_relocalization]: {' not in line:
                continue
            event = json.loads(line.split('] [agt_global_relocalization]: ', 1)[1])
            if 'state' in event:
                return dict(status=event, source='current_matcher_ros_log', cached=True,
                            note='最近一次匹配事件；缓存结果，不表示当前定位已有效')
    except (OSError, ValueError):
        pass
    return None


def observe(seconds):
    """Read-only ROS observation using ROS stamp age and steady sampling duration."""
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    from agt_robot_interfaces.msg import AdapterStatus, LocalizationStatus
    from bunker_msgs.msg import BunkerStatus
    from sensor_msgs.msg import Imu
    from livox_ros_driver2.msg import CustomMsg
    from lifecycle_msgs.srv import GetState
    rclpy.init()
    node = rclpy.create_node('field_trial_status')
    received, first_received, samples, ages, counts = {}, {}, {}, {}, {}

    def callback(name, msg):
        received[name] = time.monotonic()
        first_received.setdefault(name, received[name])
        counts[name] = counts.get(name, 0)+1
        samples[name] = msg
        if name in ('local_odom', 'wheel_odom'):
            stamp = msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
            ages.setdefault(name, []).append(node.get_clock().now().nanoseconds/1e9-stamp)

    subscriptions = []
    for typ, topic, name, raw in (
            (Odometry, '/agt/odometry/local', 'local_odom', False),
            (Odometry, '/wheel/odom', 'wheel_odom', False),
            (CustomMsg, '/livox/lidar', 'lidar', True),
            (Imu, '/livox/imu', 'imu', True),
            (LocalizationStatus, '/agt/localization/status', 'localization', False),
            (String, '/agt/global_relocalization/status', 'matcher', False),
            (AdapterStatus, '/agt/odometry/adapter_status', 'adapter', False),
            (BunkerStatus, '/bunker_status', 'chassis', False)):
        subscriptions.append(node.create_subscription(typ, topic,
            lambda msg, name=name: callback(name, msg), qos_profile_sensor_data, raw=raw))
    clients = {name: node.create_client(GetState, '/'+name+'/get_state')
               for name in ('controller_server', 'bt_navigator')}
    futures = {}
    started = time.monotonic()
    try:
        while time.monotonic()-started < seconds:
            rclpy.spin_once(node, timeout_sec=.05)
            for name, client in clients.items():
                if name not in futures and client.service_is_ready():
                    futures[name] = client.call_async(GetState.Request())
        now = time.monotonic()
        result = {name: dict(hz=(round((count-1)/(received[name]-first_received[name]), 1)
                                if count > 1 and received[name] > first_received[name] else None),
                            samples=count,
                            receive_age_sec=round(now-received[name], 3))
                  for name, count in counts.items()}
        for name, values in ages.items():
            ordered = sorted(values)
            result[name]['stamp_age_ms'] = {key: round(1000*ordered[int((len(ordered)-1)*q)], 2)
                                           for key, q in (('p50', .5), ('p95', .95), ('max', 1.))}
        for name in ('matcher',):
            if name in samples:
                try: result[name]['status'] = json.loads(samples[name].data)
                except ValueError: result[name]['status'] = samples[name].data
        if 'adapter' in samples:
            adapter = samples['adapter']
            result['adapter']['status'] = dict(state=int(adapter.state),
                input_age_sec=round(adapter.input_age_sec, 3),
                last_valid_pose_age_sec=round(adapter.last_valid_pose_age_sec, 3),
                accepted_count=int(adapter.accepted_count), rejected_count=int(adapter.rejected_count),
                publish_rate=round(adapter.publish_rate, 1))
        if 'matcher' not in samples:
            cached = matcher_log_status()
            if cached:
                result['matcher'] = cached
        manager = samples.get('localization')
        if manager:
            result['localization'].update(state=int(manager.state), reason=manager.reason,
                local_odom_fresh=bool(manager.local_odom_fresh),
                global_correction_valid=bool(manager.global_correction_valid))
        if 'chassis' in samples:
            chassis = samples['chassis']
            result['chassis'].update(vehicle_state=int(chassis.vehicle_state),
                control_mode=int(chassis.control_mode), error_code=int(chassis.error_code))
        result['nav2'] = {name: future.result().current_state.label
                         for name, future in futures.items() if future.done() and future.result()}
        if not counts:
            result['note'] = '未观察到运行中的测试链路；先运行 ./field_trial start'
        # This describes observations only; the existing command guard decides motion permission.
        print(json.dumps(result, ensure_ascii=False, indent=2))
    finally:
        node.destroy_node()
        rclpy.shutdown()


def require_navigation_ready(node, timeout_sec=3.):
    """Check live localization/odom/TF before a Nav2 lifecycle call can block."""
    from agt_robot_interfaces.msg import LocalizationStatus
    from nav_msgs.msg import Odometry
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from tf2_ros import Buffer, TransformListener, TransformException
    import rclpy
    samples = {}
    subscriptions = [
        node.create_subscription(LocalizationStatus, '/agt/localization/status',
                                 lambda msg: samples.update(status=msg), qos_profile_sensor_data),
        node.create_subscription(Odometry, '/agt/odometry/local',
                                 lambda msg: samples.update(odom=msg), qos_profile_sensor_data)]
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    reason = '未收到有效本地里程计和定位状态'
    deadline = time.monotonic()+timeout_sec
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
            now = node.get_clock().now().nanoseconds
            status, odom = samples.get('status'), samples.get('odom')
            if status is None:
                reason = '未收到 /agt/localization/status'
                continue
            if not status.local_odom_fresh or not status.global_correction_valid:
                reason = f'定位未就绪：{status.reason}；local_odom_fresh={status.local_odom_fresh}，global_correction_valid={status.global_correction_valid}'
                continue
            status_age = (now-Time.from_msg(status.stamp).nanoseconds)/1e9
            if not -.05 <= status_age <= .75:
                reason = '定位状态已过期'
                continue
            if odom is None:
                reason = '未收到 /agt/odometry/local；先检查 FAST-LIO input_fault'
                continue
            age = (now-Time.from_msg(odom.header.stamp).nanoseconds)/1e9
            if not -.05 <= age <= .20:
                reason = f'本地里程计时间戳年龄 {age:.3f}s，超出 0.20s 范围'
                continue
            try:
                transform = buffer.lookup_transform('map', 'base_link', Time())
                age = (now-Time.from_msg(transform.header.stamp).nanoseconds)/1e9
                if not -.05 <= age <= .20:
                    reason = 'map → odom → base_link TF 已过期'
                    continue
            except TransformException:
                reason = '缺少 map → odom → base_link TF'
                continue
            return
        raise ValueError(reason+'；未发送 Nav2 STARTUP。运行 ./field_trial status 检查，恢复本地里程计并完成重定位后再 nav。')
    finally:
        for subscription in subscriptions:
            node.destroy_subscription(subscription)
        listener.unregister()


def activate_navigation(map_only=False):
    """Activate map and Nav2 explicitly after the operator requests relocalization."""
    import rclpy
    from nav2_msgs.srv import ManageLifecycleNodes
    from lifecycle_msgs.srv import GetState
    rclpy.init()
    node = rclpy.create_node('field_trial_activate_navigation')
    try:
        managers = [('lifecycle_manager_map', ('map_server',))]
        if not map_only:
            managers.append(('lifecycle_manager_navigation', ('controller_server', 'bt_navigator')))
        for manager, managed in managers:
            if manager == 'lifecycle_manager_navigation':
                require_navigation_ready(node)
            active = []
            for name in managed:
                client = node.create_client(GetState, '/'+name+'/get_state')
                if not client.wait_for_service(timeout_sec=3.):
                    raise ValueError(name+' 未启动；先运行 ./field_trial start')
                future = client.call_async(GetState.Request())
                rclpy.spin_until_future_complete(node, future, timeout_sec=3.)
                active.append(future.done() and future.result() and future.result().current_state.id == 3)
                node.destroy_client(client)
            if all(active):
                print(manager+' already ACTIVE', flush=True)
                continue
            client = node.create_client(ManageLifecycleNodes, '/'+manager+'/manage_nodes')
            if not client.wait_for_service(timeout_sec=3.):
                raise ValueError(manager+' service unavailable')
            future = client.call_async(ManageLifecycleNodes.Request(command=ManageLifecycleNodes.Request.STARTUP))
            rclpy.spin_until_future_complete(node, future, timeout_sec=45.)
            if not future.done() or not future.result() or not future.result().success:
                raise ValueError(manager+' 启动未成功；检查全局定位、TF 和 start 终端日志')
            print(manager+' STARTUP succeeded', flush=True)
            node.destroy_client(client)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def request_relocalization(timeout_sec):
    """Set the backend's compute budget, then request normal stationary relocalization."""
    import rclpy
    from rclpy.parameter import Parameter
    from rcl_interfaces.srv import SetParametersAtomically
    from std_srvs.srv import Trigger
    rclpy.init()
    node = rclpy.create_node('field_trial_request_relocalization')
    try:
        def call(typ, name, request):
            client = node.create_client(typ, name)
            if not client.wait_for_service(timeout_sec=5.):
                raise ValueError('service unavailable: '+name)
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=5.)
            if not future.done() or future.result() is None:
                raise ValueError('service response timeout: '+name)
            response = future.result()
            node.destroy_client(client)
            return response
        changed = call(SetParametersAtomically,
            '/agt_global_relocalization/set_parameters_atomically',
            SetParametersAtomically.Request(parameters=[
                Parameter('sdk_timeout_sec', value=float(timeout_sec)).to_parameter_msg()]))
        if not changed.result.successful:
            raise ValueError('backend rejected compute budget: '+changed.result.reason)
        result = call(Trigger, '/agt/localization/relocalize', Trigger.Request())
        print(json.dumps(dict(success=bool(result.success), message=result.message,
                              sdk_timeout_sec=timeout_sec), ensure_ascii=False, indent=2))
        return 0 if result.success else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main(argv=None):
    parser = argparse.ArgumentParser(description='Bunker 命令行 + RViz 现场测试；无需正式验收报告')
    parser.add_argument('command', choices=('check', 'start', 'build-polar-context', 'map', 'nav', 'rviz', 'relocalize', 'status', 'record', 'cancel'))
    parser.add_argument('--config', type=Path, default=ROOT/'config/field_trial.yaml')
    parser.add_argument('--map-id', help='显式覆盖配置中的地图版本')
    parser.add_argument('--global-backend', choices=('sc_kiss', 'bbs'), help='启动时显式选择全局匹配后端')
    parser.add_argument('--planning-profile', choices=('navigation','distance'), default='navigation',
                        help='distance 用于空地距离测试的路径平滑配置')
    parser.add_argument('--hold-system-clock', action='store_true',
                        help='start 时暂时停止已同步的 systemd-timesyncd，退出后恢复，避免现场时钟跳变')
    parser.add_argument('--timeout-sec', type=float, help='relocalize 请求的匹配计算预算（秒），不改变数据时效门禁')
    parser.add_argument('--dry-run', action='store_true', help='只输出计划，不启动任何 ROS 节点')
    parser.add_argument('--seconds', type=float, default=3., help='status 采样秒数')
    args = parser.parse_args(argv)
    if args.hold_system_clock and args.command != 'start':
        parser.error('--hold-system-clock applies to start only')
    if args.global_backend and args.command not in ('check', 'start'):
        parser.error('--global-backend applies to check/start; restart the session to change backends')
    if args.seconds <= 0 or not math.isfinite(args.seconds):
        parser.error('--seconds must be finite and positive')
    if args.timeout_sec is not None:
        if args.command != 'relocalize' or not math.isfinite(args.timeout_sec) or args.timeout_sec <= 0:
            parser.error('--timeout-sec requires relocalize and a finite positive duration')
    if args.command == 'build-polar-context':
        plan = make_plan(ROOT, args.config, args.map_id, global_backend='bbs')
        if args.dry_run:
            print('Build Polar Context from recorded keyframes for '+plan['map_id']+'; existing PCD/BBS/SC assets preserved')
            return 0
        from .artifacts import Clouds
        record = Maps(ROOT).get(plan['map_id'])
        result = Clouds(ROOT).build_assets(record['cloud_id'], map_id=record['id'])
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return 0 if result.get('polar_context') == 'READY' else 1
    elif args.command in ('start', 'check'):
        plan = make_plan(ROOT, args.config, args.map_id, args.global_backend,args.planning_profile)
        command = ['bash', str(REPO/'scripts/run_field_test.sh'), *plan['launch_args']]
        if args.command == 'check' or args.dry_run:
            if args.hold_system_clock:
                command = ['bash', str(REPO/'scripts/hold_field_clock.sh'), *command]
            print(json.dumps(plan, ensure_ascii=False, indent=2))
            print(shlex.join(command))
            return 0
        directory, frozen = snapshot_plan(ROOT, plan)
        print('现场测试配置已冻结：'+str(directory), flush=True)
        print(f"地图：{plan['map_name']} · {plan['map_id']} · {plan['map_size_m'][0]} × {plan['map_size_m'][1]} m", flush=True)
        print('全局匹配：'+plan['relocalization_strategy'], flush=True)
        print('启动传感器、底盘、定位和 Nav2；停车后另开终端运行 ./field_trial relocalize。', flush=True)
        print('先运行 ./field_trial map 显示静态地图，便于核对；此命令不激活导航。', flush=True)
        print('全局定位成功后运行 ./field_trial nav 激活地图和 Nav2；然后在 RViz 发目标。', flush=True)
        print('退出整条链路：在本终端 Ctrl+C。定位未就绪时 command guard 保持停车。', flush=True)
        command = ['bash', str(REPO/'scripts/run_field_test.sh'), *frozen['launch_args']]
        if args.hold_system_clock:
            command = ['bash', str(REPO/'scripts/hold_field_clock.sh'), *command]
    elif args.command in ('map', 'nav'):
        if args.dry_run:
            print('STARTUP /lifecycle_manager_map/manage_nodes'+
                  (' then /lifecycle_manager_navigation/manage_nodes' if args.command == 'nav' else ' (map only)'))
        else:
            activate_navigation(map_only=args.command == 'map')
        return 0
    elif args.command == 'status':
        if args.dry_run:
            print('Read-only ROS observation for '+str(args.seconds)+' seconds')
        else:
            observe(args.seconds)
        return 0
    elif args.command == 'rviz':
        command = ['ros2', 'run', 'rviz2', 'rviz2', '-d', str(REPO/'config/field_trial.rviz')]
    elif args.command == 'relocalize':
        if args.timeout_sec is not None:
            if args.dry_run:
                print(f'Set sdk_timeout_sec={args.timeout_sec}, then request /agt/localization/relocalize')
                return 0
            return request_relocalization(args.timeout_sec)
        command = ['ros2', 'service', 'call', '/agt/localization/relocalize', 'std_srvs/srv/Trigger', '{}']
    elif args.command == 'cancel':
        command = ['ros2', 'service', 'call', '/navigate_to_pose/_action/cancel_goal', 'action_msgs/srv/CancelGoal', '{}']
    else:
        stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
        output = ROOT/'bags'/('field_trial_'+stamp+'_'+uuid.uuid4().hex[:8])
        topics = list(dict.fromkeys([*TEMPLATES['full_field_test'], '/fastlio2/lio_odom',
                                    '/agt/odometry/adapter_status', '/rosout', '/map', '/plan']))
        command = ['ros2', 'bag', 'record', '-o', str(output), *topics]
    if args.dry_run:
        print(shlex.join(command))
        return 0
    os.execvp(command[0], command)


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (ValueError, KeyError, OSError) as error:
        raise SystemExit('现场测试配置错误：'+str(error))
