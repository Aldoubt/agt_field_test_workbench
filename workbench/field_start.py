"""Foreground launch modes composed from the existing split CLI components."""
import argparse
import fcntl
import json
import math
import os
from pathlib import Path
import shlex
import signal
import sys
import time

from . import field_trial, field_photo
from .runtime import Jobs, REPO, ROOT, write_json, utc_now
from .ros_parameters import read_parameters
from .mode_state import publish_mode_state


def positive(value):
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise argparse.ArgumentTypeError('must be finite and positive')
    return number


def make_mode_plan(args):
    planning_profile = 'distance' if args.mode == 'distance' else 'navigation'
    base = field_trial.make_plan(ROOT, args.config, args.map_id, args.global_backend,planning_profile)
    common = ['--config', str(args.config), '--map-id', base['map_id'],
              '--global-backend', base['global_backend'],'--planning-profile',planning_profile]
    core = ['bash', str(REPO/'scripts/field_trial.sh'), 'start', *common]
    components = [('navigation_stack', core)]
    if args.mode == 'photo':
        components += [('camera', ['bash', str(REPO/'scripts/field_photo.sh'), 'camera',
                                   '--camera-device', args.camera_device, '--gimbal-port', args.gimbal_port]),
                       ('runtime', ['bash', str(REPO/'scripts/field_photo.sh'), 'runtime']),
                       ('queue', ['bash', str(REPO/'scripts/field_photo.sh'), 'queue',
                                  '--map-id', base['map_id']])]
    elif args.mode == 'distance':
        from ament_index_python.packages import get_package_share_directory
        params = Path(get_package_share_directory('agt_navigation_runtime'))/'config/runtime.yaml'
        components.append(('runtime', ['ros2', 'run', 'agt_navigation_runtime', 'mission_runtime',
            '--ros-args', '--params-file', str(params), '-p', 'nav_behavior_tree:=""',
            '-p', 'wait_for_bunker_control:=true', '-p', 'recenter_after_capture:=false', '-p', 'record_root:='+str(ROOT/'data/captures/missions')]))
    if not args.no_rviz:
        script = 'field_photo.sh' if args.mode == 'photo' else 'field_trial.sh'
        components.append(('rviz', ['bash', str(REPO/'scripts'/script), 'rviz']))
    return dict(mode=args.mode, map_id=base['map_id'], base=base,
                hold_system_clock=not args.no_clock_hold, clock_owner='foreground_mode',
                automatic_relocalization=not args.manual_localization,
                components=[dict(name=name, command=command) for name,command in components],
                external_task_trigger='./field_photo start' if args.mode == 'photo' else None)


class Readiness:
    """ROS reads and startup services only: this class has no Action client."""
    CORE_NODES = {'agt_localization_manager', 'agt_map_tracker', 'controller_server',
                  'bt_navigator', 'map_server', 'agt_bunker_base', 'agt_bunker_base_node',
                  'livox_ros_driver2', 'livox_ros_driver2_node', 'fastlio2'}

    def __init__(self, health_check=lambda: None):
        import rclpy
        from rclpy.qos import qos_profile_sensor_data
        from agt_robot_interfaces.msg import LocalizationStatus
        from nav_msgs.msg import Odometry
        self.rclpy = rclpy
        rclpy.init(args=[])
        self.node = rclpy.create_node('field_mode_startup')
        self.health_check = health_check
        self.status = self.odom = None
        self.status_rx = self.odom_rx = 0.
        self.stationary_since = None
        self.node.create_subscription(LocalizationStatus, '/agt/localization/status', self.on_status,
                                      qos_profile_sensor_data)
        self.node.create_subscription(Odometry, '/agt/odometry/local', self.on_odom, qos_profile_sensor_data)

    def on_status(self, msg):
        self.status, self.status_rx = msg, time.monotonic()

    def on_odom(self, msg):
        from agt_navigation_runtime.mission_runtime import odom_planar_motion_rate
        now = time.monotonic()
        rates = odom_planar_motion_rate(self.odom, msg, .5) if self.odom is not None else None
        twist = msg.twist.twist
        stopped = (rates is not None and rates[0] <= .04 and rates[1] <= .06
                   and math.sqrt(twist.linear.x**2+twist.linear.y**2+twist.linear.z**2) <= .03
                   and math.sqrt(twist.angular.x**2+twist.angular.y**2+twist.angular.z**2) <= .05
                   and now-self.odom_rx <= .5)
        self.stationary_since = (self.stationary_since or now) if stopped else None
        self.odom, self.odom_rx = msg, now

    def fresh_odom(self):
        if self.odom is None or time.monotonic()-self.odom_rx > .5:
            return False
        stamp = self.odom.header.stamp
        age = (self.node.get_clock().now().nanoseconds-(stamp.sec*10**9+stamp.nanosec))/1e9
        return -.05 <= age <= .20

    def stationary(self):
        return (self.fresh_odom() and self.stationary_since is not None
                and time.monotonic()-self.stationary_since >= .8)

    def localized(self, map_id):
        if self.status is None or time.monotonic()-self.status_rx > .5:
            return False
        s = self.status
        age = (self.node.get_clock().now().nanoseconds-(s.stamp.sec*10**9+s.stamp.nanosec))/1e9
        if s.map_id and s.map_id != map_id:
            raise ValueError('当前定位地图与本次启动选择不一致')
        return (self.fresh_odom() and -.05 <= age <= .5
                and s.local_odom_fresh and s.global_correction_valid)

    def wait(self, predicate, timeout, message):
        print(message, flush=True)
        deadline, last_notice = time.monotonic()+timeout, time.monotonic()
        while time.monotonic() < deadline:
            self.health_check()
            self.rclpy.spin_once(self.node, timeout_sec=.05)
            if predicate():
                return
            if time.monotonic()-last_notice >= 10.:
                reason = self.status.reason if self.status is not None else '等待传感器/定位状态'
                print(message+'；'+reason, flush=True)
                last_notice = time.monotonic()
        raise ValueError(message+'超时；查看本次 runs 日志和 ./field_trial status')

    def reject_existing(self, mode):
        # Discover only; never stop or adopt a manually started component.
        until = time.monotonic()+1.
        while time.monotonic() < until:
            self.rclpy.spin_once(self.node, timeout_sec=.05)
        names = {name for name,_ in self.node.get_node_names_and_namespaces()}
        collision = names & self.CORE_NODES
        if mode in ('photo', 'distance'):
            collision |= names & {'mission_runtime'}
        if mode == 'photo':
            collision |= names & {'rviz_patrol'}
            for name,namespace in self.node.get_node_names_and_namespaces():
                if namespace in ('/camera_gimbal','/cv_camera0','/pantilt_camera_serial0'):
                    collision.add(namespace+'/'+name)
        if collision:
            raise ValueError('已有分开启动的组件：'+', '.join(sorted(collision))+
                '。继续使用原 field_trial/field_photo 命令；组合入口用于独立启动，未停止旧进程。')

    def call(self, typ, name, request, timeout=45., before_send=None):
        client = self.node.create_client(typ, name)
        try:
            self.wait(client.service_is_ready, timeout, '等待 '+name)
            if before_send is not None:
                before_send()
            future = client.call_async(request)
            self.wait(future.done, timeout, '等待服务响应 '+name)
            if future.result() is None:
                raise ValueError(name+' returned no response')
            return future.result()
        finally:
            self.node.destroy_client(client)

    def activate(self, map_only=False):
        from lifecycle_msgs.srv import GetState
        from nav2_msgs.srv import ManageLifecycleNodes
        groups = [('lifecycle_manager_map', ['map_server'])]
        if not map_only:
            field_trial.require_navigation_ready(self.node)
            groups.append(('lifecycle_manager_navigation', ['controller_server','bt_navigator']))
        for manager,nodes in groups:
            if all(self.call(GetState, '/'+name+'/get_state', GetState.Request()).current_state.id == 3
                   for name in nodes):
                continue
            response = self.call(ManageLifecycleNodes, '/'+manager+'/manage_nodes',
                                 ManageLifecycleNodes.Request(command=ManageLifecycleNodes.Request.STARTUP))
            if not response.success:
                raise ValueError(manager+' STARTUP failed')

    def relocalize(self, timeout):
        from rcl_interfaces.srv import SetParametersAtomically
        from rclpy.parameter import Parameter
        from std_srvs.srv import Trigger
        if not self.stationary():
            raise ValueError('未确认停车，未请求重定位')
        changed = self.call(SetParametersAtomically, '/agt_global_relocalization/set_parameters_atomically',
            SetParametersAtomically.Request(parameters=[Parameter('sdk_timeout_sec',value=timeout).to_parameter_msg()]))
        if not changed.result.successful:
            raise ValueError(changed.result.reason)
        if not self.stationary():
            raise ValueError('设置计算预算期间车辆移动，未请求重定位')
        def check_stopped():
            if not self.stationary():
                raise ValueError('服务发现期间车辆移动，未请求重定位')
        response = self.call(Trigger, '/agt/localization/relocalize', Trigger.Request(),
                             before_send=check_stopped)
        if not response.success:
            raise ValueError(response.message)

    def parameters(self, name, keys):
        return read_parameters(self.node,name,keys,health_check=self.health_check)

    def mission_ready(self, mode, timeout):
        from std_srvs.srv import Trigger
        # Service discovery only; do not invoke start or any Action endpoint.
        task = self.node.create_client(Trigger, '/agt/task/cancel')
        try:
            self.wait(task.service_is_ready,timeout,'等待任务执行器')
        finally:
            self.node.destroy_client(task)
        if mode == 'photo':
            runtime = self.parameters('mission_runtime',['nav_behavior_tree','recenter_after_capture'])
            controller = self.parameters('controller_server',['goal_checker_plugins',
                'photo_goal_checker.xy_goal_tolerance','photo_goal_checker.yaw_goal_tolerance'])
            field_photo.validate_photo_parameters(runtime,controller)
            queue = self.node.create_client(Trigger,'/agt/rviz_patrol/start')
            try:
                self.wait(queue.service_is_ready,timeout,'等待 RViz 点位队列（不触发）')
            finally:
                self.node.destroy_client(queue)
            from camera_gimbal_interfaces.msg import CapabilityHealth
            health = []
            sub = self.node.create_subscription(CapabilityHealth,'/camera_gimbal/health',
                lambda m:health.append((time.monotonic(),m)),10)
            try:
                def camera_ready():
                    if not health or time.monotonic()-health[-1][0] > 1.:
                        return False
                    m = health[-1][1]
                    return (m.state == m.STATE_READY and m.camera_alive and m.gimbal_serial_connected
                            and m.gimbal_feedback_alive and m.move_action_ready and not m.busy)
                self.wait(camera_ready,timeout,'等待相机图像和云台反馈')
            finally:
                self.node.destroy_subscription(sub)
        elif self.parameters('mission_runtime',['nav_behavior_tree'])['nav_behavior_tree'] != '':
            raise ValueError('距离模式应使用原精度到达判定，执行器配置不匹配')

    def close(self):
        self.node.destroy_node()
        self.rclpy.shutdown()


class ModeSession:
    def __init__(self, plan, jobs):
        self.plan, self.jobs, self.owned = plan, jobs, []

    def start(self, name):
        component = next(c for c in self.plan['components'] if c['name'] == name)
        kind = 'field_session' if name == 'navigation_stack' else 'mode_'+name
        record = self.jobs._new(kind, component['command'], {'mode':self.plan['mode']})
        self.owned.append((name, record['id']))
        print(name+' 已启动，日志：'+record['log'], flush=True)

    def check(self):
        for name,ident in self.owned:
            process = self.jobs.jobs[ident]['process']
            code = process.poll()
            if code is not None and name != 'rviz':
                log = self.jobs.jobs[ident]['record']['log']
                tail = ''
                try:
                    with Path(log).open('rb') as stream:
                        stream.seek(0,2)
                        stream.seek(max(0,stream.tell()-65536))
                        lines = stream.read().decode(errors='replace').splitlines()
                        # Launch can exit 0 after a required child aborts; the
                        # final lines then contain only orderly peer shutdown.
                        failures = [line for line in lines if any(marker in line for marker in (
                            'process has died', 'free():', 'double free',
                            'terminate called', 'Traceback (most recent call last)'))]
                        tail = '\n'.join((failures[-5:] + lines[-8:]))
                except OSError:
                    pass
                raise ValueError(f'{name} exited ({code}); {log}'+('\n'+tail if tail else ''))

    def close(self):
        errors = []
        for name,ident in reversed(self.owned):
            print('停止本次组件 '+name,flush=True)
            try:
                self.jobs.stop(ident)
            except (OSError,ValueError) as exc:
                errors.append(name+': '+str(exc))
        if errors:
            raise ValueError('; '.join(errors))


def parser():
    p = argparse.ArgumentParser(description='独立组合启动；原分终端入口保持可用；不发送任务目标')
    p.add_argument('mode', choices=['distance','nav','photo'])
    p.add_argument('--config', type=Path, default=ROOT/'config/field_trial.yaml')
    p.add_argument('--map-id')
    p.add_argument('--global-backend', choices=['bbs','sc_kiss'], default='bbs')
    p.add_argument('--manual-localization', action='store_true', help='等待外部 field_trial relocalize，不自动请求')
    p.add_argument('--no-clock-hold', action='store_true', help='不执行现有系统时钟保持助手')
    p.add_argument('--no-rviz', action='store_true')
    p.add_argument('--startup-timeout-sec', type=positive, default=180.)
    p.add_argument('--relocalization-timeout-sec', type=positive, default=30.)
    p.add_argument('--camera-device', default='/dev/v4l/by-id/usb-Wasintek_Wasintek_camera_00.00.01-video-index0')
    p.add_argument('--gimbal-port', default='/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0')
    p.add_argument('--dry-run', action='store_true')
    return p


def foreground_clock_command(argv):
    # Keep sudo and clock restoration in the operator's controlling terminal.
    # The component Jobs use setsid and must never ask for a password there.
    return ['bash',str(REPO/'scripts/hold_field_clock.sh'),
            'env','AGT_FIELD_MODE_CLOCK_SCOPE=foreground',
            'bash',str(REPO/'scripts/field_start.sh'),*argv]


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    args = parser().parse_args(argv)
    plan = make_mode_plan(args)
    if args.dry_run:
        print(json.dumps(plan,ensure_ascii=False,indent=2))
        for c in plan['components']:
            print(c['name']+': '+shlex.join(c['command']))
        if not args.no_clock_hold:
            print('前台时钟保持：'+shlex.join(foreground_clock_command(argv)))
        print('准备过程不发送导航/拍照目标。')
        return 0
    if not args.no_clock_hold and os.environ.get('AGT_FIELD_MODE_CLOCK_SCOPE') != 'foreground':
        # Reject an existing split stack before changing any clock service.
        preflight = Readiness()
        try:
            preflight.reject_existing(args.mode)
        finally:
            preflight.close()
        print('在当前终端保持本轮系统时钟；如需密码，请在这里输入。',flush=True)
        command = foreground_clock_command(argv)
        os.execvp(command[0],command)
    (ROOT/'work').mkdir(exist_ok=True)
    with (ROOT/'work/field_mode.lock').open('w') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise ValueError('另一个 field_start 模式已运行；原会话未改变')
        session = ModeSession(plan, Jobs(ROOT))
        probe = Readiness(session.check)
        directory = ROOT/'runs'/('field_mode_'+utc_now().replace(':','').replace('.','_'))
        write_json(directory/'session.json',dict(plan=plan,state='STARTING'))
        publish_mode_state(ROOT,args.mode,plan['map_id'],'STARTING')
        def interrupted(_signum, _frame):
            raise KeyboardInterrupt
        previous = signal.signal(signal.SIGTERM, interrupted)
        try:
            probe.reject_existing(args.mode)
            print('组合模式 '+args.mode+'；保持机器人静止。',flush=True)
            session.start('navigation_stack')
            probe.wait(probe.fresh_odom,args.startup_timeout_sec,'等待新鲜本地里程计')
            probe.activate(map_only=True)
            if args.mode == 'photo':
                session.start('camera')  # start after the core clock hold has taken effect
            if not probe.localized(plan['map_id']):
                if args.manual_localization:
                    print('请另开终端运行 ./field_trial relocalize',flush=True)
                else:
                    probe.wait(probe.stationary,args.startup_timeout_sec,'等待实测停车后请求一次全局重定位')
                    probe.relocalize(args.relocalization_timeout_sec)
            probe.wait(lambda:probe.localized(plan['map_id']),args.startup_timeout_sec,'等待有效全局定位')
            probe.activate()
            for component in plan['components']:
                if component['name'] not in ('navigation_stack','camera'):
                    session.start(component['name'])
            if args.mode in ('photo','distance'):
                probe.mission_ready(args.mode,args.startup_timeout_sec)
            write_json(directory/'session.json',dict(plan=plan,state='READY',owned=session.owned))
            publish_mode_state(ROOT,args.mode,plan['map_id'],'READY')
            print('导航已激活。'+({'photo':'在 RViz 排队后执行 ./field_photo start。',
                  'distance':'使用 ./field_distance teach home、accuracy、run；不会自动发车。',
                  'nav':'可在 RViz 手动指定导航目标。'}[args.mode]),flush=True)
            while True:
                probe.health_check()
                probe.rclpy.spin_once(probe.node,timeout_sec=.1)
        except KeyboardInterrupt:
            print('退出当前组合模式，只停止本次启动的组件。',flush=True)
            return 130
        finally:
            old_int = signal.signal(signal.SIGINT,signal.SIG_IGN)
            signal.signal(signal.SIGTERM,signal.SIG_IGN)
            final_state = 'STOPPED'
            try:
                publish_mode_state(ROOT,args.mode,plan['map_id'],'STOPPING')
                session.close()
            except Exception:
                final_state = 'STOP_FAILED'
                raise
            finally:
                probe.close()
                write_json(directory/'session.json',dict(plan=plan,state=final_state,owned=session.owned))
                publish_mode_state(ROOT,args.mode,plan['map_id'],final_state)
                signal.signal(signal.SIGINT,old_int)
                signal.signal(signal.SIGTERM,previous)


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (ValueError,OSError) as exc:
        raise SystemExit(str(exc))
