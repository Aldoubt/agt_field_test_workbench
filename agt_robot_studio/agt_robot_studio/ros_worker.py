"""ROS -> Qt adapter. Motion runs in Nav2's *separate* Behavior Server process.

This GUI neither publishes Twist nor owns a control clock. Data transport failures
must never make this UI process a safety controller. The final command gate and
hardware watchdog must live downstream of Nav2, outside this process.
"""
import math
import queue
import threading
import time
from PyQt5.QtCore import QThread, pyqtSignal
from .cloud import xyz_from_cloud


class RosWorker(QThread):
    state = pyqtSignal(object)
    cloud = pyqtSignal(object)
    grid = pyqtSignal(object)
    notice = pyqtSignal(str)

    def __init__(self, cfg):
        super().__init__()
        self.cfg = dict(cfg)
        self.actions = queue.Queue(maxsize=64)
        self.alive = True
        self.odom = None
        self.odom_seen = 0.0  # UI display only; never used to terminate process/motion
        self.guard_seen = 0.0
        self.guard_allowed = False
        self.operator_armed = False
        self.pending_goal = False
        self.goal_handle = None
        self.cancel_requested = False
        self.cancel_sent = False
        self.active_kind = None
        self.target = 0.0
        self.progress = 0.0
        self.cloud_points = 0
        self.last_report = 0.0
        self.last_cloud = 0.0
        self.last_map_cloud = 0.0
        self.node = None
        self.drive_client = None
        self.spin_client = None
        self._decode_queue = queue.Queue(maxsize=1)
        self._decode_stop = threading.Event()
        self._decode_thread = None
        self._exiting_since = None

    def send(self, name, **kwargs):
        try:
            self.actions.put_nowait((name, kwargs))
        except queue.Full:
            # Stop must never be dropped behind old button presses.
            if name == 'stop':
                try:
                    self.actions.get_nowait()
                except queue.Empty:
                    pass
                self.actions.put_nowait((name, kwargs))
            else:
                self.notice.emit('请求队列忙，忽略重复操作')

    def shutdown(self):
        # Stop is delivered before shutdown. We spin briefly for action-cancel ack.
        self.send('stop')
        self.send('shutdown')

    @staticmethod
    def _yaw(q):
        return math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z))

    def _on_odom(self, msg):
        try:
            p = msg.pose.pose
            pose = (float(p.position.x), float(p.position.y), self._yaw(p.orientation))
            if all(math.isfinite(x) for x in pose):
                self.odom = pose
                self.odom_seen = time.monotonic()
        except (TypeError, ValueError, AttributeError, OverflowError):
            return  # malformed odometry is a display error, not a process exception

    def _on_guard(self, msg):
        self.guard_allowed = bool(msg.data)
        self.guard_seen = time.monotonic()
        # Lack of external guard permission cancels *our* action. Final-stop safety
        # still comes from the independent downstream motor-command gate.
        if not self.guard_allowed:
            self._request_cancel('外部运动许可被撤销')

    def _on_cloud(self, msg, is_map=False):
        now = time.monotonic()
        key = 'last_map_cloud' if is_map else 'last_cloud'
        if now - getattr(self, key) < (1.4 if is_map else .6):
            return
        setattr(self, key, now)
        # Never decode a large cloud inside the ROS callback/action-client spinner.
        item = (msg, is_map)
        try:
            self._decode_queue.put_nowait(item)
        except queue.Full:
            try:
                self._decode_queue.get_nowait()
                self._decode_queue.put_nowait(item)
            except queue.Empty:
                pass

    def _decode_loop(self):
        while not self._decode_stop.is_set():
            try:
                msg, is_map = self._decode_queue.get(timeout=.2)
            except queue.Empty:
                continue
            try:
                pts = xyz_from_cloud(msg, limit=6500)
                self.cloud_points = len(pts)
                self.cloud.emit({'xyz': pts, 'source': 'GLIM 全局地图' if is_map else 'GLIM 实时点云'})
            except (ValueError, TypeError, AttributeError, IndexError) as ex:
                self.notice.emit('点云显示已跳过异常帧：' + str(ex))
            except Exception as ex:
                self.notice.emit('点云显示异常（不影响控制）：' + repr(ex))

    def _on_grid(self, msg):
        try:
            self.grid.emit({'width': msg.info.width, 'height': msg.info.height,
                            'data': list(msg.data), 'resolution': msg.info.resolution,
                            'origin': (msg.info.origin.position.x, msg.info.origin.position.y, 0.0)})
        except Exception as ex:
            self.notice.emit('地图预览失败（不影响运动端）：' + repr(ex))

    def _guard_ok(self):
        return (self.guard_allowed and
                time.monotonic() - self.guard_seen <= float(self.cfg.get('guard_max_age_s', 1.5)))

    def _request_cancel(self, reason):
        if self.pending_goal and self.goal_handle is None:
            self.cancel_requested = True
        if self.goal_handle is not None and not self.cancel_sent:
            try:
                self.goal_handle.cancel_goal_async()  # non-blocking
                self.cancel_sent = True
                self.notice.emit('已请求 Nav2 取消运动：' + reason)
            except Exception as ex:
                self.notice.emit('Nav2 取消请求失败，请使用实体急停：' + str(ex))
        elif self.pending_goal:
            self.notice.emit('目标尚待接受，已标记收到后取消：' + reason)

    def _clear_goal(self):
        self.pending_goal = False
        self.goal_handle = None
        self.active_kind = None
        self.cancel_requested = False
        self.cancel_sent = False
        self.target = 0.0

    def _feedback(self, msg):
        try:
            if self.active_kind == 'distance':
                self.progress = float(msg.feedback.distance_traveled)
            elif self.active_kind == 'rotation':
                self.progress = float(msg.feedback.angular_distance_traveled)
        except (TypeError, ValueError, AttributeError):
            pass

    def _accepted(self, future):
        try:
            handle = future.result()
            if handle is None or not handle.accepted:
                self.notice.emit('Nav2 拒绝目标；检查 Behavior Server、局部地图与 TF')
                self._clear_goal()
                return
            self.goal_handle = handle
            self.pending_goal = False
            self.notice.emit('Nav2 已接受任务；实际碰撞检查由 Behavior Server 执行')
            result = handle.get_result_async()
            result.add_done_callback(self._finished)
            if self.cancel_requested or not (self.operator_armed and self._guard_ok()):
                self._request_cancel('接收目标期间安全门禁失效')
        except Exception as ex:
            self.notice.emit('Nav2 目标响应异常：' + repr(ex))
            self._clear_goal()

    def _finished(self, future):
        try:
            from action_msgs.msg import GoalStatus
            result = future.result()
            outcome = ('已完成' if result.status == GoalStatus.STATUS_SUCCEEDED else
                       '已取消' if result.status == GoalStatus.STATUS_CANCELED else
                       '失败或终止 (status={})'.format(result.status))
            self.notice.emit('Nav2 定距/转角任务：' + outcome)
        except Exception as ex:
            self.notice.emit('Nav2 结果不可读：' + repr(ex))
        finally:
            self._clear_goal()

    def _start_goal(self, kind, target):
        from nav2_msgs.action import DriveOnHeading, Spin
        from builtin_interfaces.msg import Duration
        if not bool(self.cfg.get('live_motion_enabled', False)):
            self.notice.emit('运动锁定：请先完成实体安全链验收，再在配置中启用 live_motion_enabled')
            return
        if not self.operator_armed or not self._guard_ok():
            self.notice.emit('拒绝：缺少操作者使能或独立 Motion Guard 许可')
            return
        if self.pending_goal or self.goal_handle is not None:
            self.notice.emit('已有任务，先停止或等待执行完成')
            return
        if not math.isfinite(target) or (kind == 'distance' and not .02 <= abs(target) <= 5.) or (kind == 'rotation' and not .02 <= abs(target) <= math.pi):
            self.notice.emit('拒绝：运动目标无效或超出限幅')
            return
        if kind == 'distance' and target < 0 and not bool(self.cfg.get('allow_reverse', False)):
            self.notice.emit('拒绝自动倒车：后向感知与制动距离尚未验收')
            return
        client = self.drive_client if kind == 'distance' else self.spin_client
        if not client.server_is_ready():
            self.notice.emit('Nav2 动作服务器未 ACTIVE / 未发现：' + ('DriveOnHeading' if kind == 'distance' else 'Spin'))
            return
        goal = DriveOnHeading.Goal() if kind == 'distance' else Spin.Goal()
        if kind == 'distance':
            goal.target.x = target
            goal.speed = float(self.cfg.get('drive_speed_mps', .08))
        else:
            goal.target_yaw = target
        goal.time_allowance = Duration(sec=int(self.cfg.get('motion_timeout_s', 30)))
        self.target = target
        self.progress = 0.0
        self.active_kind = kind
        self.pending_goal = True
        self.cancel_requested = False
        self.cancel_sent = False
        try:
            future = client.send_goal_async(goal, feedback_callback=self._feedback)
            future.add_done_callback(self._accepted)
        except Exception as ex:
            self.notice.emit('Nav2 请求发送失败：' + repr(ex))
            self._clear_goal()

    def _handle_actions(self):
        # Bounded dispatch: an overfull UI command queue cannot starve DDS callbacks.
        for _ in range(12):
            try:
                action, args = self.actions.get_nowait()
            except queue.Empty:
                return
            try:
                if action == 'stop':
                    self._request_cancel('操作者按下停止')
                elif action == 'arm':
                    self.operator_armed = bool(args.get('value'))
                    if not self.operator_armed:
                        self._request_cancel('人工解除使能')
                elif action == 'move':
                    self._start_goal(args['kind'], float(args['target']))
                elif action == 'shutdown':
                    self._exiting_since = time.monotonic()
            except Exception as ex:
                self.notice.emit('操作请求失败（界面继续运行）：' + repr(ex))

    def _emit_state(self):
        now = time.monotonic()
        if now - self.last_report < .3:
            return
        self.last_report = now
        guard = self._guard_ok()
        if (self.active_kind or self.pending_goal) and not guard:
            if not self.cancel_requested:
                self.cancel_requested = True
                self._request_cancel('独立 Motion Guard 许可超时')
        actions_ready = (self.drive_client is not None and self.drive_client.server_is_ready() and
                         self.spin_client is not None and self.spin_client.server_is_ready())
        self.state.emit({'connected': True,
            'guards': {'nav2_behaviors': actions_ready,
                       'motion_guard': guard, 'operator': self.operator_armed,
                       'configuration': bool(self.cfg.get('live_motion_enabled', False))},
            'odom': self.odom, 'odom_age_s': now-self.odom_seen if self.odom else None,
            'moving': self.active_kind, 'progress': self.progress, 'target': self.target,
            'cloud_points': self.cloud_points,
            'command_topic': 'Nav2 Action /drive_on_heading + /spin（不直接发布 Twist）'})

    def run(self):
        node = None
        try:
            import rclpy
            from rclpy.node import Node
            from rclpy.action import ActionClient
            from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
            from nav_msgs.msg import Odometry, OccupancyGrid
            from nav2_msgs.action import DriveOnHeading, Spin
            from sensor_msgs.msg import PointCloud2
            from std_msgs.msg import Bool
            rclpy.init(args=None)
            node = Node('agt_robot_studio_gui')
            self.node = node
            self.drive_client = ActionClient(node, DriveOnHeading, self.cfg.get('drive_action', '/drive_on_heading'))
            self.spin_client = ActionClient(node, Spin, self.cfg.get('spin_action', '/spin'))
            node.create_subscription(Odometry, self.cfg['odom_topic'], self._on_odom, qos_profile_sensor_data)
            node.create_subscription(Bool, self.cfg['safety_permitted_topic'], self._on_guard, qos_profile_sensor_data)
            map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                 reliability=ReliabilityPolicy.RELIABLE)
            node.create_subscription(OccupancyGrid, self.cfg['map_topic'], self._on_grid, map_qos)
            node.create_subscription(PointCloud2, self.cfg['points_topic'], lambda m: self._on_cloud(m, True), map_qos)
            node.create_subscription(PointCloud2, self.cfg['live_points_topic'],
                                     lambda m: self._on_cloud(m, False), qos_profile_sensor_data)
            self._decode_thread = threading.Thread(target=self._decode_loop, name='studio-cloud-display', daemon=True)
            self._decode_thread.start()
            self.notice.emit('ROS 已连接；定距/旋转由独立 Nav2 Behavior Server 执行')
            while rclpy.ok():
                try:
                    rclpy.spin_once(node, timeout_sec=.04)
                except Exception as ex:
                    # A malformed sensor/visualization callback must not kill the GUI.
                    self.notice.emit('ROS 回调异常（已隔离）：' + repr(ex))
                self._handle_actions()
                try:
                    self._emit_state()
                except Exception as ex:
                    self.notice.emit('状态显示更新失败（ROS进程保持运行）：' + repr(ex))
                if self._exiting_since is not None:
                    if (self.goal_handle is None and not self.pending_goal) or time.monotonic()-self._exiting_since > 1.2:
                        break
        except Exception as ex:
            self.notice.emit('ROS 适配器不可用，Qt 界面仍可查看日志/离线地图：' + repr(ex))
            self.state.emit({'connected': False, 'guards': {}, 'odom': None, 'moving': None})
        finally:
            self._decode_stop.set()
            if self.goal_handle is not None:
                try:
                    self.goal_handle.cancel_goal_async()
                except Exception:
                    pass
            try:
                if node is not None:
                    node.destroy_node()
                import rclpy
                if rclpy.ok():
                    rclpy.shutdown()
            except Exception:
                pass
