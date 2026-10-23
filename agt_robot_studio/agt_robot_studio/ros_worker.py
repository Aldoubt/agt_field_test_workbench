"""All ROS interaction is confined to one QThread; UI never publishes directly."""
import math
import queue
import time
from PyQt5.QtCore import QThread, pyqtSignal
from .cloud import xyz_from_cloud
from .control import ClosedLoopMotion, Limits


class RosWorker(QThread):
    state = pyqtSignal(object)
    cloud = pyqtSignal(object)
    grid = pyqtSignal(object)
    notice = pyqtSignal(str)

    def __init__(self, cfg):
        super().__init__()
        self.cfg = dict(cfg)
        self.actions = queue.Queue()
        self.alive = True
        self.motion = ClosedLoopMotion(Limits(
            max_linear=float(cfg['max_linear_mps']),
            max_angular=float(cfg['max_angular_rps']),
            max_acceleration=float(cfg['max_acceleration_mps2'])))
        self.odom = None
        self.odom_seen = 0.0
        self.safe_seen = self.obstacle_seen = 0.0
        self.safe = self.clear = False
        self.operator_armed = False
        self.active_since = 0.0
        self.last_motion_step = time.monotonic()
        self.last_cloud = 0.0
        self.last_map_cloud = 0.0
        self.cloud_size = 0
        self.last_report = 0.0
        self.last_progress = 0.0
        self.last_command = (0.0, 0.0)
        self.node = None
        self.publisher = None

    def send(self, name, **kwargs):
        self.actions.put((name, kwargs))

    def shutdown(self):
        self.alive = False
        self.send('stop')

    @staticmethod
    def _yaw(q):
        return math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z))

    def _on_odom(self, msg):
        p = msg.pose.pose
        self.odom = (p.position.x, p.position.y, self._yaw(p.orientation))
        self.odom_seen = time.monotonic()

    def _on_permitted(self, msg):
        self.safe = bool(msg.data)
        self.safe_seen = time.monotonic()

    def _on_obstacle(self, msg):
        self.clear = bool(msg.data)
        self.obstacle_seen = time.monotonic()

    def _on_cloud(self, msg, global_map=False):
        now = time.monotonic()
        if now - (self.last_map_cloud if global_map else self.last_cloud) < (1.4 if global_map else 0.6):
            return
        if global_map:
            self.last_map_cloud = now
        else:
            self.last_cloud = now
        try:
            pts = xyz_from_cloud(msg, limit=6500)
        except (ValueError, TypeError, AttributeError) as e:
            self.notice.emit('点云解码失败: ' + str(e))
            return
        self.cloud_size = len(pts)
        self.cloud.emit({'xyz': pts, 'source': 'GLIM 全局地图' if global_map else 'GLIM 实时点云'})

    def _on_grid(self, msg):
        try:
            self.grid.emit({'width': msg.info.width, 'height': msg.info.height,
                'data': list(msg.data), 'resolution': msg.info.resolution,
                'origin': (msg.info.origin.position.x, msg.info.origin.position.y, 0.0)})
        except Exception as e:
            self.notice.emit('地图读取失败: ' + str(e))

    def _publish(self, vx=0.0, wz=0.0):
        from geometry_msgs.msg import Twist
        msg = Twist()
        msg.linear.x, msg.angular.z = float(vx), float(wz)
        self.publisher.publish(msg)
        self.last_command = (vx, wz)

    def _guards(self):
        now = time.monotonic()
        odom_ok = self.odom is not None and now-self.odom_seen < float(self.cfg['odom_max_age_s'])
        age = float(self.cfg['safety_max_age_s'])
        permission_ok = self.safe and now-self.safe_seen < age
        obstacle_ok = self.clear and now-self.obstacle_seen < age
        return {'odom': odom_ok, 'motion_guard': permission_ok,
                'obstacle_clear': obstacle_ok, 'operator': self.operator_armed}

    def _stop(self, reason):
        if self.motion.kind:
            self.notice.emit('运动已停止：' + reason)
        self.motion.reset()
        if self.publisher is not None:
            self._publish()

    def _handle_actions(self):
        while True:
            try:
                action, args = self.actions.get_nowait()
            except queue.Empty:
                return
            if action == 'stop':
                self._stop('用户停止或退出')
            elif action == 'arm':
                self.operator_armed = bool(args.get('value'))
                if not self.operator_armed:
                    self._stop('人工解除使能')
            elif action == 'move':
                guards = self._guards()
                if not all(guards.values()):
                    self.notice.emit('拒绝运动，未通过：' + ', '.join(k for k,v in guards.items() if not v))
                    continue
                if self.motion.kind:
                    self.notice.emit('拒绝新的运动：已有运动任务')
                    continue
                kind, target = args['kind'], float(args['target'])
                if not math.isfinite(target) or (kind == 'distance' and not 0.02 <= abs(target) <= 5.0) or (kind == 'rotation' and not 0.02 <= abs(target) <= math.pi):
                    self.notice.emit('拒绝运动：目标超出限制')
                    continue
                self.motion.begin(kind, target, self.odom)
                self.active_since = time.monotonic()
                self.last_motion_step = self.active_since
                self.notice.emit('运动任务开始，实时监控安全反馈')

    def _control_step(self):
        now = time.monotonic()
        if self.motion.kind is None:
            return
        if not all(self._guards().values()):
            self._stop('安全信号/里程计失效')
            return
        if now - self.active_since > float(self.cfg['motion_timeout_s']):
            self._stop('目标超时')
            return
        dt = now-self.last_motion_step
        if dt < 0.05:
            return
        self.last_motion_step = now
        try:
            vx, wz, self.last_progress, done = self.motion.tick(self.odom, dt)
        except ValueError:
            self._stop('里程计无效')
            return
        if done:
            self._stop('距离或角度达标')
        else:
            self._publish(vx, wz)

    def _emit_state(self):
        now = time.monotonic()
        if now-self.last_report < .25:
            return
        self.last_report = now
        self.state.emit({'connected': True, 'guards': self._guards(),
                         'odom': self.odom, 'moving': self.motion.kind,
                         'progress': self.last_progress, 'target': self.motion.target,
                         'cloud_points': self.cloud_size, 'command': self.last_command,
                         'command_topic': self.cfg['command_topic']})

    def run(self):
        try:
            import rclpy
            from rclpy.node import Node
            from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
            from nav_msgs.msg import Odometry, OccupancyGrid
            from sensor_msgs.msg import PointCloud2
            from std_msgs.msg import Bool
            from geometry_msgs.msg import Twist
            rclpy.init(args=None)
            node = Node('agt_robot_studio_gui')
            self.node = node
            self.publisher = node.create_publisher(Twist, self.cfg['command_topic'], 10)
            node.create_subscription(Odometry, self.cfg['odom_topic'], self._on_odom, qos_profile_sensor_data)
            node.create_subscription(Bool, self.cfg['safety_permitted_topic'], self._on_permitted, qos_profile_sensor_data)
            node.create_subscription(Bool, self.cfg['obstacle_clear_topic'], self._on_obstacle, qos_profile_sensor_data)
            grid_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                  reliability=ReliabilityPolicy.RELIABLE)
            node.create_subscription(OccupancyGrid, self.cfg['map_topic'], self._on_grid, grid_qos)
            cloud_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                   reliability=ReliabilityPolicy.RELIABLE)
            node.create_subscription(PointCloud2, self.cfg['points_topic'],
                                     lambda m: self._on_cloud(m, True), cloud_qos)
            node.create_subscription(PointCloud2, self.cfg['live_points_topic'],
                                     lambda m: self._on_cloud(m, False), qos_profile_sensor_data)
            self.notice.emit('ROS 已连接，等待各传感器数据')
            while self.alive and rclpy.ok():
                rclpy.spin_once(node, timeout_sec=.035)
                self._handle_actions()
                self._control_step()
                self._emit_state()
        except Exception as e:
            self.notice.emit('ROS 工作线程异常：' + repr(e))
            self.state.emit({'connected': False, 'guards': {}, 'odom': None, 'moving': None})
        finally:
            try:
                self._stop('ROS 线程退出')
                if self.node:
                    self.node.destroy_node()
                import rclpy
                if rclpy.ok():
                    rclpy.shutdown()
            except Exception:
                pass
