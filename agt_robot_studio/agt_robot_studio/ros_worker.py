"""Read-only ROS visualization adapter for AGT Robot Studio.

All motion, relocalization and session operations go through the existing Field
Test Workbench bridge. This thread never publishes velocity or owns a control
clock; sensor/display failures do not terminate the Qt application.
"""
import math
import queue
import threading
import time
from collections import deque
from PyQt5.QtCore import QThread, pyqtSignal
from .cloud import xyz_from_cloud


class TopicRate:
    """Arrival-time frequency over a short rolling window; no nominal-Hz assumption."""
    def __init__(self, window_sec=3.0):
        self.window_sec = float(window_sec)
        self.samples = deque()
        self.last = None

    def add(self, now=None):
        now = time.monotonic() if now is None else float(now)
        self.samples.append(now)
        self.last = now
        while self.samples and now-self.samples[0] > self.window_sec:
            self.samples.popleft()

    def hz(self, now=None):
        now = time.monotonic() if now is None else float(now)
        if self.last is None or now-self.last > self.window_sec or len(self.samples) < 2:
            return None
        span = self.samples[-1]-self.samples[0]
        return (len(self.samples)-1)/span if span > .05 else None


class RosWorker(QThread):
    state = pyqtSignal(object)
    cloud = pyqtSignal(object)
    grid = pyqtSignal(object)
    notice = pyqtSignal(str)

    def __init__(self, cfg):
        super().__init__()
        self.cfg = dict(cfg)
        self.alive = True
        self.odom = None
        self.odom_seen = 0.0
        self.cloud_points = 0
        self.last_report = 0.0
        self.last_cloud = 0.0
        self.last_map_cloud = 0.0
        self.rates = {name: TopicRate() for name in ('lidar', 'imu', 'odom')}
        self._decode_queue = queue.Queue(maxsize=1)
        self._decode_stop = threading.Event()
        self._decode_thread = None

    def shutdown(self):
        self.alive = False
        self._decode_stop.set()

    @staticmethod
    def _yaw(q):
        return math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z))

    def _on_odom(self, msg):
        self.rates['odom'].add()
        try:
            p = msg.pose.pose
            pose = (float(p.position.x), float(p.position.y), self._yaw(p.orientation))
            if all(math.isfinite(x) for x in pose):
                self.odom = pose
                self.odom_seen = time.monotonic()
        except (TypeError, ValueError, AttributeError, OverflowError):
            return

    def _on_cloud(self, msg, is_map=False):
        now = time.monotonic()
        key = 'last_map_cloud' if is_map else 'last_cloud'
        if now - getattr(self, key) < (1.4 if is_map else .6):
            return
        setattr(self, key, now)
        try:
            self._decode_queue.put_nowait((msg, is_map))
        except queue.Full:
            try:
                self._decode_queue.get_nowait()
                self._decode_queue.put_nowait((msg, is_map))
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
            except Exception as ex:
                self.notice.emit('点云显示已跳过异常帧：' + repr(ex))

    def _on_grid(self, msg):
        try:
            q = msg.info.origin.orientation
            yaw = self._yaw(q)
            self.grid.emit({'width': msg.info.width, 'height': msg.info.height,
                            'data': list(msg.data), 'resolution': msg.info.resolution,
                            'frame_id': msg.header.frame_id,
                            'origin': (msg.info.origin.position.x, msg.info.origin.position.y, yaw)})
        except Exception as ex:
            self.notice.emit('地图预览失败：' + repr(ex))

    def _emit_state(self):
        now = time.monotonic()
        if now - self.last_report < .3:
            return
        self.last_report = now
        self.state.emit({'connected': True, 'odom': self.odom,
                         'odom_age_s': now-self.odom_seen if self.odom else None,
                         'cloud_points': self.cloud_points,
                         'rates': {name: rate.hz(now) for name, rate in self.rates.items()}})

    def run(self):
        node = None
        try:
            import rclpy
            from rclpy.node import Node
            from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
            from nav_msgs.msg import Odometry, OccupancyGrid
            from sensor_msgs.msg import PointCloud2, Imu
            from livox_ros_driver2.msg import CustomMsg
            rclpy.init(args=None)
            node = Node('agt_robot_studio_gui')
            map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                 reliability=ReliabilityPolicy.RELIABLE)
            node.create_subscription(Odometry, self.cfg['odom_topic'], self._on_odom, qos_profile_sensor_data)
            node.create_subscription(CustomMsg, self.cfg['raw_lidar_topic'],
                                     lambda _m: self.rates['lidar'].add(), qos_profile_sensor_data)
            node.create_subscription(Imu, self.cfg['imu_topic'],
                                     lambda _m: self.rates['imu'].add(), qos_profile_sensor_data)
            node.create_subscription(OccupancyGrid, self.cfg['map_topic'], self._on_grid, map_qos)
            node.create_subscription(PointCloud2, self.cfg['points_topic'], lambda m:self._on_cloud(m, True), map_qos)
            node.create_subscription(PointCloud2, self.cfg['live_points_topic'],
                                     lambda m:self._on_cloud(m, False), qos_profile_sensor_data)
            self._decode_thread = threading.Thread(target=self._decode_loop, name='studio-cloud-display', daemon=True)
            self._decode_thread.start()
            self.notice.emit('ROS 显示适配器已连接；运动与重定位由现有 Workbench 后端执行')
            while self.alive and rclpy.ok():
                try:
                    rclpy.spin_once(node, timeout_sec=.04)
                except Exception as ex:
                    self.notice.emit('ROS 回调异常（已隔离）：' + repr(ex))
                self._emit_state()
        except Exception as ex:
            self.notice.emit('ROS 显示适配器不可用，仍可使用 Workbench 和离线地图：' + repr(ex))
            self.state.emit({'connected': False, 'odom': None})
        finally:
            self._decode_stop.set()
            try:
                if node is not None:
                    node.destroy_node()
                import rclpy
                if rclpy.ok():
                    rclpy.shutdown()
            except Exception:
                pass
