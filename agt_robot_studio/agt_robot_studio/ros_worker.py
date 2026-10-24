"""Read-only ROS visualization adapter for AGT Robot Studio.

All motion, relocalization and session operations go through the existing Field
Test Workbench bridge. This thread never publishes velocity or owns a control
clock; sensor/display failures do not terminate the Qt application.
"""
import math
import json
import queue
import threading
import time
from collections import deque
from PyQt5.QtCore import QThread, pyqtSignal
from .cloud import xyz_from_cloud
from .visualization import LatestFrames, cloud_snapshot, transform_xyz
from .map_io import occupancy_to_gray


class TopicRate:
    """Arrival-time frequency over a short rolling window; no nominal-Hz assumption."""
    def __init__(self, window_sec=3.0):
        self.window_sec = float(window_sec)
        self.samples = deque()
        self.last = None
        self._lock = threading.Lock()

    def add(self, now=None):
        now = time.monotonic() if now is None else float(now)
        with self._lock:
            self.samples.append(now)
            self.last = now
            while self.samples and now-self.samples[0] > self.window_sec:
                self.samples.popleft()

    def hz(self, now=None):
        now = time.monotonic() if now is None else float(now)
        with self._lock:
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
        self.self_filter_status = None
        self.self_filter_seen = 0.0
        self.last_report = 0.0
        self.last_cloud = 0.0
        self.last_map_cloud = 0.0
        self.rates = {name: TopicRate() for name in ('lidar', 'imu', 'odom')}
        self._decode_queue = queue.Queue(maxsize=1)
        self._decode_stop = threading.Event()
        self._decode_thread = None
        self.frames = LatestFrames()
        self._grid_queue = queue.Queue(maxsize=1)
        self.buffer = None
        self.display_error = None
        self.gimbal_angles=None;self.gimbal_seen=0.
        # Dedicated ROS executor: preview/map serialization must never starve 200Hz IMU.
        self._rate_stop = threading.Event()
        self._rate_thread = None

    def shutdown(self):
        self.alive = False
        self._decode_stop.set()
        self._rate_stop.set()

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
            except (queue.Empty, queue.Full):
                pass

    def _decode_loop(self):
        while not self._decode_stop.is_set():
            try:
                self._decode_grid(self._grid_queue.get_nowait())
            except queue.Empty:
                pass
            except Exception as ex:
                self.display_error = '栅格无效，保留上帧: ' + str(ex)
            try:
                msg, is_map = self._decode_queue.get(timeout=.2)
            except queue.Empty:
                continue
            try:
                stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
                if not is_map:
                    age=self._ros_node.get_clock().now().nanoseconds/1e9-stamp
                    if stamp<=0 or not -.1<=age<=2.:
                        raise ValueError('live cloud timestamp missing/stale')
                pts = xyz_from_cloud(msg, limit=6500)
                frame = msg.header.frame_id
                target = self.cfg.get('map_frame', 'map')
                if not frame:
                    raise ValueError('cloud frame_id missing')
                if frame != target:
                    from rclpy.time import Time
                    transform = self.buffer.lookup_transform(target, frame, Time.from_msg(msg.header.stamp))
                    pts = transform_xyz(pts, transform.transform)
                snapshot = cloud_snapshot(pts, target, '全局点云' if is_map else '实时点云')
                snapshot['stamp_sec']=stamp
                self.cloud_points = len(pts)
                self.frames.put('cloud', snapshot)
                self.display_error = None
            except Exception as ex:
                self.display_error = '点云/TF 无效，保留上帧: ' + str(ex)
                self.notice.emit('点云显示已跳过异常帧：' + repr(ex))

    def _on_self_filter_status(self, msg):
        try:
            value = json.loads(msg.data)
            if isinstance(value,dict) and isinstance(value.get('state'),str):
                self.self_filter_status = value
                self.self_filter_seen = time.monotonic()
        except (ValueError,AttributeError,TypeError):
            pass

    def _on_grid(self, msg):
        try:
            self._grid_queue.put_nowait(msg)
        except queue.Full:
            try:
                self._grid_queue.get_nowait()
                self._grid_queue.put_nowait(msg)
            except (queue.Empty, queue.Full):
                pass

    def _decode_grid(self, msg):
        try:
            if msg.header.frame_id != self.cfg.get('map_frame', 'map'):
                raise ValueError('occupancy grid must use the configured map frame')
            q = msg.info.origin.orientation
            if not all(math.isfinite(v) for v in (msg.info.origin.position.x,msg.info.origin.position.y,q.x,q.y,q.z,q.w)) or abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1.)>1e-3:
                raise ValueError('invalid grid origin/rotation')
            yaw = self._yaw(q)
            if not math.isfinite(msg.info.resolution) or msg.info.resolution <= 0:
                raise ValueError('invalid grid resolution')
            if not 0 < msg.info.width * msg.info.height <= 16_000_000:
                raise ValueError('invalid grid dimensions')
            gray = occupancy_to_gray(msg.data, msg.info.width, msg.info.height)
            self.frames.put('grid', {'width': msg.info.width, 'height': msg.info.height,
                            'gray': gray, 'resolution': msg.info.resolution,
                            'received': time.monotonic(),
                            'frame_id': msg.header.frame_id,
                            'origin': (msg.info.origin.position.x, msg.info.origin.position.y, yaw)})
        except Exception as ex:
            self.display_error = '栅格无效，保留上帧: ' + str(ex)
            self.notice.emit('地图预览失败：' + repr(ex))

    def _map_robot(self):
        if self.buffer is None:
            return None
        try:
            from rclpy.time import Time
            tf = self.buffer.lookup_transform(self.cfg.get('map_frame', 'map'), 'base_link', Time())
            stamp = tf.header.stamp.sec + tf.header.stamp.nanosec/1e9
            age = self._ros_node.get_clock().now().nanoseconds/1e9 - stamp
            if stamp == 0 or not -.1 <= age <= .8:
                raise ValueError('map → base_link TF stale')
            t = tf.transform
            pose = dict(x=t.translation.x, y=t.translation.y, z=t.translation.z,
                        yaw=self._yaw(t.rotation), frame_id=self.cfg.get('map_frame', 'map'),
                        received=time.monotonic())
            if not all(math.isfinite(pose[k]) for k in ('x','y','z','yaw')):
                raise ValueError('invalid robot TF')
            return pose
        except Exception as ex:
            self.display_error = '机器人 TF 不可用: ' + str(ex)
            return None

    def _emit_state(self):
        now = time.monotonic()
        if now - self.last_report < .3:
            return
        self.last_report = now
        self.state.emit({'connected': True, 'odom': self.odom,
                         'gimbal_angles':self.gimbal_angles if now-self.gimbal_seen<1.5 else None,
                         'map_robot': self._map_robot(), 'display_error': self.display_error,
                         'odom_age_s': now-self.odom_seen if self.odom else None,
                         'cloud_points': self.cloud_points,
                         'self_filter': self.self_filter_status
                             if now-self.self_filter_seen < 3. else None,
                         'rates': {name: rate.hz(now) for name, rate in self.rates.items()}})

    def run(self):
        node = None
        rate_node = None
        rate_executor = None
        try:
            import rclpy
            from rclpy.node import Node
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
            from nav_msgs.msg import Odometry, OccupancyGrid
            from sensor_msgs.msg import PointCloud2, Imu
            from std_msgs.msg import String
            try:
                from livox_ros_driver2.msg import CustomMsg
            except ImportError:
                CustomMsg=None
            from tf2_ros import Buffer, TransformListener
            rclpy.init(args=None)
            node = Node('agt_robot_studio_gui')
            try:
                from pantilt_camera_serial.msg import PantiltAngleInfo
                def on_angles(msg):
                    self.gimbal_angles={axis:float(getattr(msg,axis)) for axis in ('heading','pitch','roll')}
                    self.gimbal_seen=time.monotonic()
                node.create_subscription(PantiltAngleInfo,'/pantilt_camera_serial0/pantilt_angle_info',on_angles,qos_profile_sensor_data)
            except ImportError:
                pass
            self._ros_node = node
            self.buffer = Buffer()
            listener = TransformListener(self.buffer, node)
            rate_node = Node('agt_robot_studio_sensor_rates')
            map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                 reliability=ReliabilityPolicy.RELIABLE)
            node.create_subscription(Odometry, self.cfg['odom_topic'], self._on_odom, qos_profile_sensor_data)
            # Rate accounting needs only message arrival, not 400KB+ of decoded
            # CustomPoint objects. Full Python deserialization at 10 Hz starves
            # the IMU callback on the same executor (measured ~81 vs ~203 Hz).
            if CustomMsg is not None:
                rate_node.create_subscription(CustomMsg, self.cfg['raw_lidar_topic'],
                                              lambda _m: self.rates['lidar'].add(),
                                              qos_profile_sensor_data, raw=True)
            else:self.notice.emit('Livox 消息包缺失：雷达频率不可用；其余实时视图继续运行')
            rate_node.create_subscription(Imu, self.cfg['imu_topic'],
                                          lambda _m: self.rates['imu'].add(), qos_profile_sensor_data)
            node.create_subscription(
                String,self.cfg.get('self_filter_status_topic','/mapping/self_filter/status'),
                self._on_self_filter_status,10)
            node.create_subscription(OccupancyGrid, self.cfg['map_topic'], self._on_grid, map_qos)
            node.create_subscription(PointCloud2, self.cfg['points_topic'], lambda m:self._on_cloud(m, True), map_qos)
            node.create_subscription(PointCloud2, self.cfg['live_points_topic'],
                                     lambda m:self._on_cloud(m, False), qos_profile_sensor_data)
            # Read-only FAST-LIO2 preview; no additional mapping, driver or TF process.
            node.create_subscription(PointCloud2, '/fastlio2/world_cloud',
                                     lambda m:self._on_cloud(m, False), qos_profile_sensor_data)
            node.create_subscription(PointCloud2, '/agt/livox/points',
                                     lambda m:self._on_cloud(m, False), qos_profile_sensor_data)
            rate_executor = SingleThreadedExecutor()
            rate_executor.add_node(rate_node)

            def sample_sensor_rates():
                while self.alive and not self._rate_stop.is_set() and rclpy.ok():
                    try:
                        rate_executor.spin_once(timeout_sec=.01)
                    except Exception as ex:
                        self.notice.emit('传感器显示采样异常（不影响驱动）：' + repr(ex))
                        if self._rate_stop.wait(.1):
                            break

            self._rate_thread = threading.Thread(target=sample_sensor_rates,
                                                  name='studio-sensor-rates', daemon=True)
            self._rate_thread.start()
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
            self._rate_stop.set()
            self._decode_stop.set()
            if self._decode_thread is not None:
                self._decode_thread.join(timeout=2.0)
            if self._rate_thread is not None:
                self._rate_thread.join(timeout=2.0)
            try:
                if rate_executor is not None:
                    rate_executor.shutdown(timeout_sec=1.0)
                if rate_node is not None:
                    rate_node.destroy_node()
                if node is not None:
                    node.destroy_node()
                import rclpy
                if rclpy.ok():
                    rclpy.shutdown()
            except Exception:
                pass
