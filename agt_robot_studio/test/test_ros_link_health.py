"""Read-only adapter data integrity tests using local fake messages/clocks."""
import sys
from types import SimpleNamespace as NS, ModuleType

import pytest

from agt_robot_studio.ros_worker import RosWorker, TopicRate


def fake_clock(seconds=100.2):
    return NS(get_clock=lambda: NS(now=lambda: NS(nanoseconds=int(seconds*1e9))))


def odometry(stamp=100, quaternion=(0., 0., 0., 1.)):
    return NS(header=NS(stamp=NS(sec=stamp, nanosec=0)),
              pose=NS(pose=NS(position=NS(x=1., y=2.),
                              orientation=NS(**dict(zip(('x','y','z','w'),quaternion))))))


def test_topic_rate_snapshot_exposes_age_and_prunes_old_window():
    rate = TopicRate(3.)
    assert rate.snapshot(0.) == {'rate_hz': None, 'age_s': None}
    for now in (0., .1, .2):
        rate.add(now)
    assert rate.snapshot(.2) == {'rate_hz': 10., 'age_s': 0.}
    assert rate.snapshot(3.15)['rate_hz'] is None
    assert rate.snapshot(3.15)['age_s'] == pytest.approx(2.95)


def test_odom_source_age_expiration_and_bad_messages_do_not_refresh_pose(monkeypatch):
    clock = [200.]
    monkeypatch.setattr('agt_robot_studio.ros_worker.time.monotonic', lambda: clock[0])
    worker = RosWorker({})
    worker._ros_node = fake_clock()
    try:
        worker._on_odom(odometry())
        state = worker._state_snapshot(clock[0])
        assert state['odom'] == (1., 2., 0.)
        assert state['odom_age_s'] == pytest.approx(.2)
        clock[0] = 201.4
        assert worker._state_snapshot(clock[0])['odom'] is None
        worker._on_odom(odometry(stamp=98))
        assert worker.odom_seen == 200.
        assert '时间戳' in worker.odom_error
        worker._on_odom(odometry(quaternion=(0.,0.,0.,0.)))
        assert '四元数' in worker.odom_error
        assert worker._state_snapshot(clock[0])['odom'] is None
        worker._on_odom(odometry())
        assert worker.odom_error is None
        assert worker._state_snapshot(clock[0])['odom'] is not None
    finally:
        worker.shutdown()


def test_invalid_gimbal_feedback_is_not_offered_as_actual_angles(monkeypatch):
    clock = [100.]
    monkeypatch.setattr('agt_robot_studio.ros_worker.time.monotonic', lambda: clock[0])
    worker = RosWorker({})
    try:
        worker._on_gimbal(NS(heading=10., pitch=-20., roll=0.))
        clock[0] += .2
        worker._on_gimbal(NS(heading=float('nan'), pitch=-20., roll=0.))
        state = worker._state_snapshot(clock[0])
        assert state['gimbal_angles'] is None
        assert state['gimbal_age_s'] == pytest.approx(.2)
        assert state['topic_status']['gimbal']['age_s'] == 0.
        assert '非有限' in state['topic_status']['gimbal']['error']
        worker._on_gimbal(NS(heading=11., pitch=-20., roll=0.))
        assert worker._state_snapshot(clock[0])['gimbal_angles']['heading'] == 11.
    finally:
        worker.shutdown()


def test_camera_source_rate_counts_messages_before_preview_throttling(monkeypatch):
    clock = [100.]
    monkeypatch.setattr('agt_robot_studio.ros_worker.time.monotonic', lambda: clock[0])
    worker = RosWorker({'camera_preview_hz': 1., 'camera_image_topic': '/custom/image'})
    try:
        for i in range(10):
            clock[0] = 100. + i*.1
            worker._on_image(object())
        state = worker._state_snapshot(clock[0])
        assert state['topic_status']['camera']['rate_hz'] == pytest.approx(10.)
        assert state['topic_status']['camera']['topic'] == '/custom/image'
        assert worker._image_queue.qsize() == 1
        assert worker._image_queue.get_nowait()[1] == 100.
        assert state['camera_image_age_s'] is None
    finally:
        worker.shutdown()


def test_missing_message_package_remains_visible_in_topic_diagnostics():
    worker = RosWorker({})
    try:
        worker.topic_errors['gimbal'] = 'message package missing'
        assert worker._state_snapshot(100.)['topic_status']['gimbal']['error'] == 'message package missing'
        state = worker._state_snapshot(100., connected=False)
        assert not state['connected']
        assert state['odom'] is state['gimbal_angles'] is state['map_robot'] is None
    finally:
        worker.shutdown()


def test_recovered_map_tf_clears_its_previous_error(monkeypatch):
    fake_time = ModuleType('rclpy.time')
    fake_time.Time = lambda: None
    monkeypatch.setitem(sys.modules, 'rclpy.time', fake_time)
    worker = RosWorker({})
    worker._ros_node = fake_clock()
    transform = NS(header=NS(stamp=NS(sec=98, nanosec=0)), transform=NS(
        translation=NS(x=1., y=2., z=0.), rotation=NS(x=0., y=0., z=0., w=1.)))
    worker.buffer = NS(lookup_transform=lambda *_:transform)
    try:
        assert worker._map_robot() is None
        assert worker.map_robot_error
        transform.header.stamp.sec = 100
        pose = worker._map_robot()
        assert pose['source_age_s'] == pytest.approx(.2)
        assert worker.map_robot_error is worker.display_error is None
        transform.transform.rotation.w = 0.
        assert worker._map_robot() is None
        assert 'quaternion' in worker.map_robot_error
    finally:
        worker.shutdown()


def test_normal_adapter_exit_emits_disconnected_without_real_ros(monkeypatch):
    def module(name, **attrs):
        value = ModuleType(name)
        value.__dict__.update(attrs)
        monkeypatch.setitem(sys.modules, name, value)
        return value

    class Node:
        def __init__(self, _name): pass
        def create_subscription(self, *args, **kwargs): pass
        def destroy_node(self): pass

    class Executor:
        def add_node(self, node): pass
        def shutdown(self, **kwargs): pass

    module('rclpy', init=lambda **_:None, ok=lambda:False, shutdown=lambda:None)
    module('rclpy.node', Node=Node)
    module('rclpy.executors', SingleThreadedExecutor=Executor)
    module('rclpy.qos', QoSProfile=lambda **kwargs:NS(**kwargs),
           DurabilityPolicy=NS(TRANSIENT_LOCAL=1, VOLATILE=0),
           ReliabilityPolicy=NS(RELIABLE=1, BEST_EFFORT=0), qos_profile_sensor_data=object())
    module('nav_msgs.msg', Odometry=object, OccupancyGrid=object)
    module('sensor_msgs.msg', PointCloud2=object, Imu=object, Image=object)
    module('std_msgs.msg', String=object)
    module('livox_ros_driver2.msg', CustomMsg=object)
    module('pantilt_camera_serial.msg', PantiltAngleInfo=object)
    module('tf2_ros', Buffer=NS, TransformListener=lambda *_:None)
    worker = RosWorker(dict(odom_topic='/odom', raw_lidar_topic='/lidar', imu_topic='/imu',
                            map_topic='/map', points_topic='/cloud', live_points_topic='/live'))
    states = []
    worker.state.connect(states.append)
    worker.shutdown()  # Setup can complete, but the executor loops never run.
    worker.run()
    assert states and states[-1]['connected'] is False
    assert states[-1]['adapter_error'] is None
