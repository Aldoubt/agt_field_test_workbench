#!/usr/bin/env python3
"""Isolated Humble wire test for field_trial nav; no chassis or velocity publisher."""
import os
if os.environ.get('ROS_DOMAIN_ID', '0') == '0':
    raise SystemExit('Run this test in an explicit isolated ROS_DOMAIN_ID, never domain 0')

import threading
import rclpy
from rclpy.context import Context
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from lifecycle_msgs.srv import GetState
from nav2_msgs.srv import ManageLifecycleNodes
from workbench.field_trial import activate_navigation, require_navigation_ready
from agt_robot_interfaces.msg import LocalizationStatus
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


def main():
    context = Context()
    rclpy.init(context=context)
    server = Node('field_trial_lifecycle_test_services', context=context)
    states = {name: 1 for name in ('map_server', 'controller_server', 'bt_navigator')}
    requests = []
    services = []
    for name in states:
        def get_state(request, response, name=name):
            response.current_state.id = states[name]
            response.current_state.label = 'active' if states[name] == 3 else 'unconfigured'
            return response
        services.append(server.create_service(GetState, '/'+name+'/get_state', get_state))
    for manager, names in (('lifecycle_manager_map', ('map_server',)),
                           ('lifecycle_manager_navigation', ('controller_server', 'bt_navigator'))):
        def startup(request, response, manager=manager, names=names):
            assert request.command == ManageLifecycleNodes.Request.STARTUP
            if manager == 'lifecycle_manager_navigation':
                assert states['map_server'] == 3, 'map must activate before navigation'
            requests.append(manager)
            for name in names:
                states[name] = 3
            response.success = True
            return response
        services.append(server.create_service(ManageLifecycleNodes, '/'+manager+'/manage_nodes', startup))
    executor = SingleThreadedExecutor(context=context)
    status_pub = server.create_publisher(LocalizationStatus, '/agt/localization/status', 10)
    odom_pub = server.create_publisher(Odometry, '/agt/odometry/local', 10)
    broadcaster = TransformBroadcaster(server)
    publish_inputs = [False]
    def inputs():
        if not publish_inputs[0]:
            return
        stamp = server.get_clock().now().to_msg()
        status = LocalizationStatus(stamp=stamp, state=3, local_odom_fresh=True,
                                    global_correction_valid=True)
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.orientation.w = 1.
        status_pub.publish(status)
        odom_pub.publish(odom)
        transforms = []
        for parent, child in [('map', 'odom'), ('odom', 'base_link')]:
            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = parent
            transform.child_frame_id = child
            transform.transform.rotation.w = 1.
            transforms.append(transform)
        broadcaster.sendTransform(transforms)
    timer = server.create_timer(.05, inputs)
    executor.add_node(server)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        activate_navigation(map_only=True)
        assert requests == ['lifecycle_manager_map']
        assert states['map_server'] == 3 and states['controller_server'] == 1
        rclpy.init()
        probe = Node('missing_odom_probe')
        try:
            try:
                require_navigation_ready(probe, timeout_sec=.25)
                raise AssertionError('missing localization must reject Nav2 startup')
            except ValueError as exc:
                assert '未发送 Nav2 STARTUP' in str(exc)
        finally:
            probe.destroy_node()
            rclpy.shutdown()
        assert requests == ['lifecycle_manager_map']
        publish_inputs[0] = True
        activate_navigation()
        assert requests == ['lifecycle_manager_map', 'lifecycle_manager_navigation']
        assert all(value == 3 for value in states.values())
        activate_navigation()
        assert len(requests) == 2, 'already ACTIVE navigation must not be restarted'
        print('PASS: map before Nav2; repeated activation leaves ACTIVE nodes running')
    finally:
        executor.shutdown()
        thread.join(timeout=3)
        server.destroy_node()
        rclpy.shutdown(context=context)


if __name__ == '__main__':
    main()
