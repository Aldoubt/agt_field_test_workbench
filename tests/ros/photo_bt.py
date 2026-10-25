"""Run Humble's real BT navigator with fake planner/controller, private domain only."""
import os
from pathlib import Path
import subprocess
import threading
import time



def test_humble_photo_tree_success_and_failure(tmp_path):
    assert os.environ.get('ROS_DOMAIN_ID') == '184'
    assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
    import rclpy
    from action_msgs.msg import GoalStatus
    from geometry_msgs.msg import TransformStamped
    from lifecycle_msgs.srv import ChangeState
    from nav_msgs.msg import Odometry
    from nav2_msgs.action import ComputePathToPose, FollowPath, NavigateToPose
    from nav2_msgs.srv import IsPathValid
    from rclpy.action import ActionClient, ActionServer
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from tf2_ros import StaticTransformBroadcaster

    rclpy.init()
    node = rclpy.create_node('photo_bt_fake_peers')
    group = ReentrantCallbackGroup()
    goals = []
    fail = [False]

    def plan(handle):
        result = ComputePathToPose.Result()
        result.path.header = handle.request.goal.header
        result.path.poses = [handle.request.goal, handle.request.goal]
        handle.succeed()
        return result

    def follow(handle):
        goals.append(handle.request)
        if fail[0]:
            handle.abort()
        else:
            handle.succeed()
        return FollowPath.Result()

    servers = [ActionServer(node, ComputePathToPose, '/compute_path_to_pose', plan, callback_group=group),
               ActionServer(node, FollowPath, '/follow_path', follow, callback_group=group)]
    def valid(request, response):
        response.is_valid = bool(request.path.poses)
        return response
    service = node.create_service(IsPathValid, '/is_path_valid', valid, callback_group=group)
    broadcaster = StaticTransformBroadcaster(node)
    tf = TransformStamped()
    tf.header.stamp = node.get_clock().now().to_msg()
    tf.header.frame_id, tf.child_frame_id = 'map', 'base_link'
    tf.transform.rotation.w = 1.
    broadcaster.sendTransform(tf)
    pub = node.create_publisher(Odometry, '/odom', 10)
    def odom():
        msg = Odometry()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.child_frame_id = 'base_link'
        msg.pose.pose.orientation.w = 1.
        pub.publish(msg)
    timer = node.create_timer(.05, odom, callback_group=group)
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    tree = Path(__file__).resolve().parents[2]/'config/photo_navigate.xml'
    logfile = (tmp_path/'navigator.log').open('w')
    proc = subprocess.Popen([
        '/opt/ros/humble/lib/nav2_bt_navigator/bt_navigator', '--ros-args',
        '-p', 'default_nav_to_pose_bt_xml:='+str(tree),
        '-p', 'default_nav_through_poses_bt_xml:='+str(tree),
        '-p', 'default_server_timeout:=100',
    ], stdout=logfile, stderr=subprocess.STDOUT)
    def wait(future, timeout=15):
        end = time.monotonic()+timeout
        while not future.done() and time.monotonic() < end:
            time.sleep(.01)
        assert future.done(), (tmp_path/'navigator.log').read_text()
        return future.result()
    try:
        state = node.create_client(ChangeState, '/bt_navigator/change_state')
        assert state.wait_for_service(timeout_sec=10.)
        for transition in (1, 3):
            request = ChangeState.Request()
            request.transition.id = transition
            assert wait(state.call_async(request)).success, (tmp_path/'navigator.log').read_text()
        client = ActionClient(node, NavigateToPose, '/navigate_to_pose', callback_group=group)
        assert client.wait_for_server(timeout_sec=5.)
        for should_fail in (False, True):
            fail[0] = should_fail
            goal = NavigateToPose.Goal()
            goal.behavior_tree = str(tree)
            goal.pose.header.frame_id = 'map'
            goal.pose.header.stamp = node.get_clock().now().to_msg()
            goal.pose.pose.position.x = 1.
            goal.pose.pose.orientation.w = 1.
            handle = wait(client.send_goal_async(goal))
            assert handle.accepted
            result = wait(handle.get_result_async())
            assert result.status == (GoalStatus.STATUS_ABORTED if should_fail else GoalStatus.STATUS_SUCCEEDED)
        assert len(goals) == 2
        assert all(g.goal_checker_id == 'photo_goal_checker' for g in goals)
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait()
        logfile.close()
        executor.shutdown(timeout_sec=3.)
        thread.join(timeout=3.)
        node.destroy_node()
        rclpy.shutdown()
