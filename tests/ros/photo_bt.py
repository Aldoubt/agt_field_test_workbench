"""Run Humble's real BT navigator with fake planner/controller, private domain only."""
import json
import os
from pathlib import Path
import subprocess
import threading
import time

import pytest

@pytest.mark.parametrize('case', [
    'success', 'follow_retry', 'planner_failure', 'follow_failure',
    'cancel_during_wait', 'invalid_path_replan',
])
def test_humble_photo_tree_bounded_retry_and_replan(tmp_path, case):
    assert os.environ.get('ROS_DOMAIN_ID') == '184'
    assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
    import rclpy
    from action_msgs.msg import GoalStatus
    from geometry_msgs.msg import TransformStamped
    from lifecycle_msgs.srv import ChangeState
    from nav_msgs.msg import Odometry
    from nav2_msgs.action import ComputePathToPose, FollowPath, NavigateToPose, Wait
    from nav2_msgs.srv import IsPathValid
    from rclpy.action import ActionClient, ActionServer, CancelResponse
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from tf2_ros import StaticTransformBroadcaster

    rclpy.init()
    node = rclpy.create_node('photo_bt_fake_peers')
    group = ReentrantCallbackGroup()
    goals, events = [], []
    counts = dict(plan=0, follow=0, wait=0, invalid=0)
    lock = threading.Lock()
    wait_started = threading.Event()
    second_follow_started = threading.Event()
    stopping = threading.Event()

    def event(kind):
        with lock:
            counts[kind] += 1
            number = counts[kind]
            events.append(dict(kind=kind, number=number, time=time.monotonic()))
            return number

    def plan(handle):
        number = event('plan')
        result = ComputePathToPose.Result()
        if case == 'planner_failure':
            handle.abort()
            return result
        result.path.header = handle.request.goal.header
        # Distinct paths exercise FollowPath's native goal-update handling.
        import copy
        waypoint = copy.deepcopy(handle.request.goal)
        waypoint.pose.position.y = number*.1
        result.path.poses = [waypoint, handle.request.goal]
        handle.succeed()
        return result

    def follow(handle):
        number = event('follow')
        goals.append(handle.request)
        if case == 'invalid_path_replan':
            if number == 1:
                # Keep the original action running while the 1 Hz validity
                # check triggers a new plan and FollowPath goal update.
                end = time.monotonic()+8.
                while not second_follow_started.is_set() and time.monotonic()<end and not stopping.is_set():
                    if handle.is_cancel_requested:
                        handle.canceled()
                        return FollowPath.Result()
                    time.sleep(.01)
                handle.abort()  # Superseded action result must be ignored.
            else:
                second_follow_started.set()
                time.sleep(.1)
                handle.succeed()
        elif case in ('follow_failure','cancel_during_wait') or (case=='follow_retry' and number==1):
            handle.abort()
        else:
            handle.succeed()
        return FollowPath.Result()

    def stationary_wait(handle):
        event('wait')
        wait_started.set()
        duration = handle.request.time.sec + handle.request.time.nanosec*1e-9
        assert duration == 1.
        end = time.monotonic()+duration
        while time.monotonic()<end and not stopping.is_set():
            if handle.is_cancel_requested:
                handle.canceled()
                return Wait.Result()
            time.sleep(.01)
        handle.succeed()
        return Wait.Result()

    servers = [ActionServer(node, ComputePathToPose, '/compute_path_to_pose', plan, callback_group=group),
               ActionServer(node, FollowPath, '/follow_path', follow, callback_group=group,
                            cancel_callback=lambda _:CancelResponse.ACCEPT),
               ActionServer(node, Wait, '/wait', stationary_wait, callback_group=group,
                            cancel_callback=lambda _:CancelResponse.ACCEPT)]
    def valid(request, response):
        invalid = case=='invalid_path_replan' and counts['follow']==1 and counts['invalid']==0
        response.is_valid = bool(request.path.poses) and not invalid
        if invalid:
            event('invalid')
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
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    tree = Path(__file__).resolve().parents[2]/'config/photo_navigate.xml'
    log_path = tmp_path/'navigator.log'
    logfile = log_path.open('w')
    proc = subprocess.Popen([
        '/opt/ros/humble/lib/nav2_bt_navigator/bt_navigator', '--ros-args',
        '-p', 'default_nav_to_pose_bt_xml:='+str(tree),
        '-p', 'default_nav_through_poses_bt_xml:='+str(tree),
        '-p', 'default_server_timeout:=100',
    ], stdout=logfile, stderr=subprocess.STDOUT)
    def wait(future, timeout=15):
        end = time.monotonic()+timeout
        while not future.done() and time.monotonic() < end:
            assert proc.poll() is None, log_path.read_text()
            time.sleep(.01)
        assert future.done(), log_path.read_text()
        return future.result()
    client = None
    status = None
    try:
        state = node.create_client(ChangeState, '/bt_navigator/change_state')
        assert state.wait_for_service(timeout_sec=10.)
        for transition in (1, 3):
            request = ChangeState.Request()
            request.transition.id = transition
            assert wait(state.call_async(request)).success, (tmp_path/'navigator.log').read_text()
        client = ActionClient(node, NavigateToPose, '/navigate_to_pose', callback_group=group)
        assert client.wait_for_server(timeout_sec=5.)
        goal = NavigateToPose.Goal()
        goal.behavior_tree = str(tree)
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = node.get_clock().now().to_msg()
        goal.pose.pose.position.x = 1.
        goal.pose.pose.orientation.w = 1.
        handle = wait(client.send_goal_async(goal))
        assert handle.accepted
        if case=='cancel_during_wait':
            assert wait_started.wait(timeout=5.), log_path.read_text()
            canceled = wait(handle.cancel_goal_async())
            assert canceled.goals_canceling
        result = wait(handle.get_result_async())
        status = result.status
        expected = GoalStatus.STATUS_SUCCEEDED
        if case in ('planner_failure','follow_failure'):
            expected = GoalStatus.STATUS_ABORTED
        elif case=='cancel_during_wait':
            expected = GoalStatus.STATUS_CANCELED
            # Outwait the recovery interval to catch a retry after cancellation.
            time.sleep(1.2)
        assert result.status == expected, log_path.read_text()
        expected_counts = {
            'success':(1,1,0,0), 'follow_retry':(2,2,1,0),
            'planner_failure':(2,0,1,0), 'follow_failure':(2,2,1,0),
            'cancel_during_wait':(1,1,1,0), 'invalid_path_replan':(2,2,0,1),
        }[case]
        assert tuple(counts.values())==expected_counts, (counts,events,log_path.read_text())
        if case in ('follow_retry','planner_failure','follow_failure'):
            first_wait = next(item for item in events if item['kind']=='wait')
            second_plan = next(item for item in events if item['kind']=='plan' and item['number']==2)
            assert second_plan['time']-first_wait['time']>=.95
        assert all(g.controller_id == 'PhotoFollowPath' for g in goals)
        assert all(g.goal_checker_id == 'photo_goal_checker' for g in goals)
        if len(goals)==2:
            # Each retry/update must actually deliver the new planner output,
            # rather than reusing an old blackboard path after halting.
            assert [g.path.poses[0].pose.position.y for g in goals]==pytest.approx([.1,.2])
    finally:
        (tmp_path/'events.json').write_text(json.dumps(dict(case=case,status=status,
            counts=counts,events=events),indent=2)+'\n')
        stopping.set()
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait()
        logfile.close()
        executor.shutdown(timeout_sec=3.)
        thread.join(timeout=3.)
        if client is not None:
            client.destroy()
        for server in servers:
            server.destroy()
        node.destroy_node()
        rclpy.shutdown()
