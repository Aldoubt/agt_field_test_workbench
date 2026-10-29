"""Exercise native Humble global obstacle planning, exclusively in private DDS.

No controller, behavior server, motion guard, driver, or hardware launch is used.
Invoke with ROS_DOMAIN_ID=186 ROS_LOCALHOST_ONLY=1 and the workspace sourced.
AGT_OBSTACLE_REPLAN_EVIDENCE optionally selects the evidence directory.
"""
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import numpy as np
import pytest
import yaml


pytestmark = pytest.mark.skipif(
    os.environ.get('ROS_DOMAIN_ID') != '186' or os.environ.get('ROS_LOCALHOST_ONLY') != '1',
    reason='requires isolated domain 186 on localhost')


def _domain_processes(domain):
    """Never join a pre-existing test graph or an operational DDS domain."""
    found = []
    own = os.getpid()
    for entry in Path('/proc').glob('[0-9]*/environ'):
        pid = int(entry.parent.name)
        if pid == own:
            continue
        try:
            environment = entry.read_bytes().split(b'\0')
            if ('ROS_DOMAIN_ID=' + domain).encode() not in environment:
                continue
            command = (entry.parent / 'cmdline').read_bytes().replace(b'\0', b' ').decode(errors='replace')
        except (OSError, ProcessLookupError):
            continue
        # The invoking shell and pytest runner cannot publish ROS data before
        # rclpy.init; any other process with this private domain is a conflict.
        if pid == os.getppid():
            continue
        found.append({'pid': pid, 'command': command})
    return found


@pytest.mark.parametrize('mode', ['static_only_baseline', 'field_dynamic_obstacles'])
def test_native_global_obstacle_invalidates_path_detours_and_clears(tmp_path, mode):
    import rclpy
    from action_msgs.msg import GoalStatus
    from geometry_msgs.msg import PoseStamped, TransformStamped
    from lifecycle_msgs.srv import ChangeState
    from nav_msgs.msg import OccupancyGrid, Odometry
    from nav2_msgs.action import ComputePathToPose
    from nav2_msgs.srv import IsPathValid
    from rcl_interfaces.msg import Parameter as RosParameter, ParameterType, ParameterValue
    from rcl_interfaces.srv import GetParameters, SetParameters
    from rclpy.action import ActionClient
    from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
    from sensor_msgs.msg import PointCloud2, PointField
    from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
    from agt_field_test_bringup import profiles
    from workbench.runtime import REPO, ROOT

    evidence = Path(os.environ.get('AGT_OBSTACLE_REPLAN_EVIDENCE', str(tmp_path))) / mode
    evidence.mkdir(parents=True, exist_ok=True)
    report = {'status': 'RUNNING', 'ros_domain_id': '186', 'ros_localhost_only': '1',
              'started_unix': time.time(), 'checks': {}, 'controller_started': False,
              'hardware_started': False, 'mode': mode}
    conflicts = _domain_processes('186')
    report['preexisting_domain_processes'] = conflicts
    (evidence / 'isolation.json').write_text(json.dumps(report, indent=2) + '\n')
    assert not conflicts, 'private domain 186 already in use: ' + repr(conflicts)

    parameters = evidence / 'generated_navigation.yaml'
    profile_path = profiles.selected_path(ROOT, 'bunker')
    profile = profiles.load(profile_path, 'bunker')
    profiles.navigation(profile, REPO.parent / 'agt_navigation_v3/config', parameters)
    tree = yaml.safe_load(parameters.read_text())
    global_params = tree['global_costmap']['global_costmap']['ros__parameters']
    assert global_params['plugins'] == ['static_layer', 'obstacle_layer', 'inflation_layer']
    obstacle = global_params['obstacle_layer']
    assert obstacle['plugin'] == 'agt_field_test_bringup/UnknownPreservingObstacleLayer'
    mark_topic = obstacle['lidar3d_mark']['topic']
    clear_topic = obstacle['lidar3d_clear']['topic']
    assert mark_topic != clear_topic
    assert obstacle['lidar3d_mark']['marking'] and not obstacle['lidar3d_mark']['clearing']
    assert obstacle['lidar3d_clear']['clearing'] and not obstacle['lidar3d_clear']['marking']
    report.update(profile_path=str(profile_path), profile_sha256=profile['profile_sha256'],
                  profiles_source=str(Path(profiles.__file__).resolve()),
                  parameters_sha256=hashlib.sha256(parameters.read_bytes()).hexdigest(),
                  generated_global_parameters=json.loads(json.dumps(global_params)))
    effective_parameters = evidence / 'effective_navigation.yaml'
    if mode == 'static_only_baseline':
        # The sole baseline difference reproduces the previous runtime:
        # Smac and path validity receive no live obstacle layer.
        global_params['plugins'].remove('obstacle_layer')
        del global_params['obstacle_layer']
    effective_parameters.write_text(yaml.safe_dump(tree, sort_keys=False))
    report['effective_parameters_sha256'] = hashlib.sha256(effective_parameters.read_bytes()).hexdigest()

    rclpy.init(args=[])
    node = rclpy.create_node('isolated_global_obstacle_replan')
    mark = node.create_publisher(PointCloud2, mark_topic, qos_profile_sensor_data)
    clear = node.create_publisher(PointCloud2, clear_topic, qos_profile_sensor_data)
    map_pub = node.create_publisher(OccupancyGrid, '/map', QoSProfile(
        depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    odom_pub = node.create_publisher(Odometry, '/agt/odometry/local', 10)
    tf = TransformBroadcaster(node)
    static_tf = StaticTransformBroadcaster(node)
    transforms = []
    for parent, child, z in [('map', 'odom', 0.), ('base_link', 'obstacle_test_sensor', .7)]:
        msg = TransformStamped()
        msg.header.frame_id, msg.child_frame_id = parent, child
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.transform.translation.z = z
        msg.transform.rotation.w = 1.
        transforms.append(msg)
    static_tf.sendTransform(transforms)

    grid_msg = OccupancyGrid()
    grid_msg.header.frame_id = 'map'
    grid_msg.info.resolution = .05
    grid_msg.info.width, grid_msg.info.height = 360, 280
    grid_msg.info.origin.position.x, grid_msg.info.origin.position.y = -4., -7.
    grid_msg.info.origin.orientation.w = 1.
    static_grid = np.zeros((280, 360), dtype=np.int8)
    # Static obstacle also lies in the later clearing fan: clearing must not
    # erase occupancy belonging to the separate static-map layer.
    static_grid[192:208, 132:148] = 100  # x=2.6..3.4, y=2.6..3.4
    # Unmapped space must remain blocked with allow_unknown=False even when
    # raw sensor rays traverse it. Free rays establish visibility, not terrain
    # traversability or permission to leave the published static map.
    static_grid[180:196, 220:236] = -1  # x=7.0..7.8, y=2.0..2.8
    grid_msg.data = static_grid.ravel().tolist()
    state = {'car': False, 'input': True, 'background': False}
    latest = {}
    car = np.array([[x, y, .2] for x in np.arange(4.2, 5.41, .04)
                    for y in np.arange(-1.8, 1.81, .04)], dtype='<f4')
    # Dense rays cross the whole former car footprint and the static obstacle.
    background = np.array([[10., y, -.6] for y in np.arange(-10., 10.01, .025)], dtype='<f4')

    def cloud(points):
        array = np.asarray(points, dtype='<f4').reshape(-1, 3)
        msg = PointCloud2(height=1, width=len(array), point_step=12,
                          row_step=12 * len(array), is_dense=True,
                          fields=[PointField(name=k, offset=4*i, datatype=PointField.FLOAT32, count=1)
                                  for i, k in enumerate(('x', 'y', 'z'))], data=array.tobytes())
        msg.header.frame_id = 'obstacle_test_sensor'
        msg.header.stamp = node.get_clock().now().to_msg()
        return msg

    def tick():
        stamp = node.get_clock().now().to_msg()
        transform = TransformStamped()
        transform.header.frame_id, transform.child_frame_id = 'odom', 'base_link'
        transform.header.stamp = stamp
        transform.transform.rotation.w = 1.
        tf.sendTransform(transform)
        odom = Odometry()
        odom.header.frame_id, odom.child_frame_id = 'odom', 'base_link'
        odom.header.stamp = stamp
        odom.pose.pose.orientation.w = 1.
        odom_pub.publish(odom)
        grid_msg.header.stamp = stamp
        map_pub.publish(grid_msg)
        if state['input']:
            mark.publish(cloud(car if state['car'] else []))
            clear.publish(cloud(background if state['background'] else (car if state['car'] else [])))

    node.create_timer(.05, tick)

    def receive(msg):
        latest['grid'] = msg
        latest['received_monotonic'] = time.monotonic()

    node.create_subscription(OccupancyGrid, '/global_costmap/costmap', receive, qos_profile_sensor_data)
    log_path = evidence / 'planner.log'
    log = log_path.open('w')
    process = subprocess.Popen(['/opt/ros/humble/lib/nav2_planner/planner_server',
                                '--ros-args', '--params-file', str(effective_parameters)],
                               stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    report['planner_pid'] = process.pid
    action = None

    def spin(seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            assert process.poll() is None, log_path.read_text()
            rclpy.spin_once(node, timeout_sec=.02)

    def wait(future, seconds=10.):
        deadline = time.monotonic() + seconds
        while not future.done() and time.monotonic() < deadline:
            spin(.02)
        assert future.done(), 'native planner timed out; see ' + str(log_path)
        return future.result()

    def cost(x, y):
        msg = latest['grid']
        i = int(math.floor((x-msg.info.origin.position.x)/msg.info.resolution))
        j = int(math.floor((y-msg.info.origin.position.y)/msg.info.resolution))
        return int(msg.data[j*msg.info.width+i])

    def snapshot(name):
        msg = latest['grid']
        np.save(evidence / (name + '_costmap.npy'), np.asarray(msg.data, dtype=np.int8).reshape(
            msg.info.height, msg.info.width))
        (evidence / (name + '_costmap_metadata.json')).write_text(json.dumps({
            'resolution': msg.info.resolution, 'origin_xy': [msg.info.origin.position.x, msg.info.origin.position.y],
            'width': msg.info.width, 'height': msg.info.height,
            'car_center_cost': cost(4.8, 0.), 'static_center_cost': cost(3., 3.),
            'static_unknown_center_cost': cost(7.4, 2.4)}, indent=2) + '\n')

    def wait_cost(expected, seconds=8.):
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline:
            if 'grid' in latest and cost(4.8, 0.) == expected:
                return
            spin(.1)
        assert 'grid' in latest and cost(4.8, 0.) == expected, repr(latest.keys())

    try:
        lifecycle = node.create_client(ChangeState, '/planner_server/change_state')
        deadline = time.monotonic()+8.
        while not lifecycle.service_is_ready() and time.monotonic() < deadline:
            spin(.05)
        assert lifecycle.service_is_ready(), log_path.read_text()
        for transition in (1, 3):
            request = ChangeState.Request()
            request.transition.id = transition
            assert wait(lifecycle.call_async(request)).success, log_path.read_text()
        action = ActionClient(node, ComputePathToPose, '/compute_path_to_pose')
        assert action.wait_for_server(timeout_sec=5.)
        validity = node.create_client(IsPathValid, '/is_path_valid')
        assert validity.wait_for_service(timeout_sec=5.)
        wait_cost(0)
        assert cost(3., 3.) == 100
        assert cost(7.4, 2.4) == -1

        def plan(name):
            request = ComputePathToPose.Goal()
            request.use_start, request.planner_id = True, 'GridBased'
            for field, x in [('start', 0.), ('goal', 9.)]:
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.header.stamp = node.get_clock().now().to_msg()
                pose.pose.position.x, pose.pose.orientation.w = x, 1.
                setattr(request, field, pose)
            begin = time.monotonic()
            handle = wait(action.send_goal_async(request))
            assert handle.accepted
            result = wait(handle.get_result_async())
            assert result.status == GoalStatus.STATUS_SUCCEEDED, log_path.read_text()
            path = result.result.path
            xy = np.array([[p.pose.position.x, p.pose.position.y] for p in path.poses])
            assert len(xy) > 2
            values = {'elapsed_sec': time.monotonic()-begin, 'pose_count': len(xy),
                      'max_lateral_deviation_m': float(np.max(np.abs(xy[:, 1]))),
                      'path_length_m': float(np.linalg.norm(np.diff(xy, axis=0), axis=1).sum())}
            report['checks'][name] = values
            (evidence / (name+'_path.json')).write_text(json.dumps(xy.tolist()) + '\n')
            return path, xy

        def valid(path):
            request = IsPathValid.Request()
            request.path = path
            result = wait(validity.call_async(request))
            return result.is_valid, list(result.invalid_pose_indices)

        initial_path, initial_xy = plan('initial')
        assert np.max(np.abs(initial_xy[:, 1])) < .10
        assert valid(initial_path)[0]
        snapshot('initial')
        if mode == 'field_dynamic_obstacles':
            set_parameters = node.create_client(SetParameters, '/global_costmap/global_costmap/set_parameters')
            get_parameters = node.create_client(GetParameters, '/global_costmap/global_costmap/get_parameters')
            assert set_parameters.wait_for_service(timeout_sec=5.)
            assert get_parameters.wait_for_service(timeout_sec=5.)
            change = SetParameters.Request()
            change.parameters = [RosParameter(
                name='obstacle_layer.combination_method',
                value=ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=0))]
            response = wait(set_parameters.call_async(change))
            assert len(response.results) == 1 and not response.results[0].successful, (
                'unsupported global obstacle combination method must be rejected')
            read = GetParameters.Request()
            read.names = ['obstacle_layer.combination_method']
            effective = wait(get_parameters.call_async(read))
            assert len(effective.values) == 1 and effective.values[0].integer_value == 1
            report['checks']['unsupported_combination_method_rejected'] = {
                'passed': True, 'requested_value': 0, 'effective_value': effective.values[0].integer_value,
                'rejection_reason': response.results[0].reason}
        state['car'] = True
        begin = time.monotonic()
        if mode == 'static_only_baseline':
            spin(3.0)
            assert cost(4.8, 0.) == 0
            assert valid(initial_path)[0]
            _, baseline_xy = plan('occupied_baseline')
            assert np.max(np.abs(baseline_xy[:, 1])) < .10
            report['checks']['old_failure_reproduced'] = {
                'passed': True, 'car_center_cost': cost(4.8, 0.),
                'old_path_still_valid': True, 'new_plan_still_crosses_car': True}
            snapshot('occupied_baseline')
            report['status'] = 'PASS'
            return
        wait_cost(100)
        is_valid, indices = valid(initial_path)
        # Some Humble releases return the boolean but leave the optional
        # invalid_pose_indices array empty; the boolean is the BT contract.
        assert not is_valid, 'native IsPathValid must reject the occupied original path'
        report['checks']['original_path_invalidated'] = {'passed': True,
            'observation_to_published_costmap_sec': time.monotonic()-begin,
            'returned_invalid_pose_indices': indices}
        snapshot('occupied')
        detour_path, detour_xy = plan('detour')
        assert np.max(np.abs(detour_xy[:, 1])) > 2.0
        assert valid(detour_path)[0], 'new Smac path must be valid in native updated costmap'
        # Sample entire segments, rather than only the returned path poses.
        samples = np.concatenate([a + np.linspace(0, 1, max(2, int(np.linalg.norm(b-a)/.01)+1))[:, None]*(b-a)
                                  for a, b in zip(detour_xy, detour_xy[1:])])
        assert not np.any((samples[:, 0] >= 4.2) & (samples[:, 0] <= 5.4) &
                          (samples[:, 1] >= -1.8) & (samples[:, 1] <= 1.8))

        state['input'] = False
        spin(3.0)
        assert cost(4.8, 0.) == 100
        assert not valid(initial_path)[0]
        report['checks']['input_dropout_preserves_obstacle'] = {'passed': True, 'dropout_sec': 3.0,
                                                                'car_center_cost': cost(4.8, 0.)}
        snapshot('dropout')
        state.update(car=False, input=True, background=True)
        begin = time.monotonic()
        wait_cost(0)
        # Give every dense background ray time to cover former occupied cells.
        spin(1.2)
        assert cost(3., 3.) == 100
        assert valid(initial_path)[0], 'background rays must restore validity of the original path'
        restored_path, restored_xy = plan('cleared')
        assert valid(restored_path)[0]
        assert np.max(np.abs(restored_xy[:, 1])) < .10
        snapshot('cleared')
        report['checks']['evidence_clearing_preserves_static'] = {'passed': True,
            'clear_and_publish_sec': time.monotonic()-begin, 'car_center_cost': cost(4.8, 0.),
            'static_center_cost': cost(3., 3.)}
        unknown_cost = cost(7.4, 2.4)
        report['checks']['static_unknown_remains_impassable'] = {
            'passed': unknown_cost == -1 or unknown_cost >= 99,
            'initial_cost': -1, 'after_clearing_cost': unknown_cost,
            'planner_allow_unknown': tree['planner_server']['ros__parameters']['GridBased']['allow_unknown']}
        assert unknown_cost == -1 or unknown_cost >= 99, (
            'raw clearing rays made static unknown traversable: occupancy=' + str(unknown_cost))
        report['status'] = 'PASS'
    except BaseException as error:
        report.update(status='FAIL', error=repr(error))
        raise
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=5.)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=3.)
        report['planner_exit_code'] = process.returncode
        report['finished_unix'] = time.time()
        report['limits'] = ['synthetic cloud and fixed TF; no physical obstacle perception or stopping-distance validation',
                            'direct planner and IsPathValid service; behavior-tree dispatch is outside this test',
                            'three-second dropout checks absence of a short TTL; not indefinite sensor-fault readiness']
        (evidence / 'result.json').write_text(json.dumps(report, indent=2) + '\n')
        if action is not None:
            action.destroy()
        node.destroy_node()
        rclpy.shutdown()
        log.close()
