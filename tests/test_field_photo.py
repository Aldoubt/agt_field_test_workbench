"""Use a private ROS domain: enqueue real poses without sending any Action goal."""
from pathlib import Path
import time
import importlib.util
import math

import pytest
import yaml

from workbench import field_photo

REPO = Path(__file__).resolve().parents[1]


def test_rviz_collects_on_a_separate_topic_and_has_no_direct_navigation_tool():
    data = yaml.safe_load((REPO/'config/field_photo.rviz').read_text())
    tools = data['Visualization Manager']['Tools']
    assert not any(t['Class'] == 'nav2_rviz_plugins/GoalTool' for t in tools)
    goal = next(t for t in tools if t['Class'] == 'rviz_default_plugins/SetGoal')
    assert goal['Topic']['Value'] == '/agt/rviz_patrol/goal'
    displays = data['Visualization Manager']['Displays']
    assert next(d for d in displays if d['Name'] == 'Photo Waypoints')['Topic']['Value'] == '/agt/rviz_patrol/markers'


@pytest.mark.parametrize('command', ['camera', 'runtime', 'queue', 'rviz', 'start', 'clear', 'cancel'])
def test_dry_run_never_launches_or_sends_a_service(monkeypatch, command):
    def forbidden(*args):
        pytest.fail('dry-run attempted execution')
    monkeypatch.setattr(field_photo.os, 'execvp', forbidden)
    assert field_photo.main([command, '--dry-run']) == 0


def test_camera_device_resolves_relative_by_id_symlink(tmp_path, capsys):
    device = tmp_path/'video0'
    device.touch()
    ids = tmp_path/'v4l/by-id'
    ids.mkdir(parents=True)
    link = ids/'camera-index0'
    link.symlink_to('../../video0')
    field_photo.main(['camera', '--camera-device', str(link), '--dry-run'])
    assert 'device_path:='+str(device) in capsys.readouterr().out


def test_start_refuses_old_runtime_without_submitting_task(monkeypatch):
    def old():
        field_photo.validate_photo_parameters({'nav_behavior_tree': None}, {})
    monkeypatch.setattr(field_photo, 'check_photo_runtime', old)
    monkeypatch.setattr(field_photo.os, 'execvp', lambda *args: pytest.fail('submitted with old runtime'))
    assert field_photo.main(['start']) == 2


def test_start_accepts_photo_runtime_and_loaded_checker(monkeypatch):
    runtime = {'nav_behavior_tree': str(REPO/'config/photo_navigate.xml'), 'recenter_after_capture': True}
    controller = {'goal_checker_plugins': ['general_goal_checker', 'photo_goal_checker'],
                  'photo_goal_checker.xy_goal_tolerance': .35,
                  'photo_goal_checker.yaw_goal_tolerance': math.pi}
    monkeypatch.setattr(field_photo, 'check_photo_runtime',
                        lambda: field_photo.validate_photo_parameters(runtime, controller))
    sent = []
    monkeypatch.setattr(field_photo.os, 'execvp', lambda *args: sent.append(args))
    field_photo.main(['start'])
    assert len(sent) == 1 and '/agt/rviz_patrol/start' in sent[0][1]
    with pytest.raises(ValueError, match='photo_goal_checker'):
        field_photo.validate_photo_parameters(runtime, {})


def test_field_gimbal_error_replay_uses_real_arrival_judge(tmp_path):
    import subprocess
    preset = yaml.safe_load((REPO/'config/photo_three_views.yaml').read_text())
    center = preset['views'][1]
    source = tmp_path/'arrival.cpp'
    source.write_text('''#include <cassert>
#include "pantilt_camera_serial/arrival_judge.hpp"
int main(int argc, char** argv) {
  using namespace autolabor_driver;
  const Angles target{0., 0., 0.}, observed{2.47, .28, -.68};
  ArrivalJudge old(target, 1.5, 3);
  for (int i=0;i<80;i++) assert(!old.observe(observed));
  ArrivalJudge field(target, std::stod(argv[1]), std::stoi(argv[2]));
  assert(!field.observe(observed));
  assert(field.observe(observed));
  ArrivalJudge blocked(target, std::stod(argv[1]), std::stoi(argv[2]));
  for (int i=0;i<80;i++) assert(!blocked.observe(Angles{15.,0.,0.}));
}
'''.replace('#include <cassert>', '#include <cassert>\n#include <string>'))
    includes = REPO.parent/'Autolabor-C1-ROS2/src/pantilt_camera_serial/include'
    binary = tmp_path/'arrival'
    subprocess.run(['g++', '-std=c++17', '-I'+str(includes), str(source), '-o', str(binary)], check=True)
    subprocess.run([str(binary), str(center['tolerance']), str(center['stable_samples'])], check=True)
    assert preset['base_settle_time'] == .3 and center['settle_time'] == .2


def test_real_queue_transport_and_generated_mission(tmp_path, monkeypatch):
    import os
    # This test must never participate in a live robot domain.
    if os.environ.get('ROS_DOMAIN_ID') != '183':
        pytest.skip('run in isolated ROS_DOMAIN_ID=183')
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from agt_rviz_patrol.rviz_patrol import RvizPatrol
    from agt_navigation_runtime.mission_schema import load_mission
    from std_srvs.srv import Trigger

    rclpy.init(args=['--ros-args', '-p', 'goal_topic:=/agt/rviz_patrol/goal',
                     '-p', 'mission_dir:='+str(tmp_path),
                     '-p', 'preset_file:='+str(REPO/'config/photo_three_views.yaml'),
                     '-p', 'map_id:=test_map'])
    node = RvizPatrol()
    publisher = node.create_publisher(PoseStamped, '/agt/rviz_patrol/goal', 10)
    def forbidden(*args, **kwargs):
        pytest.fail('collecting or serializing waypoints sent an Action goal')
    monkeypatch.setattr(node.client, 'send_goal_async', forbidden)
    monkeypatch.setattr(node, 'current_home', lambda:dict(x=0., y=0., yaw=0., frame_id='map'))
    try:
        deadline = time.monotonic()+3
        while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
        assert publisher.get_subscription_count() == 1
        for index in range(3):
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = float(index+1)
            pose.pose.position.y = 2.
            pose.pose.orientation.w = 1.
            publisher.publish(pose)
        deadline = time.monotonic()+3
        while len(node.queue) < 3 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
        assert len(node.queue) == 3
        mission = load_mission(str(node.build_mission_file()))
        assert [p.id for p in mission.points] == ['P001', 'P002', 'P003', 'RETURN_HOME']
        assert [p.x for p in mission.points] == [1., 2., 3., 0.]
        assert all([v.heading for v in p.views] == [-30., 0., 30.] for p in mission.points[:3])
        assert mission.points[-1].views == []
        response = node.on_clear(None, Trigger.Response())
        assert response.success and not node.queue
    finally:
        node.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize('nav_fails', [False, True])
def test_photo_runtime_three_points_and_failure_stop(tmp_path, nav_fails):
    import os
    if os.environ.get('ROS_DOMAIN_ID') != '183' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        pytest.skip('requires isolated ROS domain 183 on localhost')
    # Exercise the production runtime with real ROS Actions, fake Nav2/camera,
    # measured odometry and the existing locked repository's test fixture.
    path = REPO.parent/'agt_navigation_v3/navigation/nav2/agt_navigation_runtime/test/test_runtime_isolated_ros.py'
    spec = importlib.util.spec_from_file_location('photo_runtime_fixture', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    rig = module.Rig(tmp_path)
    try:
        tree = str(REPO/'config/photo_navigate.xml')
        assert rig.runtime.get_parameter('nav_behavior_tree').value == ''
        rig.parameter('nav_behavior_tree', tree)
        rig.parameter('nav_action_timeout_sec', 2.)
        rig.parameter('camera_action_timeout_sec', 2.)
        name, mission = rig.mission(views=3, points=3, return_home=True)
        document = yaml.safe_load(mission.read_text())
        for p in document['points'][:3]:
            for v, heading in zip(p['views'], [-30., 0., 30.]):
                v['heading'] = heading
        mission.write_text(yaml.safe_dump(document))
        if nav_fails:
            rig.peers.nav_fail_at = 1
        result = rig.result(rig.submit(mission), timeout=15.)
        _, manifest = rig.record(name)
        assert all(h.request.behavior_tree == tree for h in rig.peers.nav_goals)
        if nav_fails:
            assert not result.result.success
            assert len(rig.peers.nav_goals) == 1
            assert not rig.peers.camera_goals
            assert manifest['completed_points'] == manifest['capture_records'] == 0
            assert manifest['base_stationary'] and manifest['child_actions_terminal']
        else:
            assert result.result.success and result.result.completed_points == 4
            assert len(rig.peers.nav_goals) == 4
            assert [h.request.heading for h in rig.peers.camera_goals] == [-30., 0., 30.]*3
            assert manifest['capture_records'] == 9
    finally:
        rig.close()


def test_photo_navigation_keeps_default_checker_and_uses_separate_arrival(tmp_path):
    from agt_field_test_bringup import profiles
    import xml.etree.ElementTree as ET
    profile = profiles.load(field_photo.ROOT/'config/robots/bunker.yaml', 'bunker')
    source = REPO.parent/'agt_navigation_v3/config'
    output = tmp_path/'navigation.yaml'
    profiles.navigation(profile, source, output)
    controller = yaml.safe_load(output.read_text())['controller_server']['ros__parameters']
    original = yaml.safe_load((source/'controller.yaml').read_text())['controller_server']['ros__parameters']
    assert controller['goal_checker_plugins'][0] == original['goal_checker_plugins'][0]
    assert controller['general_goal_checker'] == original['general_goal_checker']
    assert controller['photo_goal_checker']['xy_goal_tolerance'] == .35
    assert controller['photo_goal_checker']['yaw_goal_tolerance'] == math.pi
    for key in ('default_nav_to_pose_bt_xml', 'default_nav_through_poses_bt_xml'):
        default = yaml.safe_load(output.read_text())['bt_navigator']['ros__parameters'][key]
        assert all(f.attrib['goal_checker_id'] == 'general_goal_checker'
                   for f in ET.parse(default).iter('FollowPath'))
    tree = ET.parse(REPO/'config/photo_navigate.xml')
    assert tree.find('.//FollowPath').attrib['goal_checker_id'] == 'photo_goal_checker'
    assert all(tree.find('.//'+tag) is None for tag in ('Spin', 'BackUp', 'ClearEntireCostmap'))


def test_concurrent_steady_waits_do_not_destroy_executor_timers(tmp_path):
    import os
    if os.environ.get('ROS_DOMAIN_ID') != '183' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        pytest.skip('requires isolated ROS domain 183 on localhost')
    path = REPO.parent/'agt_navigation_v3/navigation/nav2/agt_navigation_runtime/test/test_runtime_isolated_ros.py'
    spec = importlib.util.spec_from_file_location('photo_wait_fixture', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    rig = module.Rig(tmp_path)
    async def repeat():
        for _ in range(100):
            await rig.runtime._waiter.sleep(.001)
    try:
        tasks = [rig.executor.create_task(repeat) for _ in range(20)]
        module.wait(lambda: all(t.done() for t in tasks), timeout=10., label='2000 steady waits')
        assert all(t.exception() is None for t in tasks)
        assert rig.spin_error is None
    finally:
        rig.close()


@pytest.mark.parametrize('mode', ['success', 'failed', 'bad_angles', 'timeout', 'cancel'])
def test_recenter_feedback_before_next_navigation(tmp_path, mode):
    import os
    import json
    from rclpy.action import ActionClient, ActionServer, CancelResponse
    from pantilt_camera_serial.action import MovePantilt
    if os.environ.get('ROS_DOMAIN_ID') != '183' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        pytest.skip('requires isolated ROS domain 183 on localhost')
    path = REPO.parent/'agt_navigation_v3/navigation/nav2/agt_navigation_runtime/test/test_runtime_isolated_ros.py'
    spec = importlib.util.spec_from_file_location('photo_recenter_fixture', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    rig = module.Rig(tmp_path)
    moves = []
    def execute(handle):
        moves.append(handle)
        assert len(rig.peers.camera_goals) == 3 * len(moves)
        count = len(rig.peers.nav_goals)
        # A pending recenter must block navigation to the next waypoint.
        deadline = time.monotonic() + (.08 if mode not in ('timeout', 'cancel') else 3.)
        while time.monotonic() < deadline:
            assert len(rig.peers.nav_goals) == count
            if handle.is_cancel_requested:
                handle.canceled()
                return MovePantilt.Result()
            time.sleep(.005)
        result = MovePantilt.Result()
        result.success = mode != 'failed'
        result.result_code = 0 if result.success else 4
        result.final_heading = 30. if mode == 'bad_angles' else 1.5
        if result.success:
            handle.succeed()
        else:
            handle.abort()
        return result
    server = ActionServer(rig.peers, MovePantilt, rig.prefix+'/gimbal', execute,
                          cancel_callback=lambda _: CancelResponse.ACCEPT,
                          callback_group=rig.peers.group)
    try:
        rig.runtime.gimbal_client.destroy()
        rig.runtime.gimbal_client = ActionClient(rig.runtime, MovePantilt, rig.prefix+'/gimbal',
                                                callback_group=rig.runtime._control_group)
        module.wait(rig.runtime.gimbal_client.server_is_ready)
        rig.parameter('recenter_after_capture', True)
        rig.parameter('gimbal_action_timeout_sec', .25)
        rig.parameter('gimbal_server_timeout_sec', .3)
        name, mission = rig.mission(views=3, points=2, return_home=True)
        handle = rig.submit(mission)
        if mode == 'cancel':
            module.wait(lambda: len(moves) == 1)
            rig.cancel(handle)
        result = rig.result(handle, timeout=10.)
        root, manifest = rig.record(name)
        assert all((m.request.heading, m.request.roll, m.request.pitch) == (0., 0., 0.) for m in moves)
        assert all(m.request.stable_samples == 2 for m in moves)
        events = [json.loads(line) for line in (root/'events.jsonl').read_text().splitlines()]
        finished = [e for e in events if e['event'] == 'gimbal_recenter_finished']
        if mode == 'success':
            assert result.result.success
            assert len(moves) == 2 and len(rig.peers.nav_goals) == 3
            assert manifest['capture_records'] == 6  # return-home creates no photo/recenter
            assert all(e['success'] for e in finished)
        else:
            assert not result.result.success
            assert len(moves) == len(rig.peers.nav_goals) == 1
            assert manifest['completed_points'] == 0
            assert manifest['capture_records'] == 3
            assert manifest['child_actions_terminal'] and manifest['base_stationary']
            assert not any(e['success'] for e in finished)
        assert rig.spin_error is None
    finally:
        rig.close()
