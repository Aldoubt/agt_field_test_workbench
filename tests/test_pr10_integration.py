"""Exercise the combined UI/field configuration without starting a ROS graph."""
import ast
import json
import os
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace
import xml.etree.ElementTree as ET

import pytest
import yaml

from agt_field_test_bringup import profiles
from agt_field_test_bringup.photo_policy import photo_runtime_parameters
from workbench import field_photo

REPO = Path(__file__).resolve().parents[1]


def ament_share(monkeypatch, share):
    package = ModuleType('ament_index_python')
    module = ModuleType('ament_index_python.packages')
    module.get_package_share_directory = lambda name: str(share)
    package.packages = module
    monkeypatch.setitem(sys.modules, package.__name__, package)
    monkeypatch.setitem(sys.modules, module.__name__, module)


def test_cli_and_qt_photo_policy_share_bt_and_stop_parameters(monkeypatch, tmp_path):
    ament_share(monkeypatch, tmp_path)
    command = field_photo.command_plan(SimpleNamespace(command='runtime'))
    cli = {command[i + 1].split(':=', 1)[0]: yaml.safe_load(command[i + 1].split(':=', 1)[1])
           for i, token in enumerate(command) if token == '-p'}
    qt = photo_runtime_parameters(REPO/'config/photo_navigate.xml', field_photo.ROOT/'data/captures/missions')
    assert cli == qt
    assert qt['recenter_after_capture'] is True
    assert qt['stationary_timeout_sec'] == 15.0
    assert qt['wheel_stop_gate'] is True and qt['wait_for_bunker_control'] is True
    tree = ET.parse(qt['nav_behavior_tree'])
    follow = tree.find('.//FollowPath')
    assert follow.attrib['controller_id'] == 'PhotoFollowPath'
    assert follow.attrib['goal_checker_id'] == 'photo_goal_checker'
    assert tree.find('.//RecoveryNode').attrib['number_of_retries'] == '1'
    assert all(tree.find('.//' + tag) is None for tag in ('Spin', 'BackUp', 'ClearEntireCostmap'))


def test_photo_policy_refuses_missing_tree_and_does_not_wait_for_bunker_on_fwmini(tmp_path):
    with pytest.raises(ValueError, match='missing'):
        photo_runtime_parameters(tmp_path/'missing.xml', tmp_path)
    config = photo_runtime_parameters(REPO/'config/photo_navigate.xml', tmp_path, 'fwmini')
    assert config['wait_for_bunker_control'] is False
    assert config['wheel_stop_gate'] is False


@pytest.mark.parametrize('robot', ['bunker', 'fwmini'])
def test_launch_passes_shared_photo_policy_to_actual_mission_node(tmp_path, robot):
    # Evaluate the production Node's parameters expression, without importing
    # launch_ros or creating a process. This catches a helper wired to no caller.
    tree = ast.parse((REPO/'agt_field_test_bringup/launch/field_test.launch.py').read_text())
    node = next(call for call in ast.walk(tree) if isinstance(call, ast.Call)
                and isinstance(call.func, ast.Name) and call.func.id == 'Node'
                and any(kw.arg == 'executable' and isinstance(kw.value, ast.Constant)
                        and kw.value.value == 'mission_runtime' for kw in call.keywords))
    params = next(kw.value for kw in node.keywords if kw.arg == 'parameters')
    actual = eval(compile(ast.Expression(params), 'launch_parameters', 'eval'), {
        'runtime_share': tmp_path/'runtime', 'workspace': tmp_path, 'robot': robot,
        'photo_runtime_parameters': photo_runtime_parameters,
        'value': lambda name: str(REPO/'config/photo_navigate.xml')
            if name == 'inspection_behavior_tree' else None})
    assert actual[1] == photo_runtime_parameters(
        REPO/'config/photo_navigate.xml', tmp_path/'data/captures/missions', robot)


@pytest.mark.parametrize('robot', ['bunker', 'fwmini'])
@pytest.mark.parametrize('mode', ['navigation', 'distance'])
def test_combined_profiles_preserve_field_geometry_and_controller_policy(monkeypatch, tmp_path, robot, mode):
    source = Path(os.environ.get('AGT_NAV_SOURCE', str(REPO.parent/'agt_navigation_v3')))
    config = source/'config'
    if not (config/'controller.yaml').is_file():
        pytest.skip('set AGT_NAV_SOURCE to the reviewed navigation source')
    share = tmp_path/'nav2_bt_navigator'
    bt = share/'behavior_trees'
    bt.mkdir(parents=True)
    for name in ('navigate_w_recovery_and_replanning_only_if_path_becomes_invalid.xml',
                 'navigate_through_poses_w_replanning_and_recovery.xml'):
        (bt/name).write_text('<root><BehaviorTree><Sequence><ComputePathToPose/>'
                            '<FollowPath controller_id="FollowPath"/></Sequence></BehaviorTree></root>')
    ament_share(monkeypatch, share)
    profile = profiles.load(REPO/f'templates/config/robots/{robot}.yaml', robot)
    output = tmp_path/'navigation.yaml'
    profiles.navigation(profile, config, output, mode)
    generated = yaml.safe_load(output.read_text())
    controller = generated['controller_server']['ros__parameters']
    local = generated['local_costmap']['local_costmap']['ros__parameters']
    global_map = generated['global_costmap']['global_costmap']['ros__parameters']
    assert controller['FollowPath']['plugin'] == 'agt_field_test_bringup/PlanarRegulatedPurePursuitController'
    assert controller['PhotoFollowPath']['use_rotate_to_heading'] is False
    assert controller['PhotoFollowPath']['allow_reversing'] is False
    planner = generated['planner_server']['ros__parameters']['GridBased']
    assert planner['allow_unknown'] is False
    if mode == 'distance':
        assert planner['plugin'] == 'agt_field_test_bringup/ExactGoalPlanner'
        assert planner['smoother']['w_data'] == 0.0
        assert controller['general_goal_checker']['xy_goal_tolerance'] == .02
        assert controller['FollowPath']['allow_reversing'] is True
        tree = ET.parse(generated['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml'])
        assert tree.find('.//RecoveryNode') is None
    if robot == 'bunker':
        assert json.loads(local['footprint']) == [[.6,.475],[.6,-.475],[-.6,-.475],[-.6,.475]]
        assert local['inflation_layer']['inflation_radius'] == 1.0
        assert global_map['inflation_layer']['inflation_radius'] == 1.2
        for name in ('FollowPath', 'PhotoFollowPath'):
            assert controller[name]['inflation_cost_scaling_factor'] == local['inflation_layer']['cost_scaling_factor'] == 2.0
        assert global_map['obstacle_layer']['plugin'] == 'agt_field_test_bringup/UnknownPreservingObstacleLayer'
        assert global_map['obstacle_layer'] is not local['obstacle_layer']
    else:
        assert global_map['plugins'] == ['static_layer', 'inflation_layer']
        assert local['plugins'] == ['voxel_layer', 'inflation_layer']


def test_default_install_does_not_apply_experimental_bbs():
    for name in ('init_workspace.sh', 'build_humble.sh'):
        executable_lines = '\n'.join(line for line in (REPO/'scripts'/name).read_text().splitlines()
                                     if not line.lstrip().startswith('#'))
        assert 'apply_bbs_recovery.py' not in executable_lines
