"""Field mode removes paperwork only; exercise the actual launch without spawning it."""
import copy
import importlib.util
import json
from pathlib import Path

import pytest
import yaml
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, LogInfo
from agt_field_test_bringup import calibration
from agt_field_test_bringup.navigation_policy import require_navigation_calibration
from workbench import field_trial

REPO = Path(__file__).resolve().parents[1]
ROOT = REPO.parent.parent


def test_field_policy_uses_candidate_without_fabricating_verification():
    selected = dict(robot='bunker', status='CANDIDATE')
    original = copy.deepcopy(selected)
    assert require_navigation_calibration('field_test', True, 'bunker', selected) is False
    assert selected == original
    for mode, supervised, robot in [('formal', True, 'bunker'), ('field_test', False, 'bunker'),
                                    ('field_test', True, 'fwmini'), ('typo', True, 'bunker')]:
        with pytest.raises(ValueError):
            require_navigation_calibration(mode, supervised, robot, selected)
    selected['status'] = 'VERIFIED'
    assert require_navigation_calibration('formal', False, 'bunker', selected) is True


def test_freeze_preserves_calibration_and_rejects_mid_startup_changes(tmp_path):
    cal_path = ROOT/'calibration/bunker/bunker_calibration_candidate.yaml'
    plan = dict(inputs={'calibration_bundle': str(cal_path)},
                calibration_file_sha256=field_trial.hash_file(cal_path),
                launch_args=['calibration_bundle:='+str(cal_path)])
    directory, frozen = field_trial.snapshot_plan(tmp_path, plan)
    saved = directory/'calibration_bundle.yaml'
    assert saved.read_bytes() == cal_path.read_bytes()
    assert frozen['launch_args'] == ['calibration_bundle:='+str(saved)]
    assert plan['launch_args'] == ['calibration_bundle:='+str(cal_path)]
    assert json.loads((directory/'session.json').read_text())['inputs'] == plan['inputs']
    plan['calibration_file_sha256'] = 'changed'
    with pytest.raises(ValueError, match='changed during startup'):
        field_trial.snapshot_plan(tmp_path, plan)


@pytest.mark.parametrize('backend,planning_profile', [('sc_kiss','navigation'),('bbs','navigation'),('bbs','distance')])
def test_actual_field_launch_keeps_runtime_guards_and_selected_frontend(monkeypatch, tmp_path, backend,planning_profile):
    # Uses current map/SC files read-only, and constructs launch actions without executing them.
    plan = field_trial.make_plan(ROOT, ROOT/'config/field_trial.yaml', global_backend=backend,planning_profile=planning_profile)
    assert plan['calibration_status'] == 'CANDIDATE'
    before = Path(plan['inputs']['calibration_bundle']).read_bytes()
    module_path = REPO/'agt_field_test_bringup/launch/field_test.launch.py'
    spec = importlib.util.spec_from_file_location('field_trial_launch', module_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    for action in module.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    context.launch_configurations.update(item.split(':=', 1) for item in plan['launch_args'])
    includes = []
    def include(package, filename, args=None):
        includes.append((package, filename, args))
        return LogInfo(msg='offline include '+filename)
    monkeypatch.setattr(module, '_include', include)
    import workbench.safety
    def no_report(*args, **kwargs):
        pytest.fail('supervised field trial must not require a formal report')
    monkeypatch.setattr(workbench.safety, 'report', no_report)
    nodes = module._configure(context)
    assert nodes
    loc = next(args for package, filename, args in includes if filename == 'localization.launch.py')
    assert loc['lio_backend'] == 'fastlio2'
    assert loc['global_backend'] == backend
    if backend == 'bbs':
        assert Path(loc['relocalization_assets'], 'voxelmaps_coords').is_dir()
    assert loc['auto_relocalize'] == 'false'
    assert (loc['lidar_topic'], loc['imu_topic']) == ('/livox/lidar', '/livox/imu')
    generated = yaml.safe_load(Path(loc['fastlio_config']).read_text())
    assert (generated['world_frame'], generated['body_frame']) == ('odom', 'body')
    nav = next(args for package, filename, args in includes if filename == 'navigation.launch.py')
    assert nav['autostart'] == 'false'
    tree = yaml.safe_load(Path(nav['runtime_params_file']).read_text())
    planner=tree['planner_server']['ros__parameters']['GridBased']
    assert planner['allow_unknown'] is False
    assert planner['cost_travel_multiplier']==3.
    if planning_profile=='distance':
        assert planner['smoother']==dict(w_data=0.,max_iterations=100000)
        inflation=tree['global_costmap']['global_costmap']['ros__parameters']['inflation_layer']
        assert inflation['inflate_around_unknown'] and inflation['inflate_unknown']
    else:
        assert 'smoother' not in planner
        assert 'inflate_around_unknown' not in tree['global_costmap']['global_costmap']['ros__parameters']['inflation_layer']
    guard = tree['agt_cmd_vel_guard']['ros__parameters']
    assert guard['require_localization_status'] is True
    assert guard['require_chassis_feedback'] is True
    assert guard['output_topic'] == '/mux/cmd_vel'
    assert guard['command_timeout_sec'] == .2
    assert guard['max_linear_x'] == plan['limits']['forward_mps']
    assert tree['controller_server']['ros__parameters']['FollowPath']['plugin'] == 'agt_field_test_bringup/PlanarRegulatedPurePursuitController'
    assert tree['controller_server']['ros__parameters']['FollowPath']['desired_linear_vel'] == plan['limits']['forward_mps']
    assert tree['velocity_smoother']['ros__parameters']['max_velocity'] == [plan['limits']['forward_mps'], 0., plan['limits']['angular_radps']]
    assert tree['local_costmap']['local_costmap']['ros__parameters']['update_frequency'] == 20.
    assert tree['local_costmap']['local_costmap']['ros__parameters']['publish_frequency'] == 10.
    local=tree['local_costmap']['local_costmap']['ros__parameters']
    assert local['plugins']==['obstacle_layer','inflation_layer']
    assert local['obstacle_layer']['plugin']=='nav2_costmap_2d::ObstacleLayer'
    assert local['obstacle_layer']['min_obstacle_height']==-3.
    assert local['inflation_layer']['inflation_radius']==.65
    assert tree['global_costmap']['global_costmap']['ros__parameters']['inflation_layer']['inflation_radius']==.8
    assert local['inflation_layer']['cost_scaling_factor']==tree['controller_server']['ros__parameters']['FollowPath']['inflation_cost_scaling_factor']==3.
    assert tree['controller_server']['ros__parameters']['FollowPath']['cost_scaling_dist']<=local['inflation_layer']['inflation_radius']
    assert tree['agt_pointcloud_preprocessor']['ros__parameters']['self_filter']['extra_boxes'] == [
        -.36,.02,-.17,.19,.72,1.68, -.95,-.60,-.12,.31,.82,1.68]
    assert Path(plan['inputs']['calibration_bundle']).read_bytes() == before
    # Formal launch remains separate and refuses this unpromoted candidate.
    context.launch_configurations['navigation_mode'] = 'formal'
    with pytest.raises(ValueError, match='VERIFIED'):
        module._configure(context)


@pytest.mark.parametrize('command', ['start', 'build-polar-context', 'map', 'nav', 'rviz', 'relocalize', 'cancel', 'record', 'status'])
def test_dry_run_never_executes_or_observes_hardware(command, monkeypatch, capsys):
    def forbidden(*args, **kwargs):
        pytest.fail('dry-run must not execute actions')
    monkeypatch.setattr(field_trial.os, 'execvp', forbidden)
    monkeypatch.setattr(field_trial, 'snapshot_plan', forbidden)
    monkeypatch.setattr(field_trial, 'observe', forbidden)
    monkeypatch.setattr(field_trial, 'activate_navigation', forbidden)
    monkeypatch.setattr(field_trial, 'request_relocalization', forbidden)
    assert field_trial.main([command, '--dry-run', '--config', str(ROOT/'config/field_trial.yaml')]) == 0
    assert capsys.readouterr().out


def test_one_shot_matcher_result_comes_from_live_process_only(tmp_path):
    proc = tmp_path/'proc';logs = tmp_path/'logs'
    (proc/'123').mkdir(parents=True);logs.mkdir()
    executable = str(tmp_path/'install/agt_global_relocalization/lib/agt_global_relocalization/global_relocalization')
    (proc/'123/cmdline').write_bytes(('python3\0'+executable+'\0').encode())
    event = dict(state='REJECTED', detail='SCAN_CONTEXT_NO_CANDIDATE')
    (logs/'python3_123_1.log').write_text('[INFO] [1.0] [agt_global_relocalization]: '+json.dumps(event)+'\n')
    result = field_trial.matcher_log_status(tmp_path, proc, logs)
    assert result['status'] == event and result['cached'] is True
    (proc/'123/cmdline').write_bytes(b'other_process\0')
    assert field_trial.matcher_log_status(tmp_path, proc, logs) is None


def test_bbs_selection_uses_existing_assets_without_requiring_sc(monkeypatch):
    def no_sc(*args, **kwargs):
        pytest.fail('BBS must not require Scan Context retrieval or descriptors')
    monkeypatch.setattr(field_trial, 'validate_sc_database', no_sc)
    plan = field_trial.make_plan(ROOT, ROOT/'config/field_trial.yaml', global_backend='bbs')
    args = dict(v.split(':=', 1) for v in plan['launch_args'])
    assert args['global_backend'] == 'bbs'
    assert 'sc_database' not in args
    assert Path(args['relocalization_assets'], 'global_map_downsampled.pcd').is_file()
    assert '3D-BBS' in plan['relocalization_strategy']
    assert args['lio_backend'] == 'fastlio2' and args['navigation_autostart'] == 'false'
    monkeypatch.setattr(field_trial, 'relocalization_assets', lambda m: '')
    with pytest.raises(ValueError, match='BBS assets'):
        field_trial.make_plan(ROOT, ROOT/'config/field_trial.yaml', global_backend='bbs')


def test_bbs_cli_override_and_compute_budget_dry_run(monkeypatch, capsys):
    def forbidden(*args, **kwargs):
        pytest.fail('dry-run must not execute ROS services or start devices')
    monkeypatch.setattr(field_trial.os, 'execvp', forbidden)
    monkeypatch.setattr(field_trial, 'request_relocalization', forbidden)
    assert field_trial.main(['start', '--global-backend', 'bbs', '--dry-run']) == 0
    assert 'global_backend:=bbs' in capsys.readouterr().out
    assert field_trial.main(['relocalize', '--timeout-sec', '60', '--dry-run']) == 0
    assert 'sdk_timeout_sec=60.0' in capsys.readouterr().out
    assert field_trial.main(['start', '--global-backend', 'bbs', '--hold-system-clock', '--dry-run']) == 0
    assert 'hold_field_clock.sh' in capsys.readouterr().out
