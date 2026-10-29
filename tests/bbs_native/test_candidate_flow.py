"""Execute the production matcher with deterministic BBS/GICP return values.

Real Eigen, PCL file I/O, voxelization, descriptor retrieval and the production
candidate loop execute. Only expensive registration APIs are injected. These
prove orchestration/rejection behavior, not geometric accuracy on a robot.
"""
import json
import os
from pathlib import Path
import shutil
import subprocess

import pytest

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
NATIVE_REL = Path('navigation/localization/agt_global_relocalization_native')


def native_package():
    explicit = os.environ.get('BBS_NATIVE_SOURCE')
    root = Path(explicit) if explicit else REPO.parent/'agt_navigation_v3'
    candidates = [root, root/NATIVE_REL, root.parent.parent]
    for candidate in candidates:
        if (candidate/'src/candidate_bbs_gicp_localizer.cpp').is_file():
            return candidate
    message = 'patched agt_navigation_v3 unavailable; set BBS_NATIVE_SOURCE'
    if explicit:
        pytest.fail(message)
    pytest.skip(message)


@pytest.fixture(scope='session')
def matcher(tmp_path_factory):
    native = native_package()
    include_dirs = sorted(Path('/usr/include').glob('pcl-*'))
    compiler = shutil.which('g++')
    if not compiler or not include_dirs or not Path('/usr/include/eigen3/Eigen/Core').is_file():
        pytest.skip('C++ compiler, PCL headers and Eigen are required for native flow test')
    build = tmp_path_factory.mktemp('bbs-native-flow')
    binary = build/'candidate_bbs_gicp_localizer'
    command = [compiler, '-std=c++17', '-O0', '-g0', '-pthread',
               '-I'+str(HERE/'stubs'), '-I'+str(native/'include'),
               '-I/usr/include/eigen3', '-I'+str(include_dirs[-1]),
               str(native/'src/candidate_bbs_gicp_localizer.cpp'),
               '-o', str(binary), '-lpcl_io', '-lpcl_filters', '-lpcl_common']
    compiled = subprocess.run(command, capture_output=True, text=True, timeout=120)
    assert compiled.returncode == 0, compiled.stderr
    return binary


def write_inputs(directory, patch_names=('first.pcd', 'second.pcd')):
    # 400 separated points survive the real 0.35 m PCL voxel filter.
    points = [(x*.5, y*.5, 0.) for x in range(20) for y in range(20)]
    cloud = directory/'query.pcd'
    cloud.write_text(
        '# .PCD v0.7\nVERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\n'
        f'COUNT 1 1 1\nWIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n'
        f'POINTS {len(points)}\nDATA ascii\n' +
        ''.join(f'{x} {y} {z}\n' for x, y, z in points))
    assets = directory/'assets'
    assets.mkdir()
    # Same perfect sector similarity, deterministic ring-distance tie-break:
    # first ring key matches the query; second key is farther away.
    (assets/'polar_context.db').write_text(
        '# AGT_POLAR_CONTEXT_V1 1 1 35 .5 3 30\n'
        f'{patch_names[0]} 0 0 0 1 0 0 0 3 3\n'
        f'{patch_names[1]} 10 0 0 1 0 0 0 30 3\n')
    return cloud, assets


def run_case(matcher, directory, scenario, *extra, patch_names=('first.pcd', 'second.pcd')):
    cloud, assets = write_inputs(directory, patch_names)
    calls = directory/'calls.txt'
    env = dict(os.environ, AGT_TEST_SCENARIO=scenario, AGT_TEST_CALLS=str(calls))
    command = [str(matcher), '--map', str(cloud), '--scan', str(cloud),
               '--assets-dir', str(assets), '--candidate-top-k', '2', *extra]
    proc = subprocess.run(command, capture_output=True, text=True, env=env, timeout=10)
    result = json.loads(proc.stdout.strip().splitlines()[-1])
    trace = result.get('candidate_attempts', [])
    # Progress records are valid JSON independently, including failed stages.
    progress = [json.loads(line) for line in proc.stderr.splitlines() if line.startswith('{')]
    assert progress == trace
    return proc, result, calls.read_text().splitlines() if calls.exists() else []


@pytest.mark.parametrize(('scenario', 'rejection'), [
    ('first_nonconverged', 'GICP_NON_CONVERGENCE'),
    ('first_zero_inliers', 'GICP_NON_CONVERGENCE'),
    ('first_bad_fitness', 'FITNESS_HIGH'),
    ('first_low_overlap', 'OVERLAP_LOW'),
    ('first_nan_pose', 'GICP_NON_FINITE'),
])
def test_failed_high_coarse_candidate_reaches_second(matcher, tmp_path, scenario, rejection):
    proc, result, calls = run_case(matcher, tmp_path, scenario)
    assert proc.returncode == 0 and result['success'] is True
    assert result['candidate_patch'] == 'second.pcd'
    assert result['x'] == 10 and result['bbs_score'] == .8
    assert calls == ['bbs 0', 'gicp 0', 'bbs 10', 'gicp 10']
    assert [(t['rank'], t['state']) for t in result['candidate_attempts']] == [
        (1, 'GICP_REFINING'), (1, rejection), (2, 'GICP_REFINING'), (2, 'ACCEPTED')]
    assert result['candidate_attempts'][0]['bbs_score'] == .9


def test_all_fine_candidates_rejected_with_trace(matcher, tmp_path):
    proc, result, calls = run_case(matcher, tmp_path, 'all_nonconverged')
    assert proc.returncode == 4 and result['success'] is False
    assert result['reason_code'] == 'NO_ACCEPTABLE_REFINED_CANDIDATE'
    assert calls == ['bbs 0', 'gicp 0', 'bbs 10', 'gicp 10']
    assert [t['state'] for t in result['candidate_attempts']].count('GICP_NON_CONVERGENCE') == 2


@pytest.mark.parametrize(('scenario', 'reason'), [
    ('no_bbs_pose', 'BBS_NO_POSE'), ('bbs_timeout', 'BBS_TIMEOUT')])
def test_no_coarse_candidate_never_runs_refinement(matcher, tmp_path, scenario, reason):
    proc, result, calls = run_case(matcher, tmp_path, scenario)
    assert proc.returncode == 3 and result['success'] is False
    assert result['reason_code'] == 'NO_COARSE_CANDIDATE'
    assert calls == ['bbs 0', 'bbs 10']
    assert [t['state'] for t in result['candidate_attempts']] == [reason, reason]


def test_first_fully_accepted_candidate_finishes(matcher, tmp_path):
    proc, result, calls = run_case(matcher, tmp_path, 'first_success')
    assert proc.returncode == 0 and result['candidate_patch'] == 'first.pcd'
    assert calls == ['bbs 0', 'gicp 0']
    assert [t['state'] for t in result['candidate_attempts']] == ['GICP_REFINING', 'ACCEPTED']


def test_configured_quality_gate_is_applied_per_candidate(matcher, tmp_path):
    proc, result, calls = run_case(matcher, tmp_path, 'first_success', '--max-fitness', '.05')
    assert proc.returncode == 4 and result['success'] is False
    assert [t['state'] for t in result['candidate_attempts']].count('FITNESS_HIGH') == 2
    assert calls == ['bbs 0', 'gicp 0', 'bbs 10', 'gicp 10']


def test_late_convergence_is_not_accepted_after_total_budget(matcher, tmp_path):
    proc, result, calls = run_case(matcher, tmp_path, 'refine_over_budget', '--timeout', '1.0')
    assert proc.returncode == 4 and result['success'] is False
    assert result['reason_code'] == 'BUDGET_EXHAUSTED'
    assert result['candidate_attempts'][-1]['state'] == 'BUDGET_EXHAUSTED'
    assert calls == ['bbs 0', 'gicp 0']


@pytest.mark.parametrize('arguments', [
    ('--min-score', 'nan'), ('--min-overlap', '1.01'), ('--max-fitness', '-1')])
def test_invalid_quality_configuration_is_rejected_before_matching(matcher, tmp_path, arguments):
    proc, result, calls = run_case(matcher, tmp_path, 'first_success', *arguments)
    assert proc.returncode != 0 and result['success'] is False
    assert not calls


def test_patch_name_is_json_escaped_in_result_and_trace(matcher, tmp_path):
    name = 'first"quoted.pcd'
    proc, result, calls = run_case(matcher, tmp_path, 'first_success',
                                   patch_names=(name, 'second.pcd'))
    assert proc.returncode == 0 and result['candidate_patch'] == name
    assert all(row['patch'] == name for row in result['candidate_attempts'])
    assert calls == ['bbs 0', 'gicp 0']
