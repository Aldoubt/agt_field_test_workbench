"""Fixed anchors must survive repeated runs; use the production mission schema."""
import copy
import importlib.util
import math
from pathlib import Path

import pytest
import yaml

REPO = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('field_tasks', REPO/'scripts/field_tasks.py')
tasks = importlib.util.module_from_spec(spec)
spec.loader.exec_module(tasks)


def book():
    return dict(map_id='test_map', points=dict(home=dict(
        pose=dict(x=12., y=-4., yaw=math.pi/2, frame_id='map'),
        view=dict(heading=0., roll=0., pitch=0.))))


def test_accuracy_uses_one_anchor_without_cumulative_distance_and_has_no_photos():
    data = book()
    before = copy.deepcopy(data)
    for distance in (5, 7, 10):
        mission = tasks.accuracy_mission(data, distance)
        assert len(mission['points']) == 2
        for index, point in enumerate(mission['points']):
            assert point['views'] == []
            expected_y = -4. if index % 2 == 0 else -4.+distance
            assert point['pose']['x'] == pytest.approx(12.)
            assert point['pose']['y'] == pytest.approx(expected_y)
    assert data == before


def test_photo_order_angles_and_runtime_schema(tmp_path):
    from agt_navigation_runtime.mission_schema import load_mission
    data = book()
    data['points']['P01'] = copy.deepcopy(data['points']['home'])
    data['points']['P01']['view']['heading'] = 30.
    data['points']['P02'] = copy.deepcopy(data['points']['home'])
    data['points']['P02']['pose']['x'] = 15.
    path = tmp_path/'mission.yaml'
    tasks.save_new(path, tasks.photo_mission(data, ['P02', 'P01']))
    mission = load_mission(str(path))
    assert [point.id for point in mission.points] == ['P02', 'P01']
    assert mission.points[1].views[0].heading == 30.
    assert all(point.views[0].required for point in mission.points)
    with pytest.raises(FileExistsError):
        tasks.save_new(path, {})
    with pytest.raises(ValueError):
        tasks.photo_mission(data, ['P01', 'P01'])


def test_accuracy_cli_produces_fifteen_unscored_physical_measurements(tmp_path, monkeypatch):
    mock_live_pose(tmp_path, monkeypatch)
    import csv
    from agt_navigation_runtime.mission_schema import load_mission
    source = tmp_path/'points.yaml'
    source.write_text(yaml.safe_dump(book()))
    output = tmp_path/'accuracy'
    tasks.main(['--points', str(source), 'accuracy', '--output', str(output)])
    for distance in (5, 7, 10):
        assert len(load_mission(str(output/f'accuracy_{distance}m.yaml')).points) == 2
    assert len(load_mission(str(output/'return_home.yaml')).points) == 1
    with (output/'physical_measurements.csv').open() as stream:
        rows = list(csv.DictReader(stream))
    assert len(rows) == 15
    assert all(row['nav_result'] == row['measured_x'] == row['measured_y'] == '' for row in rows)
    with pytest.raises(FileExistsError):
        tasks.main(['--points', str(source), 'accuracy', '--output', str(output)])


def test_custom_distances_use_fixed_anchor_and_no_photos(tmp_path, monkeypatch):
    mock_live_pose(tmp_path, monkeypatch)
    source=tmp_path/'points.yaml';source.write_text(yaml.safe_dump(book()))
    output=tmp_path/'measurements'
    tasks.main(['--points',str(source),'accuracy','--distances','5','8','12',
                '--trials','2','--output',str(output)])
    for distance in [5,8,12]:
        doc=yaml.safe_load((output/f'accuracy_{distance}m.yaml').read_text())
        assert doc['points'][1]['pose']['y']==-4.+distance
        assert all(not point['views'] for point in doc['points'])
    import csv
    assert len(list(csv.DictReader((output/'physical_measurements.csv').open())))==6


@pytest.mark.parametrize('distances',[['0'],['-1'],['5','5'],['nan']])
def test_bad_distance_sets_never_create_partial_output(tmp_path,distances):
    source=tmp_path/'points.yaml';source.write_text(yaml.safe_dump(book()))
    output=tmp_path/'measurements'
    with pytest.raises((ValueError,SystemExit)):
        tasks.main(['--points',str(source),'accuracy','--output',str(output),'--distances',*distances])
    assert not output.exists()


def test_only_explicit_run_and_home_reach_action_submission(monkeypatch,tmp_path):
    submitted=[]
    monkeypatch.setattr(tasks,'execute_distance',lambda path:submitted.append(path))
    tasks.main(['run','8','--directory',str(tmp_path)])
    tasks.main(['home','--directory',str(tmp_path)])
    assert submitted==[tmp_path/'accuracy_8m.yaml',tmp_path/'return_home.yaml']


def test_distance_cancel_targets_parent_mission_not_nav_child(monkeypatch):
    commands=[]
    monkeypatch.setattr(tasks.os,'execvp',lambda name,args:commands.append(args))
    tasks.main(['cancel'])
    assert commands==[['ros2','service','call','/agt/task/cancel','std_srvs/srv/Trigger','{}']]


def mock_live_pose(tmp_path, monkeypatch):
    monkeypatch.setattr(tasks, 'ROOT', tmp_path)
    (tmp_path/'config').mkdir()
    (tmp_path/'config/field_trial.yaml').write_text('map_id: test_map\n')
    monkeypatch.setattr(tasks, 'read_pose', lambda map_id: book()['points']['home']['pose'].copy())


def test_accuracy_captures_current_pose_once_and_ignores_old_home(tmp_path, monkeypatch):
    mock_live_pose(tmp_path, monkeypatch)
    calls = []
    live = dict(x=100., y=200., yaw=0., frame_id='map')
    def read(map_id):
        calls.append(map_id)
        return live.copy()
    monkeypatch.setattr(tasks, 'read_pose', read)
    old = tmp_path/'old.yaml'
    old.write_text(yaml.safe_dump(book()))
    before = old.read_bytes()
    output = tmp_path/'batch'
    tasks.main(['--points', str(old), 'accuracy', '--output', str(output)])
    assert calls == ['test_map']
    assert old.read_bytes() == before
    for distance in (5, 7, 10):
        mission = yaml.safe_load((output/f'accuracy_{distance}m.yaml').read_text())
        assert mission['points'][0]['pose'] == live
        assert mission['points'][1]['pose']['x'] == 100. + distance
    assert yaml.safe_load((output/'return_home.yaml').read_text())['points'][0]['pose'] == live
    assert yaml.safe_load((output/'start_pose.yaml').read_text())['points']['home']['pose'] == live
    with pytest.raises(FileExistsError):
        tasks.main(['accuracy', '--output', str(output)])
    assert calls == ['test_map']


def test_accuracy_invalid_localization_leaves_no_batch(tmp_path, monkeypatch):
    mock_live_pose(tmp_path, monkeypatch)
    def fail(map_id):
        raise ValueError('localization is not valid')
    monkeypatch.setattr(tasks, 'read_pose', fail)
    output = tmp_path/'batch'
    with pytest.raises(ValueError, match='localization is not valid'):
        tasks.main(['accuracy', '--output', str(output)])
    assert not output.exists()


def test_default_run_requires_selected_batch_never_falls_back_to_old_directory(tmp_path, monkeypatch):
    monkeypatch.setattr(tasks, 'ROOT', tmp_path)
    (tmp_path/'missions/distance').mkdir(parents=True)
    submitted = []
    monkeypatch.setattr(tasks, 'execute_distance', submitted.append)
    with pytest.raises(ValueError, match='不会自动使用旧'):
        tasks.main(['run', '5'])
    assert not submitted


def test_generation_selects_latest_batch_for_run_and_home(tmp_path, monkeypatch):
    mock_live_pose(tmp_path, monkeypatch)
    submitted = []
    monkeypatch.setattr(tasks, 'execute_distance', submitted.append)
    for _ in range(2):
        tasks.main(['accuracy', '--distances', '5'])
        selected = Path(tasks.json.loads((tmp_path/'work/active_distance_batch.json').read_text())['directory'])
        tasks.main(['run', '5'])
        tasks.main(['home'])
        assert submitted[-2:] == [selected/'accuracy_5m.yaml', selected/'return_home.yaml']
    assert submitted[0].parent != submitted[2].parent
    active = (tmp_path/'work/active_distance_batch.json').read_bytes()
    with pytest.raises(FileExistsError):
        tasks.main(['accuracy', '--output', str(selected)])
    assert (tmp_path/'work/active_distance_batch.json').read_bytes() == active
