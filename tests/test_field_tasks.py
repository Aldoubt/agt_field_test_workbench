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


def test_accuracy_cli_produces_fifteen_unscored_physical_measurements(tmp_path):
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


def test_custom_distances_use_fixed_anchor_and_no_photos(tmp_path):
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
