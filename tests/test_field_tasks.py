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
