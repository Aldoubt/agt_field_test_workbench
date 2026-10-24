"""Regression tests for the pinned local-ground exporter bridge (no ROS nodes needed)."""
import json
from pathlib import Path

import pytest
import yaml

from workbench.artifacts import Clouds
from workbench.maps import hash_file, read_pgm


@pytest.fixture
def fixture_root(tmp_path):
    config = (tmp_path / 'src/agt-lio-pgo-mapping/exporters'
              '/agt_pcd2grid_exporter/config/projection.yaml')
    config.parent.mkdir(parents=True)
    config.write_text(yaml.safe_dump({
        'projection_mode': 'local_ground', 'resolution': .05,
        'ground': {'cell_size': .5, 'percentile': .10, 'free_tolerance': .10},
        'obstacle_height': {'min': .12, 'max': 1.8},
        'occupied_threshold': 2, 'free_space_radius_cells': 2,
        'postprocess': {'closing_radius_cells': 1, 'min_component_cells': 3},
        'temporal_filter': {'enabled': True}, 'empty_cell': 'unknown'
    }))
    source = tmp_path / 'raw.pcd'
    source.write_bytes(b'test-localization-pcd')
    parent = tmp_path / 'maps/clouds/20261024T010000Z-12345678'
    parent.mkdir(parents=True)
    (parent / 'manifest.json').write_text(json.dumps({
        'id': parent.name, 'pcd': str(source), 'sha256': hash_file(source),
        'processing': {'bounds_min': [-1., -1., -1.], 'bounds_max': [1., 1., 2.]},
        'name': 'test_map'
    }))
    return tmp_path, parent.name


def test_local_ground_reuses_pinned_exporter_and_preserves_unknown(fixture_root):
    root, cloud_id = fixture_root
    calls = []
    def fake_runner(command, timeout):
        calls.append((command, timeout))
        assert command[:4] == ['ros2', 'run', 'agt_pcd2grid_exporter', 'pcd2grid_exporter']
        assert '--pcd' in command and '--package' not in command
        spec = yaml.safe_load(Path(command[command.index('--config') + 1]).read_text())
        assert spec['projection_mode'] == 'local_ground'
        assert spec['temporal_filter']['enabled'] is False
        assert spec['ground']['cell_size'] == .4
        assert spec['obstacle_height']['min'] == .2
        output = Path(command[command.index('--output') + 1])
        output.mkdir(parents=True)
        (output / 'map.pgm').write_bytes(b'P5\n2 2\n255\n' + bytes([205, 0, 254, 205]))
        (output / 'map.yaml').write_text(yaml.safe_dump({
            'image': 'map.pgm', 'resolution': .1, 'origin': [-1., -1., 0.],
            'occupied_thresh': .65, 'free_thresh': .196, 'negate': 0
        }))
        (output / 'metadata.yaml').write_text('free_cells: 1\nunknown_cells: 2\noccupied_cells: 1\n')
    c = Clouds(root, runner=fake_runner)
    result = c.grid(cloud_id, {'resolution': .1, 'ground_cell_size': .4,
                               'obstacle_min_height': .2, 'occupied_threshold': 2})
    assert len(calls) == 1 and calls[0][1] == 3600
    assert result['status'] == 'DRAFT'
    assert result['projection_mode'] == 'local_ground'
    assert result['source_pcd_sha256'] == hash_file(root / 'raw.pcd')
    assert result['projection_stats']['unknown_cells'] == 2
    assert read_pgm(Path(result['pgm']))[2] == bytes([205, 0, 254, 205])


def test_legacy_absolute_heights_are_not_misread_as_relative_heights(fixture_root):
    root, cloud_id = fixture_root
    def fake_runner(command, timeout):
        raise RuntimeError('validated parameters; no conversion needed')
    c = Clouds(root, runner=fake_runner)
    with pytest.raises(RuntimeError):
        c.grid(cloud_id, {'z_min': .15, 'z_max': 2., 'ground_min': -.3})
    snapshots = list((root / 'runs').glob('*/projection_local_ground.yaml'))
    assert len(snapshots) == 1
    cfg = yaml.safe_load(snapshots[0].read_text())
    assert cfg['obstacle_height']['min'] == .12
    assert cfg['obstacle_height']['max'] == 1.8


def test_reject_invalid_projection_before_spawn(fixture_root):
    root, cloud_id = fixture_root
    calls = []
    c = Clouds(root, runner=lambda *a, **kw: calls.append(a))
    with pytest.raises(ValueError):
        c.grid(cloud_id, {'obstacle_min_height': 3., 'obstacle_max_height': 1.})
    assert not calls
