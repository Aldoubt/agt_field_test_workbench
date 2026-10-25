"""Pose compatibility must not reinterpret timestamps as translations or alter patches."""
from pathlib import Path
import pytest
from workbench.polar_context import prepare_builder_input


@pytest.mark.parametrize('timed', [True, False])
def test_pose_adapter_preserves_exact_pose_tokens_and_points(tmp_path, timed):
    source = tmp_path/'source';(source/'patches').mkdir(parents=True)
    patch = source/'patches/000000.pcd';patch.write_bytes(b'unchanged body cloud')
    pose = '1.2345678901234567 -2.5 .0001 .999999999999 0 0 0'
    original = '000000.pcd '+('1792841271.5510201 ' if timed else '')+pose+'\n'
    name = 'poses_timed.txt' if timed else 'poses.txt'
    (source/name).write_text(original)
    destination = tmp_path/'compatible'
    result = prepare_builder_input(source, destination)
    assert (destination/'poses.txt').read_text() == '000000.pcd '+pose+'\n'
    assert (source/name).read_text() == original
    assert (destination/'patches/000000.pcd').read_bytes() == patch.read_bytes()
    assert result['entries'] == 1
    assert Path(result['source_poses']) == source/name


@pytest.mark.parametrize('row', [
    '000000.pcd 1 0 0 0 0 0 0 0',
    '000000.pcd 1 nan 0 0 1 0 0 0',
    '000000.pcd 1 0 0 0 1 0',
    '../outside.pcd 1 0 0 0 1 0 0 0'])
def test_invalid_pose_does_not_create_builder_input(tmp_path, row):
    source=tmp_path/'source';(source/'patches').mkdir(parents=True)
    (source/'patches/000000.pcd').write_bytes(b'cloud')
    (source/'poses_timed.txt').write_text(row+'\n')
    destination=tmp_path/'compatible'
    with pytest.raises(ValueError):
        prepare_builder_input(source,destination)
    assert not destination.exists()
