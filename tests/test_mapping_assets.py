"""ROS-independent coverage of same-PCD BBS asset generation and failures."""
import json
import tempfile
from pathlib import Path

from workbench.artifacts import Clouds
from workbench.maps import hash_file, asset_summary


def cloud_fixture(root):
    folder = root / 'maps/clouds/20261024T000000Z-abcdef01'
    folder.mkdir(parents=True)
    pcd = folder / 'cloud.pcd'
    pcd.write_bytes(b'synthetic PCD fixture; native builder is mocked')
    (folder / 'manifest.json').write_text(json.dumps({
        'id': folder.name, 'pcd': str(pcd), 'sha256': hash_file(pcd)}))
    return folder, pcd


def test_bbs_asset_build_uses_same_immutable_cloud_and_is_cached():
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        folder, pcd = cloud_fixture(root)
        calls = []
        def fake_runner(command, timeout):
            calls.append(command)
            assert command[:4] == ['ros2', 'run', 'agt_global_relocalization_native',
                                    'build_relocalization_assets']
            assert command[command.index('--map') + 1] == str(pcd)
            assert timeout == 1200
            output = Path(command[command.index('--output') + 1])
            (output / 'voxelmaps_coords').mkdir(parents=True)
            (output / 'global_map_downsampled.pcd').write_bytes(b'downsampled')
        clouds = Clouds(root, runner=fake_runner)
        first = clouds.build_assets(folder.name)
        assert first['state'] == 'READY' and first['bbs'] == 'READY'
        assert first['polar_context'] == 'NOT_GENERATED_NO_KEYFRAME_PATCHES'
        second = clouds.build_assets(folder.name)
        assert len(calls) == 1 and second == first
        summary = asset_summary({'source_pcd': str(pcd),
                                 'source_pcd_sha256': hash_file(pcd)})
        assert 'BBS 体素资产：已生成' in summary
        assert 'Polar Context：未生成' in summary


def test_bbs_build_failure_does_not_claim_assets_ready():
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        folder, pcd = cloud_fixture(root)
        def failed(_command, _timeout):
            raise RuntimeError('BBS native executable missing')
        clouds = Clouds(root, runner=failed)
        result = clouds.build_assets(folder.name)
        assert result['state'] == 'FAILED' and result['bbs'] == 'FAILED'
        assert result['source_sha256'] == hash_file(pcd)
        summary = asset_summary({'source_pcd': str(pcd),
                                 'source_pcd_sha256': hash_file(pcd)})
        assert '构建失败' in summary
        assert not (folder / 'relocalization/voxelmaps_coords').exists()
