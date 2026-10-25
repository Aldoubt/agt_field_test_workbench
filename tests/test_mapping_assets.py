"""ROS-independent coverage of same-PCD BBS asset generation and failures."""
import hashlib
import json
import tempfile
from pathlib import Path
import pytest

from workbench.artifacts import Clouds
from workbench.maps import Maps, hash_file, asset_summary, relocalization_assets


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
        assert relocalization_assets({'source_pcd':str(pcd),'source_pcd_sha256':hash_file(pcd)}) == str(folder/'relocalization')


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
        assert relocalization_assets({'source_pcd':str(pcd),'source_pcd_sha256':hash_file(pcd)}) == ''


def test_build_assets_generates_sc_and_candidate_bbs_from_same_source(tmp_path):
    root=Path(tmp_path)
    folder,pcd=cloud_fixture(root)
    keyframes=root/'keyframes';(keyframes/'patches').mkdir(parents=True)
    (keyframes/'poses_timed.txt').write_text('000000.pcd 1.0 0 0 0 1 0 0 0\n')
    (keyframes/'patches/000000.pcd').write_bytes(b'body-frame keyframe')
    lio=root/'fastlio.yaml';lio.write_text('extrinsic: fixed-for-test\n')
    cloud=json.loads((folder/'manifest.json').read_text())
    cloud.update(sc_keyframes=str(keyframes),lio_config=str(lio))
    (folder/'manifest.json').write_text(json.dumps(cloud))
    maps=Maps(root)
    map_record=maps._save(3,2,bytes([0,254,205,0,254,205]),
        {'resolution':1.0,'origin':[0.,0.,0.],'negate':0,'occupied_thresh':.65,'free_thresh':.196},
        {'name':'asset-test','source_pcd':str(pcd),'source_pcd_sha256':hash_file(pcd),
         'cloud_id':folder.name,'source_yaml':None,'source_pgm_sha256':None,
         'source_bag':None,'calibration':None},status='DRAFT')
    calls=[]
    def fake_runner(command,timeout):
        calls.append(command)
        if command[:4]==['ros2','run','agt_global_relocalization_native','build_relocalization_assets']:
            output=Path(command[command.index('--output')+1])
            (output/'voxelmaps_coords').mkdir(parents=True)
            (output/'global_map_downsampled.pcd').write_bytes(b'downsampled')
        elif command[:4]==['ros2','run','agt_global_relocalization_native','build_relocalization_candidates']:
            output=Path(command[command.index('--output')+1])
            compatible=Path(command[command.index('--map-dir')+1])
            assert (compatible/'poses.txt').read_text() == '000000.pcd 0 0 0 1 0 0 0\n'
            assert (compatible/'patches/000000.pcd').read_bytes() == b'body-frame keyframe'
            output.mkdir(parents=True)
            (output/'polar_context.db').write_bytes(b'candidate-index')
            (output/'polar_context.yaml').write_text('entries: 1\n')
        elif command[0]=='python3' and command[1].endswith('build_sc_database.py'):
            output=Path(command[command.index('--output')+1]);output.mkdir(parents=True)
            descriptor=output/'descriptors.npz';descriptor.write_bytes(b'descriptor-fixture')
            yaml_path=Path(command[command.index('--map-yaml')+1])
            pgm=yaml_path.parent/'map.pgm'
            manifest={'algorithm':'scancontext','map_sha256':hash_file(pcd),
                'navigation_yaml':str(yaml_path),'navigation_yaml_sha256':hash_file(yaml_path),
                'navigation_pgm_sha256':hash_file(pgm),'descriptors_sha256':hash_file(descriptor)}
            manifest['metadata_sha256']=hashlib.sha256(
                json.dumps(manifest,sort_keys=True,allow_nan=False).encode()).hexdigest()
            (output/'manifest.json').write_text(json.dumps(manifest))
        else:
            raise AssertionError('unexpected asset builder command: '+repr(command))
    clouds=Clouds(root,runner=fake_runner)
    state=clouds.build_assets(folder.name,map_id=map_record['id'])
    assert state['state']=='READY' and state['bbs']=='READY'
    assert state['polar_context']=='READY'
    assert state['scan_context']=='READY'
    assert Path(state['sc_database']).is_dir()
    assert any('build_relocalization_candidates' in command for command in calls)
    assert any(command[0]=='python3' and command[1].endswith('build_sc_database.py') for command in calls)


@pytest.mark.parametrize('failure', ['interrupted', 'empty_index'])
def test_candidate_failure_never_publishes_partial_database(tmp_path, failure):
    folder, pcd = cloud_fixture(tmp_path)
    keyframes = tmp_path/'keyframes'
    (keyframes/'patches').mkdir(parents=True)
    poses = keyframes/'poses_timed.txt'
    poses.write_text('000000.pcd 1.0 0 0 0 1 0 0 0\n')
    (keyframes/'patches/000000.pcd').write_bytes(b'original patch')
    cloud = json.loads((folder/'manifest.json').read_text())
    cloud['sc_keyframes'] = str(keyframes)
    (folder/'manifest.json').write_text(json.dumps(cloud))
    assets = folder/'relocalization'
    (assets/'voxelmaps_coords').mkdir(parents=True)
    (assets/'global_map_downsampled.pcd').write_bytes(b'existing voxel asset')
    source_sha = hash_file(pcd)
    poses_sha = hash_file(poses)
    (assets/'asset_status.json').write_text(json.dumps({
        'source_sha256': source_sha, 'state': 'READY', 'bbs': 'READY'}))

    def failed_builder(command, timeout):
        assert command[3] == 'build_relocalization_candidates'
        output = Path(command[command.index('--output')+1])
        output.mkdir()
        (output/'polar_context.db').write_bytes(b'incomplete index')
        if failure == 'interrupted':
            raise RuntimeError('candidate builder interrupted')
        (output/'polar_context.yaml').write_text('entries: 0\n')

    result = Clouds(tmp_path, runner=failed_builder).build_assets(folder.name)
    assert result['bbs'] == 'READY'
    assert result['polar_context'] == 'FAILED'
    assert ('interrupted' if failure == 'interrupted' else 'empty index') in result['polar_context_error']
    assert not (assets/'polar_context.db').exists()
    assert not (assets/'polar_context.yaml').exists()
    assert hash_file(pcd) == source_sha and hash_file(poses) == poses_sha
    assert (assets/'global_map_downsampled.pcd').read_bytes() == b'existing voxel asset'
