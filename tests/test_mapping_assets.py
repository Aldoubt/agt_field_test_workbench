"""ROS-independent coverage of same-PCD BBS asset generation and failures."""
import hashlib
import json
import tempfile
from pathlib import Path

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
            (output/'polar_context.db').write_bytes(b'candidate-index')
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
    assert Path(state['sc_database']).is_file()
    assert any('build_relocalization_candidates' in command for command in calls)
    assert any(command[0]=='python3' and command[1].endswith('build_sc_database.py') for command in calls)
