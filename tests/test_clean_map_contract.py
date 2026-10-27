"""Exercise real cloud/version/descriptor orchestration without ROS or hardware.

Only native PCL and descriptor executables are substituted. Keyframe ROI edits,
pose handling, immutable manifests, source hashes and asset-selection checks run
through the production Python implementation.
"""
import hashlib
import json
from pathlib import Path
import shutil
import tempfile
import unittest

import numpy as np
import yaml

from workbench.artifacts import Clouds
from workbench.keyframe_edits import keyframes_identity
from workbench.maps import Maps, hash_file, relocalization_assets, scan_context_database
from workbench.query_filters import describe_query_filter


def write_xyz(path, xyz):
    values = np.asarray(xyz, dtype='<f4').reshape(-1, 3)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes((
        'VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n'
        f'WIDTH {len(values)}\nHEIGHT 1\nPOINTS {len(values)}\nDATA binary\n'
    ).encode() + values.tobytes())


def read_xyz(path):
    return np.frombuffer(path.read_bytes().split(b'DATA binary\n')[1], dtype='<f4').reshape(-1, 3)


class NativeExecutables:
    def __init__(self):
        self.calls = []
        self.polar_inputs = []
        self.sc_inputs = []
        self.fail_candidates = False

    def count(self, executable):
        return sum(executable in call for call in self.calls)

    def __call__(self, command, timeout):
        self.calls.append(command)
        if command[:4] == ['ros2', 'run', 'agt_field_test_bringup', 'pcd_ops']:
            source, specification, output = map(Path, command[4:7])
            data = read_xyz(source).copy()
            original_count = len(data)
            for operation in yaml.safe_load(specification.read_text())['operations']:
                if operation['type'] not in ('crop', 'delete'):
                    raise AssertionError('fixture only models ROI selection')
                inside = np.all((data >= operation['min']) & (data <= operation['max']), axis=1)
                data = data[inside if operation['type'] == 'crop' else ~inside]
            if not len(data):
                raise RuntimeError('native map operation would remove all points')
            output.mkdir(parents=True, exist_ok=False)
            write_xyz(output / 'cloud.pcd', data)
            (output / 'result.yaml').write_text(yaml.safe_dump(dict(
                input_points=original_count, output_points=len(data), processed_full_resolution=True)))
        elif 'build_relocalization_assets' in command:
            source = Path(command[command.index('--map') + 1])
            output = Path(command[command.index('--output') + 1])
            (output / 'voxelmaps_coords').mkdir(parents=True)
            shutil.copyfile(source, output / 'global_map_downsampled.pcd')
            (output / 'voxelmaps_coords/source.sha').write_text(hash_file(source))
        elif 'build_relocalization_candidates' in command:
            if self.fail_candidates:
                raise RuntimeError('controlled native candidate build failure')
            source = Path(command[command.index('--map-dir') + 1])
            output = Path(command[command.index('--output') + 1])
            identity = keyframes_identity(source)
            self.polar_inputs.append(dict(directory=source,
                points=[read_xyz(source / 'patches' / row['name']).copy() for row in identity['patches']]))
            output.mkdir(parents=True)
            (output / 'polar_context.db').write_bytes(b'polar:' + identity['sha256'].encode())
            (output / 'polar_context.yaml').write_text(yaml.safe_dump({'entries': identity['entries']}))
        elif command[0] == 'python3' and command[1].endswith('build_sc_database.py'):
            source = Path(command[command.index('--keyframes') + 1])
            output = Path(command[command.index('--output') + 1])
            pcd = Path(command[command.index('--map') + 1])
            navigation = Path(command[command.index('--map-yaml') + 1])
            identity = keyframes_identity(source)
            self.sc_inputs.append(dict(directory=source, identity=identity['sha256'],
                points=[read_xyz(source / 'patches' / row['name']).copy() for row in identity['patches']]))
            output.mkdir(parents=True)
            descriptor = output / 'descriptors.npz'
            descriptor.write_bytes(b'scancontext:' + identity['sha256'].encode())
            spec = None
            if '--query-self-filter-config' in command:
                spec = describe_query_filter(command[command.index('--query-self-filter-config') + 1])['query_self_filter']
            manifest = dict(algorithm='scancontext', map_sha256=hash_file(pcd),
                navigation_yaml=str(navigation), navigation_yaml_sha256=hash_file(navigation),
                navigation_pgm_sha256=hash_file(navigation.parent / 'map.pgm'),
                descriptors_sha256=hash_file(descriptor), preprocessing={'self_filter': spec})
            manifest['metadata_sha256'] = hashlib.sha256(
                json.dumps(manifest, sort_keys=True, allow_nan=False).encode()).hexdigest()
            (output / 'manifest.json').write_text(json.dumps(manifest))
        else:
            raise AssertionError('unexpected external command: ' + repr(command))


def fixture(root):
    cloud_dir = root / 'maps/clouds/20261027T000000Z-abcdef01'
    points = [[0, 0, 0], [2, 0, 0], [4, 0, 0]]
    write_xyz(cloud_dir / 'cloud.pcd', points)
    source = root / 'keyframes'
    write_xyz(source / 'patches/a.pcd', points)
    (source / 'poses_timed.txt').write_text('a.pcd 1793073832.120497330 0 0 0 1 0 0 0\n')
    lio = root / 'fastlio.yaml'
    lio.write_text('recorded: LIO configuration fixture\n')
    record = dict(id=cloud_dir.name, pcd=str(cloud_dir / 'cloud.pcd'),
        sha256=hash_file(cloud_dir / 'cloud.pcd'), status='DRAFT', sc_keyframes=str(source),
        lio_config=str(lio), calibration={'sc_keyframes': str(source)}, name='original')
    (cloud_dir / 'manifest.json').write_text(json.dumps(record))
    native = NativeExecutables()
    return Clouds(root, runner=native), native, record, source


def navigation_version(root, cloud):
    return Maps(root)._save(3, 2, bytes([254] * 6),
        dict(resolution=1.0, origin=[0., 0., 0.], negate=0, occupied_thresh=.65, free_thresh=.196),
        dict(name='candidate', source_pcd=cloud['pcd'], source_pcd_sha256=cloud['sha256'],
             cloud_id=cloud['id'], keyframe_cleanup=cloud.get('keyframe_cleanup'),
             query_self_filter_config=cloud.get('query_self_filter_config', ''),
             query_self_filter_sha256=cloud.get('query_self_filter_sha256', '')), status='DRAFT')


DELETE_ZERO = {'type': 'delete', 'min': [-.5, -.5, -.5], 'max': [.5, .5, .5]}
DELETE_TWO = {'type': 'delete', 'min': [1.5, -.5, -.5], 'max': [2.5, .5, .5]}


def filter_config(root, name='self_filter.json', xmax=.5):
    path = root / name
    path.write_text(json.dumps(dict(schema_version=1, frame='base_link',
        boxes=[-.5, xmax, -.5, .5, -.5, .5],
        base_from_body=dict(translation=[0, 0, 0], quaternion_xyzw=[0, 0, 0, 1]))))
    return path


class CleanMapContractTests(unittest.TestCase):
    def test_clean_syncs_keyframes_and_both_descriptor_builders_without_touching_parent(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            clouds, native, original, source = fixture(root)
            parent_map = navigation_version(root, original)
            clouds.build_assets(original['id'], map_id=parent_map['id'])
            original_dir = Path(original['pcd']).parent
            frozen = {str(p): p.read_bytes() for p in original_dir.rglob('*') if p.is_file()}
            frozen[str(source / 'patches/a.pcd')] = (source / 'patches/a.pcd').read_bytes()
            frozen[str(source / 'poses_timed.txt')] = (source / 'poses_timed.txt').read_bytes()
            clean = clouds.clean(original['id'], [DELETE_ZERO])
            self.assertEqual(clean['keyframe_cleanup']['state'], 'APPLIED')
            self.assertNotEqual(Path(clean['sc_keyframes']), source)
            np.testing.assert_array_equal(read_xyz(Path(clean['sc_keyframes']) / 'patches/a.pcd'), [[2, 0, 0], [4, 0, 0]])
            candidate = navigation_version(root, clean)
            state = clouds.build_assets(clean['id'], map_id=candidate['id'])
            self.assertEqual((state['bbs'], state['polar_context'], state['scan_context']), ('READY', 'READY', 'READY'))
            np.testing.assert_array_equal(native.polar_inputs[-1]['points'][0], [[2, 0, 0], [4, 0, 0]])
            np.testing.assert_array_equal(native.sc_inputs[-1]['points'][0], [[2, 0, 0], [4, 0, 0]])
            self.assertEqual(native.sc_inputs[-1]['directory'], Path(clean['sc_keyframes']))
            for path, data in frozen.items():
                self.assertEqual(Path(path).read_bytes(), data, path)
            self.assertNotEqual(relocalization_assets(parent_map), relocalization_assets(candidate))

    def test_second_clean_keeps_first_deletion(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            clouds, _, original, source = fixture(root)
            first = clouds.clean(original['id'], [DELETE_ZERO])
            second = clouds.clean(first['id'], [DELETE_TWO])
            self.assertEqual(second['keyframe_cleanup']['state'], 'APPLIED')
            self.assertEqual(second['keyframe_cleanup']['input_points'], 2)
            np.testing.assert_array_equal(read_xyz(Path(second['sc_keyframes']) / 'patches/a.pcd'), [[4, 0, 0]])
            self.assertEqual(second['parent'], first['id'])
            np.testing.assert_array_equal(read_xyz(source / 'patches/a.pcd'), [[0, 0, 0], [2, 0, 0], [4, 0, 0]])

    def test_unsupported_or_missing_patch_never_falls_back_to_artifact_keyframes(self):
        for fault in ('compressed', 'missing'):
            with self.subTest(fault=fault), tempfile.TemporaryDirectory() as folder:
                root = Path(folder)
                clouds, native, original, source = fixture(root)
                artifact = root / 'mapping_artifact'
                fallback = artifact / 'temporal_cleanup/keyframes'
                shutil.copytree(source, fallback)
                for name in ('frames.csv', 'trajectory.csv', 'points.raw'):
                    (artifact / name).write_text('present, but must never be used after failed cleanup')
                original['source_artifact'] = str(artifact)
                (Path(original['pcd']).parent / 'manifest.json').write_text(json.dumps(original))
                patch = source / 'patches/a.pcd'
                if fault == 'compressed':
                    patch.write_bytes(patch.read_bytes().replace(b'DATA binary\n', b'DATA binary_compressed\n'))
                else:
                    patch.unlink()
                clean = clouds.clean(original['id'], [DELETE_ZERO])
                self.assertEqual(clean['keyframe_cleanup']['state'], 'UNSUPPORTED' if fault == 'compressed' else 'FAILED')
                self.assertIsNone(clean['sc_keyframes'])
                _, resolved, _, reason, _ = clouds._scan_context_inputs(clean)
                self.assertIsNone(resolved)
                self.assertTrue(reason)
                candidate = navigation_version(root, clean)
                state = clouds.build_assets(clean['id'], map_id=candidate['id'])
                self.assertEqual(state['bbs'], 'READY')
                self.assertNotEqual(state['polar_context'], 'READY')
                self.assertNotEqual(state['scan_context'], 'READY')
                self.assertFalse(native.polar_inputs)
                self.assertFalse(native.sc_inputs)
                self.assertEqual(scan_context_database(candidate), '')

    def test_keyframe_content_change_rebuilds_descriptors_but_keeps_same_map_voxels(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            clouds, native, original, source = fixture(root)
            map_record = navigation_version(root, original)
            first = clouds.build_assets(original['id'], map_id=map_record['id'])
            repeated = clouds.build_assets(original['id'], map_id=map_record['id'])
            self.assertEqual(first, repeated)
            self.assertEqual(len(native.polar_inputs), 1)
            self.assertEqual(len(native.sc_inputs), 1)
            write_xyz(source / 'patches/a.pcd', [[0, 0, 0], [2, 0, 0], [5, 0, 0]])
            changed = clouds.build_assets(original['id'], map_id=map_record['id'])
            self.assertNotEqual(changed['keyframes_sha256'], first['keyframes_sha256'])
            self.assertNotEqual(changed['sc_database'], first['sc_database'])
            self.assertEqual(len(native.polar_inputs), 2)
            self.assertEqual(len(native.sc_inputs), 2)
            self.assertEqual(native.count('build_relocalization_assets'), 1)
            self.assertEqual(hash_file(original['pcd']), original['sha256'])

    def test_changed_edited_keyframes_invalidate_old_ready_assets(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            clouds, native, original, _ = fixture(root)
            clean = clouds.clean(original['id'], [DELETE_ZERO])
            map_record = navigation_version(root, clean)
            first = clouds.build_assets(clean['id'], map_id=map_record['id'])
            self.assertEqual(first['polar_context'], 'READY')
            patch = Path(clean['sc_keyframes']) / 'patches/a.pcd'
            write_xyz(patch, [[99, 0, 0]])
            changed = clouds.build_assets(clean['id'], map_id=map_record['id'])
            self.assertEqual(changed['polar_context'], 'FAILED')
            self.assertNotEqual(changed['scan_context'], 'READY')
            self.assertEqual(len(native.polar_inputs), 1)
            self.assertEqual(relocalization_assets(map_record), '')
            self.assertEqual(scan_context_database(map_record), '')

    def test_failed_candidate_rebuild_cannot_reactivate_old_database(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            clouds, native, original, source = fixture(root)
            map_record = navigation_version(root, original)
            clouds.build_assets(original['id'], map_id=map_record['id'])
            database = Path(original['pcd']).parent / 'relocalization/polar_context.db'
            old = database.read_bytes()
            write_xyz(source / 'patches/a.pcd', [[0, 0, 0], [6, 0, 0]])
            native.fail_candidates = True
            changed = clouds.build_assets(original['id'], map_id=map_record['id'])
            self.assertEqual(changed['polar_context'], 'FAILED')
            self.assertEqual(database.read_bytes(), old)
            self.assertEqual(relocalization_assets(map_record), '')

    def test_interrupted_rebuild_retires_both_old_indexes_until_successful_retry(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            clouds, native, original, source = fixture(root)
            map_record = navigation_version(root, original)
            initial = clouds.build_assets(original['id'], map_id=map_record['id'])
            assets = Path(original['pcd']).parent / 'relocalization'
            database = assets / 'polar_context.db'
            old_database = database.read_bytes()
            self.assertEqual(relocalization_assets(map_record), str(assets))
            self.assertEqual(scan_context_database(map_record), initial['sc_database'])
            write_xyz(source / 'patches/a.pcd', [[0, 0, 0], [6, 0, 0]])
            interrupted = []

            def interrupt_candidates(command, timeout):
                if 'build_relocalization_candidates' in command:
                    pending = json.loads((assets / 'asset_status.json').read_text())
                    self.assertEqual(pending['bbs'], 'READY')
                    self.assertEqual(pending['polar_context'], 'BUILDING')
                    self.assertEqual(pending['scan_context'], 'BUILDING')
                    self.assertNotIn('sc_database', pending)
                    self.assertEqual(relocalization_assets(map_record), '')
                    self.assertEqual(scan_context_database(map_record), '')
                    interrupted.append(command)
                    raise KeyboardInterrupt('controlled interruption during candidate rebuild')
                return native(command, timeout)

            clouds.runner = interrupt_candidates
            with self.assertRaises(KeyboardInterrupt):
                clouds.build_assets(original['id'], map_id=map_record['id'])
            self.assertEqual(len(interrupted), 1)
            # Old complete files still exist, but their obsolete READY state
            # must not survive an uncaught interruption or a process restart.
            self.assertEqual(database.read_bytes(), old_database)
            self.assertTrue((Path(initial['sc_database']) / 'manifest.json').is_file())
            self.assertEqual(relocalization_assets(map_record), '')
            self.assertEqual(scan_context_database(map_record), '')
            persisted = json.loads((assets / 'asset_status.json').read_text())
            self.assertEqual(persisted['bbs'], 'READY')

            clouds.runner = native
            retried = clouds.build_assets(original['id'], map_id=map_record['id'])
            self.assertEqual((retried['bbs'], retried['polar_context'], retried['scan_context']),
                             ('READY', 'READY', 'READY'))
            self.assertNotEqual(retried['keyframes_sha256'], initial['keyframes_sha256'])
            self.assertNotEqual(retried['sc_database'], initial['sc_database'])
            self.assertEqual(relocalization_assets(map_record), str(assets))
            self.assertEqual(scan_context_database(map_record), retried['sc_database'])
            self.assertEqual(native.count('build_relocalization_assets'), 1)

    def test_unready_or_modified_polar_database_is_not_exposed_for_loading(self):
        for fault in ('not_generated', 'changed_database', 'missing_ready_database'):
            with self.subTest(fault=fault), tempfile.TemporaryDirectory() as folder:
                root = Path(folder)
                clouds, _, original, _ = fixture(root)
                map_record = navigation_version(root, original)
                clouds.build_assets(original['id'], map_id=map_record['id'])
                assets = Path(original['pcd']).parent / 'relocalization'
                self.assertEqual(relocalization_assets(map_record), str(assets))
                database = assets / 'polar_context.db'
                if fault == 'not_generated':
                    status_path = assets / 'asset_status.json'
                    status = json.loads(status_path.read_text())
                    status['polar_context'] = 'NOT_GENERATED'
                    status_path.write_text(json.dumps(status))
                    self.assertTrue(database.is_file())
                elif fault == 'changed_database':
                    database.write_bytes(database.read_bytes() + b'changed descriptor bytes')
                else:
                    database.unlink()
                self.assertEqual(relocalization_assets(map_record), '')
                self.assertEqual(hash_file(original['pcd']), original['sha256'])

    def test_import_with_self_mask_prepares_matching_polar_and_sc_keyframes(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            clouds, native, original, source = fixture(root)
            config = filter_config(root)
            imported = clouds.import_pcd(original['pcd'], calibration=dict(
                sc_keyframes=str(source), internal_config=original['lio_config'], query_self_filter_config=str(config)))
            self.assertNotEqual(Path(imported['sc_keyframes']), source)
            self.assertEqual(imported['keyframe_cleanup']['state'], 'APPLIED')
            np.testing.assert_array_equal(read_xyz(Path(imported['sc_keyframes']) / 'patches/a.pcd'), [[2, 0, 0], [4, 0, 0]])
            # A keyframe mask must not pretend to have removed a moving body from
            # the fused global map without trajectory-based evidence.
            np.testing.assert_array_equal(read_xyz(Path(imported['pcd'])), read_xyz(Path(original['pcd'])))
            map_record = navigation_version(root, imported)
            state = clouds.build_assets(imported['id'], map_id=map_record['id'])
            self.assertEqual(state['polar_context'], 'READY')
            self.assertEqual(state['scan_context'], 'READY')
            np.testing.assert_array_equal(native.polar_inputs[-1]['points'][0], [[2, 0, 0], [4, 0, 0]])
            self.assertEqual(hash_file(imported['query_self_filter_config']), hash_file(config))

    def test_masked_child_cannot_disable_or_change_mask_without_original_keyframes(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            clouds, native, original, _ = fixture(root)
            initial = filter_config(root)
            modified = filter_config(root, name='changed_filter.json', xmax=2.5)
            masked = clouds.clean(original['id'], [], query_self_filter_config=str(initial))
            self.assertEqual(masked['keyframe_cleanup']['state'], 'APPLIED')
            calls_before = len(native.calls)
            for next_mask in ('', str(modified)):
                with self.subTest(next_mask=next_mask), self.assertRaises(ValueError):
                    clouds.clean(masked['id'], [], query_self_filter_config=next_mask)
            self.assertEqual(len(native.calls), calls_before)


if __name__ == '__main__':
    unittest.main()
