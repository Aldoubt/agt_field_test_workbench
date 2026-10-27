"""Real PCD/pose regressions for offline keyframe geometry synchronization."""
import hashlib
import json
import math
from pathlib import Path
import shutil
import tempfile
import unittest
from unittest import mock

import numpy as np

from workbench import keyframe_edits
from workbench.keyframe_edits import sync_keyframe_edits, keyframes_identity, UnsupportedPCDEncoding


DTYPE = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'),
                  ('intensity', '<u2'), ('ring', '<u2'), ('offset_time', '<u4'),
                  ('normal', '<f4', (3,))])


def binary_cloud(path, points):
    path.parent.mkdir(parents=True, exist_ok=True)
    header = ('# original metadata\nVERSION .7\nFIELDS x y z intensity ring offset_time normal\n'
              'SIZE 4 4 4 2 2 4 4\nTYPE F F F U U U F\nCOUNT 1 1 1 1 1 1 3\n'
              f'WIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 1 2 3 1 0 0 0\n'
              f'POINTS {len(points)}\nDATA binary\n')
    path.write_bytes(header.encode() + points.tobytes())


def records(xyz):
    output = np.zeros(len(xyz), dtype=DTYPE)
    for i, axis in enumerate(('x', 'y', 'z')):
        output[axis] = np.asarray(xyz)[:, i]
    output['intensity'] = np.arange(len(xyz)) + 32000
    output['ring'] = np.arange(len(xyz)) + 9
    output['offset_time'] = np.arange(len(xyz), dtype=np.uint32) + 4000000000
    output['normal'] = [[.25, .5, .75]] * len(xyz)
    return output


def read_records(path):
    payload = path.read_bytes().split(b'DATA binary\n', 1)[1]
    return np.frombuffer(payload, dtype=DTYPE)


def setup_source(root, points, poses='a.pcd 1793073832.120497330 0 0 0 1 0 0 0\n'):
    source = root / 'source'
    source.mkdir()
    binary_cloud(source / 'patches/a.pcd', points)
    (source / 'poses_timed.txt').write_text(poses)
    return source


class KeyframeEditTests(unittest.TestCase):
    def test_translated_tilted_frame_selects_map_xyz_but_preserves_entire_body_records(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            # +90 degree pitch: body +X points toward map -Z.
            q = math.sqrt(.5)
            pose = f'a.pcd 1793073832.120497330 10 20 30 {q:.17g} 0 {q:.17g} 0\n'
            values = records([[1, 0, 0], [0, 0, 1], [-1, 0, 0]])
            source = setup_source(root, values, pose)
            original = (source / 'patches/a.pcd').read_bytes()
            destination = root / 'edited'
            report = sync_keyframe_edits(source, destination, [
                {'type': 'delete', 'min': [9.9, 19.9, 28.9], 'max': [10.1, 20.1, 29.1]}])
            self.assertEqual(read_records(destination / 'patches/a.pcd').tobytes(), values[[1, 2]].tobytes())
            self.assertEqual((destination / 'poses_timed.txt').read_text(), pose)
            self.assertEqual((source / 'patches/a.pcd').read_bytes(), original)
            header = (destination / 'patches/a.pcd').read_bytes().split(b'DATA binary\n')[0]
            self.assertIn(b'VIEWPOINT 1 2 3 1 0 0 0', header)
            self.assertIn(b'COUNT 1 1 1 1 1 1 3', header)
            self.assertEqual((report['input_points'], report['output_points']), (3, 2))
            self.assertEqual(report['source_poses_sha256'], hashlib.sha256(pose.encode()).hexdigest())
            self.assertEqual(json.loads((destination / 'edit_provenance.json').read_text()), report)

    def test_crop_and_delete_are_inclusive_on_boundaries(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            values = records([[-1, 0, 0], [0, 0, 0], [1, 0, 0], [2, 0, 0]])
            source = setup_source(root, values)
            destination = root / 'edited'
            report = sync_keyframe_edits(source, destination, [
                {'type': 'crop', 'min': [-1, -1, -1], 'max': [1, 1, 1]},
                {'type': 'delete', 'min': [0, -1, -1], 'max': [1, 1, 1]}])
            self.assertEqual(read_records(destination / 'patches/a.pcd').tobytes(), values[:1].tobytes())
            self.assertEqual(report['removed_points'], 3)

    def test_ascii_preserves_large_integer_time_and_numeric_spelling(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            source = root / 'source'
            (source / 'patches').mkdir(parents=True)
            data = (b'FIELDS intensity x y z time\nSIZE 2 8 8 8 8\nTYPE U F F F U\n'
                    b'COUNT 1 1 1 1 1\nWIDTH 2\nHEIGHT 1\nPOINTS 2\nDATA ascii\n'
                    b'42 1.0000000000000001 0.0 -0.000 1793073832120497330\n'
                    b'99 2e0 0.0 0.0 1793073832120497331\n')
            (source / 'patches/a.pcd').write_bytes(data)
            pose = b'# original frame\na.pcd 10.000000000 0 0 0 1.0 0 0 0\n'
            (source / 'poses_timed.txt').write_bytes(pose)
            destination = root / 'edited'
            sync_keyframe_edits(source, destination, [
                {'type': 'delete', 'min': [1.5, -1, -1], 'max': [2.5, 1, 1]}])
            output = (destination / 'patches/a.pcd').read_bytes()
            self.assertEqual(output.split(b'DATA ascii\n')[1], data.split(b'DATA ascii\n')[1].splitlines(keepends=True)[0])
            self.assertEqual((destination / 'poses_timed.txt').read_bytes(), pose)

    def test_empty_frame_is_reported_and_omitted_without_changing_other_pose_tokens(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            first = 'a.pcd 1.000000001 0 0 0 1 0 0 0\n'
            second = 'b.pcd 1.000000002 10 0 0 1 0 0 0\n'
            source = setup_source(root, records([[0, 0, 0]]), '# frames\n' + first + second)
            binary_cloud(source / 'patches/b.pcd', records([[0, 0, 0]]))
            destination = root / 'edited'
            report = sync_keyframe_edits(source, destination, [
                {'type': 'delete', 'min': [-1, -1, -1], 'max': [1, 1, 1]}])
            self.assertEqual(report['empty_frames'], ['a.pcd'])
            self.assertEqual(report['output_frames'], 1)
            self.assertFalse((destination / 'patches/a.pcd').exists())
            self.assertEqual((destination / 'poses_timed.txt').read_text(), '# frames\n' + second)
            self.assertEqual(report['frames'][0]['output_points'], 0)

    def test_consecutive_edits_start_from_previous_retained_records(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            values = records([[0, 0, 0], [2, 0, 0], [4, 0, 0]])
            source = setup_source(root, values)
            sync_keyframe_edits(source, root / 'first', [
                {'type': 'delete', 'min': [-1, -1, -1], 'max': [1, 1, 1]}])
            report = sync_keyframe_edits(root / 'first', root / 'second', [
                {'type': 'delete', 'min': [1, -1, -1], 'max': [3, 1, 1]}])
            self.assertEqual(report['input_points'], 2)
            self.assertEqual(read_records(root / 'second/patches/a.pcd').tobytes(), values[2:].tobytes())

    def test_voxel_is_explicit_sampling_only_and_not_applied_to_body_records(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            values = records([[.1, 0, 0], [.2, 0, 0]])
            source = setup_source(root, values)
            report = sync_keyframe_edits(source, root / 'edited', [{'type': 'voxel', 'size': 1}])
            self.assertEqual(read_records(root / 'edited/patches/a.pcd').tobytes(), values.tobytes())
            self.assertEqual(report['sampling_operations_not_applied'], [{'type': 'voxel', 'size': 1.0}])
            self.assertEqual(report['applied_operations'], [])
            with self.assertRaisesRegex(ValueError, 'voxel then ROI'):
                sync_keyframe_edits(source, root / 'ambiguous', [
                    {'type': 'voxel', 'size': 1},
                    {'type': 'delete', 'min': [-1, -1, -1], 'max': [1, 1, 1]}])
            self.assertFalse((root / 'ambiguous').exists())

    def test_unsupported_or_invalid_patch_never_publishes_partial_output(self):
        for corrupt, expected in [
            (lambda data: data.replace(b'DATA binary\n', b'DATA binary_compressed\n'), UnsupportedPCDEncoding),
            (lambda data: data[:-1], ValueError),
            (lambda data: data.replace(b'POINTS 1\n', b'POINTS 2\n'), ValueError),
        ]:
            with self.subTest(error=expected), tempfile.TemporaryDirectory() as folder:
                root = Path(folder)
                source = setup_source(root, records([[1, 0, 0]]),
                    'a.pcd 1 0 0 0 1 0 0 0\nb.pcd 2 0 0 0 1 0 0 0\n')
                original = (source / 'patches/a.pcd').read_bytes()
                (source / 'patches/b.pcd').write_bytes(corrupt(original))
                with self.assertRaises(expected):
                    sync_keyframe_edits(source, root / 'edited', [])
                self.assertFalse((root / 'edited').exists())
                self.assertFalse(list(root.glob('.edited-*')))
                self.assertEqual((source / 'patches/a.pcd').read_bytes(), original)

    def test_all_empty_and_bad_pose_fail_without_a_ready_source(self):
        for pose, operations in [
            ('a.pcd 1 0 0 0 1 0 0 0\n', [{'type': 'delete', 'min': [-1, -1, -1], 'max': [1, 1, 1]}]),
            ('a.pcd 1 0 0 0 0 0 0 0\n', []),
            ('../a.pcd 1 0 0 0 1 0 0 0\n', []),
            ('a.pcd 1 0 0 0 1 0 0 0\n./a.pcd 2 0 0 0 1 0 0 0\n', []),
            ('a.pcd nan 0 0 0 1 0 0 0\n', []),
            ('a.pcd 1 0 0 0 1 0 0 0\na.pcd 2 0 0 0 1 0 0 0\n', []),
        ]:
            with self.subTest(pose=pose), tempfile.TemporaryDirectory() as folder:
                root = Path(folder)
                source = setup_source(root, records([[0, 0, 0]]), pose)
                with self.assertRaises(ValueError):
                    sync_keyframe_edits(source, root / 'edited', operations)
                self.assertFalse((root / 'edited').exists())

    def test_existing_destination_is_not_overwritten(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            source = setup_source(root, records([[1, 0, 0]]))
            destination = root / 'edited'
            destination.mkdir()
            (destination / 'keep').write_text('previous version')
            with self.assertRaisesRegex(ValueError, 'already exists'):
                sync_keyframe_edits(source, destination, [])
            self.assertEqual((destination / 'keep').read_text(), 'previous version')

    def test_source_change_during_edit_does_not_publish_mixed_snapshot(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            source = setup_source(root, records([[1, 0, 0]]))
            read = keyframe_edits._read_pcd
            def change_source(path):
                cloud = read(path)
                with path.open('ab') as stream:
                    stream.write(b'changed after snapshot')
                return cloud
            with mock.patch.object(keyframe_edits, '_read_pcd', side_effect=change_source):
                with self.assertRaisesRegex(ValueError, 'source keyframe changed'):
                    sync_keyframe_edits(source, root / 'edited', [])
            self.assertFalse((root / 'edited').exists())

    def test_destination_created_during_edit_is_preserved(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            source = setup_source(root, records([[1, 0, 0]]))
            destination = root / 'edited'
            read = keyframe_edits._read_pcd
            def create_other_output(path):
                cloud = read(path)
                destination.mkdir()
                (destination / 'keep').write_text('another writer')
                return cloud
            with mock.patch.object(keyframe_edits, '_read_pcd', side_effect=create_other_output):
                with self.assertRaises(FileExistsError):
                    sync_keyframe_edits(source, destination, [])
            self.assertEqual((destination / 'keep').read_text(), 'another writer')
            self.assertFalse(list(root.glob('.edited-*')))

    def test_self_mask_uses_tilted_body_to_base_before_map_roi(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            values = records([[1, 0, 0], [0, 0, 1], [2, 0, 0], [0, 2, 0]])
            source = setup_source(root, values, 'a.pcd 1.000000001 10 0 0 1 0 0 0\n')
            spec = dict(frame='base_link', boxes=[.4, .6, -.1, .1, -.1, .1],
                base_from_body_rotation=[[0, 0, 1], [0, 1, 0], [-1, 0, 0]],
                base_from_body_translation=[.5, 0, 1])
            report = sync_keyframe_edits(source, root / 'edited', [
                {'type': 'delete', 'min': [9.9, -.1, .9], 'max': [10.1, .1, 1.1]}],
                self_filter=spec)
            self.assertEqual(read_records(root / 'edited/patches/a.pcd').tobytes(), values[[2, 3]].tobytes())
            self.assertEqual(report['self_filter'], spec)
            self.assertEqual((report['self_filtered_points'], report['roi_removed_points']), (1, 1))
            self.assertIn('body keyframes only', report['self_filter_scope'])
            self.assertEqual(read_records(source / 'patches/a.pcd').tobytes(), values.tobytes())

    def test_empty_self_filter_keeps_previous_behavior(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            values = records([[1, 0, 0]])
            source = setup_source(root, values)
            report = sync_keyframe_edits(source, root / 'edited', [], self_filter={})
            self.assertIsNone(report['self_filter'])
            self.assertEqual(report['self_filtered_points'], 0)
            self.assertEqual(read_records(root / 'edited/patches/a.pcd').tobytes(), values.tobytes())

    def test_keyframe_identity_tracks_referenced_content_not_directory_or_unrelated_files(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            values = records([[1, 0, 0]])
            source = setup_source(root, values)
            initial = keyframes_identity(source)
            shutil.copytree(source, root / 'relocated')
            self.assertEqual(keyframes_identity(root / 'relocated'), initial)
            (source / 'patches/unreferenced.pcd').write_bytes(b'unrelated')
            self.assertEqual(keyframes_identity(source), initial)
            values['intensity'] += 1
            binary_cloud(source / 'patches/a.pcd', values)
            changed_patch = keyframes_identity(source)
            self.assertNotEqual(changed_patch['sha256'], initial['sha256'])
            self.assertEqual(changed_patch['poses_sha256'], initial['poses_sha256'])
            poses = source / 'poses_timed.txt'
            poses.write_text(poses.read_text().replace('1793073832.120497330', '1793073832.120497331'))
            changed_time = keyframes_identity(source)
            self.assertNotEqual(changed_time['sha256'], changed_patch['sha256'])
            self.assertEqual(changed_time['patches'], changed_patch['patches'])

    def test_identity_detects_mutation_while_hashing(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            source = setup_source(root, records([[1, 0, 0]]))
            sha = keyframe_edits._sha
            def changed(path):
                digest = sha(path)
                if path.suffix == '.pcd':
                    with path.open('ab') as stream:
                        stream.write(b'changed')
                return digest
            with mock.patch.object(keyframe_edits, '_sha', side_effect=changed):
                with self.assertRaisesRegex(ValueError, 'source changed'):
                    keyframes_identity(source)


if __name__ == '__main__':
    unittest.main()
