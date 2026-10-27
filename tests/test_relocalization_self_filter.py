"""Calibration, SC identity and actual query-loop checks without robot motion."""
import ast
import copy
from contextlib import contextmanager
import json
import math
import os
from pathlib import Path
import sys
import tempfile
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np
from scipy.spatial.transform import Rotation

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / 'agt_field_test_bringup'))
sys.path.insert(0, str(REPO / 'agt_sc_kiss'))
from agt_field_test_bringup.relocalization_self_filter import (
    QuerySelfFilter, load_self_filter, make_self_filter_config,
    normalize_self_filter, prepare_query_self_filter, self_filter_keep_mask,
)
from agt_sc_kiss.geometry import assert_contract, contract, preprocess
from agt_sc_kiss import database
import agt_sc_kiss


def pose(rotation, translation):
    return dict(zip(('x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'),
                    [*translation, *Rotation.from_matrix(rotation).as_quat()]))


def inverse(value):
    r = Rotation.from_quat([value[k] for k in ('qx', 'qy', 'qz', 'qw')]).as_matrix()
    t = np.asarray([value[k] for k in ('x', 'y', 'z')])
    return pose(r.T, -r.T @ t)


def transform(rotation, translation):
    q = Rotation.from_matrix(rotation).as_quat()
    return SimpleNamespace(translation=SimpleNamespace(**dict(zip(('x', 'y', 'z'), translation))),
                           rotation=SimpleNamespace(**dict(zip(('x', 'y', 'z', 'w'), q))))


@contextmanager
def active_lio_fixture(translation, quaternion):
    """Isolate the established LIO file parser; check its selected path at calls."""
    loader = Mock(return_value=(list(translation), list(quaternion)))
    adapter = ModuleType('agt_batch_lio_adapter')
    extrinsics = ModuleType('agt_batch_lio_adapter.extrinsics')
    extrinsics.load_lio_body_to_lidar = loader
    adapter.extrinsics = extrinsics
    with patch.dict(sys.modules, {'agt_batch_lio_adapter':adapter,
                                  'agt_batch_lio_adapter.extrinsics':extrinsics}):
        yield loader


class SelfFilterTests(unittest.TestCase):
    def setUp(self):
        self.r = Rotation.from_euler('xyz', [4., 25., -8.], degrees=True).as_matrix()
        self.t = np.asarray([.23, -.07, .63])
        self.boxes = [-.2, .2, -.1, .1, .7, .9]
        self.config = make_self_filter_config(self.boxes, self.t, Rotation.from_matrix(self.r).as_quat())
        self.spec = normalize_self_filter(self.config)

    def test_tilted_body_filter_preserves_coordinates_and_outside_points(self):
        base = np.asarray([[0., 0., .8], [.2, 0., .8], [.21, 0., .8], [3., 1., 1.2]])
        body = (base - self.t) @ self.r
        before = body.copy()
        keep = self_filter_keep_mask(body, self.spec)
        np.testing.assert_array_equal(keep, [False, False, True, True])
        np.testing.assert_array_equal(body, before)
        self.assertEqual(len(self_filter_keep_mask(np.empty((0, 3)), self.spec)), 0)

    def test_mask_before_voxel_does_not_discard_environment_point_in_same_voxel(self):
        mask = normalize_self_filter(make_self_filter_config(
            [1., 1.05, -.1, .1, -.1, .1], [0., 0., 0.], [0., 0., 0., 1.]))
        legacy = contract([0., 0., 0.], [0., 0., 0., 1.])
        filtered = contract([0., 0., 0.], [0., 0., 0., 1.], self_filter=mask)
        points = np.asarray([[1.01, 0., 0.], [1.13, 0., 0.], [4., 1., 0.]])
        np.testing.assert_array_equal(preprocess(points, legacy)[0], points[0])
        np.testing.assert_array_equal(preprocess(points, filtered)[0], points[1])

    def test_empty_config_preserves_legacy_contract_and_no_calibration_access(self):
        off = normalize_self_filter(dict(schema_version=1, frame='base_link', boxes=[]))
        old = contract([0., 0., 0.], [0., 0., 0., 1.])
        self.assertIsNone(off)
        self.assertEqual(old, contract([0., 0., 0.], [0., 0., 0., 1.], self_filter=off))
        # Any calibration parameter or node method access would fail this fixture.
        values = {'query_self_filter_config':''}
        node = SimpleNamespace(get_parameter=lambda name: SimpleNamespace(value=values[name]))
        self.assertIsNone(prepare_query_self_filter(node))
        self.assertIsNone(node._query_self_filter_spec)
        with tempfile.TemporaryDirectory() as td:
            config = Path(td) / 'empty.json'
            config.write_text(json.dumps(dict(schema_version=1, frame='base_link', boxes=[])))
            values['query_self_filter_config'] = str(config)
            self.assertIsNone(prepare_query_self_filter(node))

    def test_contract_rejects_enable_disable_and_changed_mask_both_directions(self):
        old = contract([0., 0., 0.], [0., 0., 0., 1.])
        new = contract([0., 0., 0.], [0., 0., 0., 1.], self_filter=self.spec)
        assert_contract(old, copy.deepcopy(old))
        assert_contract(new, copy.deepcopy(new))
        for actual, expected in ((new, old), (old, new)):
            with self.assertRaisesRegex(ValueError, 'mismatch'):
                assert_contract(actual, expected)
        changed = copy.deepcopy(new)
        changed['self_filter']['boxes'][1] += .01
        with self.assertRaisesRegex(ValueError, 'mismatch'):
            assert_contract(changed, new)

    def test_invalid_frame_bounds_and_nonfinite_transforms_are_rejected(self):
        for change in (dict(frame='base_footprint'), dict(boxes=[0., 0., 0., 1., 0., 1.]),
                       dict(boxes=[False, 1., 0., 1., 0., 1.]),
                       dict(base_from_body={'translation':[float('nan'), 0., 0.],
                                            'quaternion_xyzw':[0., 0., 0., 1.]})):
            with self.subTest(change=change), self.assertRaises(ValueError):
                normalize_self_filter(dict(self.config, **change))

    def test_runtime_frozen_config_transform_check_and_raw_sensor_predicate(self):
        with tempfile.TemporaryDirectory() as td:
            config = Path(td) / 'mask.json'
            config.write_text(json.dumps(self.config))
            values = {'query_self_filter_config':str(config), 'mount_base_frame':'base_link'}
            node = SimpleNamespace(get_parameter=lambda name:SimpleNamespace(value=values[name]),
                body_to_base_pose=lambda:inverse(pose(self.r, self.t)), inverse_pose=inverse,
                query_frame_contract=lambda:('base_link','base_link',None))
            query = prepare_query_self_filter(node)
            sensor_r = Rotation.from_euler('y', -15., degrees=True).as_matrix()
            sensor_t = np.asarray([.5, 0., 1.])
            keep = query.predicate(transform(sensor_r, sensor_t))
            base = np.asarray([[0., 0., .8], [3., 1., .8]])
            raw = (base - sensor_t) @ sensor_r
            self.assertEqual([keep(*point) for point in raw], [False, True])
            self.assertEqual(node.last_query_self_filter['removed_points'], 1)
            config.write_text(json.dumps(dict(schema_version=1, frame='base_link', boxes=[])))
            # One request retains its exact original mask even if the file changes.
            self.assertFalse(keep(*raw[0]))
            self.assertEqual(node._query_self_filter_spec, self.spec)
            config.write_text(json.dumps(self.config))
            node.body_to_base_pose = lambda:inverse(pose(self.r, self.t + [0., 0., .2]))
            with self.assertRaisesRegex(ValueError, 'calibration mismatch'):
                prepare_query_self_filter(node)

    def test_enabled_bbs_rejects_wrong_body_translation_rotation_and_sc_configuration_passes(self):
        from agt_sc_kiss.runtime import configure_query
        with tempfile.TemporaryDirectory() as td:
            config = Path(td) / 'mask.json'
            config.write_text(json.dumps(self.config))
            active = str(Path(td) / 'active_lio.yaml')
            expected_t = [.03, -.08, .06]
            expected_q = Rotation.from_euler('xyz', [.1, -.2, .3]).as_quat().tolist()
            values = dict(query_self_filter_config=str(config), mount_base_frame='base_link',
                mount_lidar_frame='livox_frame', relocalization_query_frame_mode='mapping_body',
                body_to_base_calibration_file=active, batch_lio_config_file='unused-legacy.yaml',
                mapping_body_livox_translation=expected_t[:],
                mapping_body_livox_quaternion_xyzw=expected_q[:])
            def query_contract():
                return ('mapping_body','body',dict(zip(('x','y','z','qx','qy','qz','qw'),
                    [*values['mapping_body_livox_translation'], *values['mapping_body_livox_quaternion_xyzw']])))
            node = SimpleNamespace(get_parameter=lambda name:SimpleNamespace(value=values[name]),
                body_to_base_pose=lambda:inverse(pose(self.r,self.t)), inverse_pose=inverse,
                query_frame_contract=query_contract,
                clouds=[SimpleNamespace(header=SimpleNamespace(frame_id='livox_frame'))],
                set_parameters=lambda params:values.update({p.name:p.value for p in params}))
            with active_lio_fixture(expected_t,expected_q) as loader:
                self.assertIsNotNone(prepare_query_self_filter(node))
                loader.assert_called_once_with(active)
                values['mapping_body_livox_translation'] = [.13, -.08, .06]
                with self.assertRaisesRegex(ValueError,'mapping_body_livox calibration mismatch'):
                    prepare_query_self_filter(node)
                self.assertIsNone(node._query_self_filter_spec)
                values['mapping_body_livox_translation'] = expected_t[:]
                values['mapping_body_livox_quaternion_xyzw'] = [0.,0.,0.,1.]
                with self.assertRaisesRegex(ValueError,'mapping_body_livox calibration mismatch'):
                    prepare_query_self_filter(node)
                # Run the real SC configure helper with a minimal ROS Parameter fixture.
                parameter_module = ModuleType('rclpy.parameter')
                parameter_module.Parameter = lambda name,value:SimpleNamespace(name=name,value=value)
                with patch.dict(sys.modules, {'rclpy.parameter':parameter_module}):
                    configure_query(node)
                self.assertIsNotNone(prepare_query_self_filter(node))
                # Selected config wins; an empty selection uses the exact legacy fallback.
                values['body_to_base_calibration_file'] = ' '
                values['batch_lio_config_file'] = str(Path(td)/'legacy.yaml')
                loader.reset_mock()
                self.assertIsNotNone(prepare_query_self_filter(node))
                loader.assert_called_once_with(values['batch_lio_config_file'])
                values['batch_lio_config_file'] = ''
                with self.assertRaisesRegex(ValueError,'selected active LIO'):
                    prepare_query_self_filter(node)
                # The base_link compatibility path does not use mapping_body parameters.
                node.query_frame_contract = lambda:('base_link','base_link',None)
                loader.reset_mock()
                self.assertIsNotNone(prepare_query_self_filter(node))
                loader.assert_not_called()

    def test_preprocess_empty_is_only_allowed_for_database_construction(self):
        spec = contract([0., 0., 0.], [0., 0., 0., 1.])
        self.assertEqual(preprocess([], spec, allow_empty=True).shape, (0, 3))
        with self.assertRaisesRegex(ValueError, 'no observed points'):
            preprocess([], spec)


class SparseDatabaseTests(unittest.TestCase):
    def test_sparse_and_empty_keyframes_are_counted_without_losing_valid_frames(self):
        with tempfile.TemporaryDirectory() as td:
            root = Path(td)
            (root / 'patches').mkdir()
            clouds = {
                'empty.pcd':np.empty((0, 3)),
                'small.pcd':np.asarray([[2. + i*.3, 0., 0.] for i in range(5)]),
                'good.pcd':np.asarray([[2. + i*.3, 1. + j*.3, 0.] for i in range(12) for j in range(12)]),
            }
            for name in clouds:
                (root / 'patches' / name).write_text(name)
            (root / 'poses.txt').write_text(''.join(f'{name} 0 0 0 1 0 0 0\n' for name in clouds))
            (root / 'map.pcd').write_text('map identity fixture')
            (root / 'map.pgm').write_bytes(b'PGM identity fixture')
            (root / 'map.yaml').write_text('image: map.pgm\nresolution: 0.05\norigin: [0,0,0]\n')
            class Index:
                def descriptor(self, points):
                    return np.asarray([len(points), points[:, 0].mean()])
            native = SimpleNamespace(Index=Index, upstream_commit='test-fixture',
                                     load_pcd=lambda path:clouds[Path(path).name])
            with patch.dict(sys.modules, {'agt_sc_kiss._scancontext':native}), \
                    patch.object(agt_sc_kiss, '_scancontext', native, create=True):
                result = database.build(root / 'map.pcd', root / 'map.yaml', root,
                                        root / 'database', contract([0,0,0], [0,0,0,1]))
                self.assertEqual(result['keyframe_counts'], dict(read=3, indexed=1, skipped=2))
                self.assertEqual(result['entries'][0]['name'], 'good.pcd')
                self.assertEqual([x['processed_points'] for x in result['skipped_keyframes']], [0, 5])
                self.assertTrue(all(x['reason']=='TOO_FEW_POINTS_AFTER_PREPROCESSING'
                                    for x in result['skipped_keyframes']))
                (root / 'poses.txt').write_text('empty.pcd 0 0 0 1 0 0 0\nsmall.pcd 0 0 0 1 0 0 0\n')
                with self.assertRaisesRegex(ValueError, 'no usable keyframes; 2 skipped'):
                    database.build(root / 'map.pcd', root / 'map.yaml', root,
                                   root / 'empty_database', contract([0,0,0], [0,0,0,1]))
                self.assertFalse((root / 'empty_database').exists())


class PatchedQueryLoopTests(unittest.TestCase):
    """Runs the real dependency method, not a duplicate query implementation.

    Set AGT_TEST_RELOCALIZATION_SOURCE for an isolated prepared dependency, or
    use the normal sibling agt_navigation_v3 after applying reviewed patches.
    """
    @classmethod
    def setUpClass(cls):
        default = REPO.parent / 'agt_navigation_v3/navigation/localization/agt_global_relocalization/agt_global_relocalization/global_relocalization.py'
        source = Path(os.environ.get('AGT_TEST_RELOCALIZATION_SOURCE', str(default)))
        if not source.is_file():
            raise unittest.SkipTest('prepared V3 dependency is unavailable; set AGT_TEST_RELOCALIZATION_SOURCE')
        tree = ast.parse(source.read_text())
        target = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name=='GlobalRelocalization')
        method = next(node for node in target.body if isinstance(node, ast.FunctionDef) and node.name=='merged_points')
        if 'prepare_query_self_filter' not in ast.unparse(method):
            raise unittest.SkipTest('V3 query self-filter patch is not applied')
        class Time:
            @staticmethod
            def from_msg(stamp):return (stamp.sec, stamp.nanosec)
        namespace = dict(math=math, Duration=lambda **kwargs:kwargs, Time=Time,
                         TransformException=LookupError,
                         point_cloud2=SimpleNamespace(read_points=lambda cloud, **kwargs:cloud.points))
        exec(compile(ast.Module(body=[method], type_ignores=[]), str(source), 'exec'), namespace)
        cls.method = staticmethod(namespace['merged_points'])

    def test_raw_sensor_filter_before_mapping_transform_preserves_message_and_intensity(self):
        with tempfile.TemporaryDirectory() as td:
            # Deliberately different base<-body and base<-sensor rotations.
            rb = Rotation.from_euler('y', 25., degrees=True).as_matrix()
            tb = np.asarray([.2, 0., .6])
            rs = Rotation.from_euler('y', -15., degrees=True).as_matrix()
            ts = np.asarray([.5, 0., 1.])
            body_from_sensor_r = rb.T @ rs
            body_from_sensor_t = rb.T @ (ts - tb)
            config = Path(td) / 'mask.json'
            config.write_text(json.dumps(make_self_filter_config(
                [-.2, .2, -.1, .1, .7, .9], tb, Rotation.from_matrix(rb).as_quat())))
            base = np.asarray([[0., 0., .8], [3., 1., .8], [4., 2., 1.3]])
            raw = (base - ts) @ rs
            rows = [tuple([*point, 40. + i]) for i, point in enumerate(raw)]
            cloud = SimpleNamespace(header=SimpleNamespace(frame_id='livox_frame',
                stamp=SimpleNamespace(sec=123, nanosec=456)),
                fields=[SimpleNamespace(name=k) for k in ('x','y','z','intensity')], points=rows)
            before = copy.deepcopy(rows)
            params = dict(cloud_contract_capture_dir='capture', max_points=1000,
                query_min_range_m=0., query_max_range_m=30., query_voxel_leaf_m=.25,
                global_backend='bbs', tf_timeout_sec=.1, mount_base_frame='base_link',
                query_self_filter_config=str(config), body_to_base_calibration_file='active-lio.yaml')
            class TF:
                def lookup_transform(self, target, source, stamp, **kwargs):
                    if target!='base_link' or source!='livox_frame':raise LookupError('wrong frame')
                    if stamp!=(123,456):raise LookupError('wrong timestamp')
                    return SimpleNamespace(transform=transform(rs,ts))
            node = SimpleNamespace(clouds=[cloud], get_parameter=lambda key:SimpleNamespace(value=params[key]),
                query_frame_contract=lambda:('mapping_body','body',pose(body_from_sensor_r,body_from_sensor_t)),
                body_to_base_pose=lambda:inverse(pose(rb,tb)), inverse_pose=inverse, tf_buffer=TF(),
                _rotate_xyz=lambda x,y,z,q:tuple(Rotation.from_quat(q).apply([x,y,z])))
            with active_lio_fixture(body_from_sensor_t,Rotation.from_matrix(body_from_sensor_r).as_quat()):
                got = np.asarray(self.method(node))
            expected = (base[1:] - tb) @ rb
            np.testing.assert_allclose(got[:, :3], expected, atol=1e-12)
            np.testing.assert_array_equal(got[:, 3], [41., 42.])
            self.assertEqual(cloud.points, before)
            self.assertEqual(node.last_query_self_filter['removed_points'], 1)
            self.assertEqual(node.last_query_self_filter['input_points'], 3)
            self.assertEqual(len(node.last_query_cloud_contract['raw_rows']), 3)
            # An enabled mask fails the request if its TF is missing.
            node.tf_buffer = SimpleNamespace(lookup_transform=lambda *a,**kw:(_ for _ in ()).throw(LookupError('missing')))
            with active_lio_fixture(body_from_sensor_t,Rotation.from_matrix(body_from_sensor_r).as_quat()):
                with self.assertRaisesRegex(RuntimeError, 'query self-filter TF unavailable'):
                    self.method(node)
            # Disabled behavior requires neither mask calibration nor extra TF.
            params['query_self_filter_config']=''
            unfiltered = np.asarray(self.method(node))
            self.assertEqual(len(unfiltered), 3)
            self.assertIsNone(node._query_self_filter_spec)


if __name__ == '__main__':
    unittest.main()
