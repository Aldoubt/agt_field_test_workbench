"""Immutable query-filter contracts using real files and no ROS execution."""
import copy
import hashlib
import json
from pathlib import Path
import tempfile
import unittest

import yaml

from workbench import field_trial, session
from workbench.maps import Maps, hash_file
from workbench.query_filters import (
    describe_query_filter, freeze_query_filter, validate_bbs_query_filter,
    validate_sc_query_filter,
)


def write_filter(root, name='self_filter.json', xmax=.5):
    path = root / name
    path.write_text(json.dumps(dict(
        schema_version=1, frame='base_link',
        boxes=[-.5, xmax, -.5, .5, -.5, .5],
        base_from_body=dict(translation=[0, 0, 0], quaternion_xyzw=[0, 0, 0, 1]),
    ), indent=2) + '\n')
    return path


def write_json(path, value):
    path.write_text(json.dumps(value, sort_keys=True, allow_nan=False) + '\n')


def field_fixture(root, masked=False):
    """Build tiny, correctly hashed map/SC/BBS/config records for real preflight."""
    mask = write_filter(root)
    query_filter = describe_query_filter(mask if masked else '')
    cloud = root / 'maps/imported/map.pcd'
    cloud.parent.mkdir(parents=True)
    cloud.write_text(
        'VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n'
        'WIDTH 1\nHEIGHT 1\nPOINTS 1\nDATA ascii\n2 0 0\n')
    selected = Maps(root)._save(
        2, 2, bytes([254] * 4),
        dict(resolution=.1, origin=[0., 0., 0.], negate=0,
             occupied_thresh=.65, free_thresh=.196),
        dict(source_pcd=str(cloud), source_pcd_sha256=hash_file(cloud),
             localization_geometry_valid=True, **query_filter), status='ACCEPTED')

    assets = cloud.parent / 'relocalization'
    (assets / 'voxelmaps_coords').mkdir(parents=True)
    (assets / 'global_map_downsampled.pcd').write_bytes(cloud.read_bytes())
    (assets / 'polar_context.db').write_bytes(b'fixture Polar descriptor database')
    status = dict(state='READY', source_sha256=hash_file(cloud),
                  bbs='READY', polar_context='READY')
    if masked:
        status.update(query_self_filter=query_filter['query_self_filter'],
                      query_self_filter_sha256=query_filter['query_self_filter_sha256'],
                      keyframes_sha256='fixture-keyframes-content-identity',
                      polar_context_db_sha256=hash_file(assets / 'polar_context.db'))
    write_json(assets / 'asset_status.json', status)

    database = assets / 'scancontext_db_fixture'
    database.mkdir()
    (database / 'descriptors.npz').write_bytes(b'fixture Scan Context descriptors')
    manifest = dict(
        algorithm='scancontext', map_sha256=hash_file(cloud),
        navigation_yaml=selected['yaml'], navigation_yaml_sha256=hash_file(selected['yaml']),
        navigation_pgm_sha256=hash_file(selected['pgm']),
        descriptors_sha256=hash_file(database / 'descriptors.npz'))
    if masked:
        manifest['preprocessing'] = {'self_filter': query_filter['query_self_filter']}
    manifest['metadata_sha256'] = hashlib.sha256(json.dumps(
        manifest, sort_keys=True, allow_nan=False).encode()).hexdigest()
    write_json(database / 'manifest.json', manifest)

    identity = [1., 0., 0., 0., 1., 0., 0., 0., 1.]
    bundle = dict(
        kind='robot_calibration_bundle', robot='bunker', status='CANDIDATE',
        sensor_id='offline-fixture',
        mount=dict(parent='base_link', child='livox_frame', xyz=[0., 0., 0.], rpy=[0., 0., 0.]),
        internal={backend: dict(rotation=identity, translation=[0., 0., 0.])
                  for backend in ('fastlio2', 'batch_lio')},
        time_sync=dict(imu_offset_sec=0., lidar_offset_sec=0.), rtk_arm=dict(enabled=False))
    (root / 'calibration.yaml').write_text(yaml.safe_dump(bundle))
    profile = dict(robot='bunker', forward_mps=.2, reverse_mps=.1,
                   angular_radps=.3, linear_accel_mps2=.1, linear_decel_mps2=.2, control_hz=50)
    (root / 'robot_profile.yaml').write_text(yaml.safe_dump(profile))
    write_json(root / 'livox.json', dict(
        Mid360s=dict(host_net_info=[dict(host_ip='192.168.1.5')]),
        lidar_configs=[dict(ip='192.168.1.164')]))
    config = root / 'field_trial.yaml'
    config.write_text(yaml.safe_dump(dict(
        map_id=selected['id'], calibration_bundle='calibration.yaml',
        robot_profile='robot_profile.yaml', livox_config='livox.json',
        sc_database=str(database), global_backend='sc_kiss')))
    return dict(config=config, selected=selected, mask=mask, assets=assets,
                database=database, query_filter=query_filter)


class QueryFilterWorkflowTests(unittest.TestCase):
    def test_freeze_preserves_exact_content_and_requested_identity(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            source = write_filter(root)
            requested = describe_query_filter(source.name, root)
            before = source.read_bytes()
            output = root / 'snapshot'
            output.mkdir()
            frozen = freeze_query_filter(source.name, output, root,
                                         expected_sha256=requested['query_self_filter_sha256'])
            self.assertEqual(frozen['query_self_filter_config'], str(output / 'query_self_filter.json'))
            self.assertEqual(frozen['query_self_filter_source'], str(source))
            self.assertEqual(frozen['query_self_filter'], requested['query_self_filter'])
            self.assertEqual(frozen['query_self_filter_sha256'], hash_file(source))
            self.assertEqual(Path(frozen['query_self_filter_config']).read_bytes(), before)
            self.assertEqual(source.read_bytes(), before)

    def test_freeze_rejects_change_or_disable_after_request_before_publishing(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            source = write_filter(root)
            requested = describe_query_filter(source)
            write_filter(root, xmax=2.)
            for name, value in (('changed', source), ('disabled', '')):
                with self.subTest(name=name):
                    output = root / name
                    output.mkdir()
                    with self.assertRaisesRegex(ValueError, 'changed'):
                        freeze_query_filter(value, output,
                                            expected_sha256=requested['query_self_filter_sha256'])
                    self.assertEqual(list(output.iterdir()), [])

    def test_freeze_cannot_overwrite_existing_snapshot(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            source = write_filter(root)
            output = root / 'snapshot'
            output.mkdir()
            target = output / 'query_self_filter.json'
            target.write_bytes(b'existing immutable configuration')
            with self.assertRaises(FileExistsError):
                freeze_query_filter(source, output)
            self.assertEqual(target.read_bytes(), b'existing immutable configuration')

    def test_scan_context_contract_rejects_both_mismatch_directions(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            records = [describe_query_filter(''),
                       describe_query_filter(write_filter(root, 'a.json')),
                       describe_query_filter(write_filter(root, 'b.json', xmax=2.))]
            for actual_index, actual in enumerate(records):
                database = root / str(actual_index)
                database.mkdir()
                # The absent preprocessing block models a legacy unfiltered DB.
                manifest = ({} if actual_index == 0 else
                            {'preprocessing': {'self_filter': actual['query_self_filter']}})
                write_json(database / 'manifest.json', manifest)
                for expected_index, expected in enumerate(records):
                    with self.subTest(actual=actual_index, requested=expected_index):
                        if actual_index == expected_index:
                            self.assertEqual(validate_sc_query_filter(database, expected), str(database))
                        else:
                            with self.assertRaisesRegex(ValueError, 'self-filter differs'):
                                validate_sc_query_filter(database, expected)

    def test_bbs_contract_is_bidirectional_and_legacy_empty_mask_still_works(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            records = [describe_query_filter(''),
                       describe_query_filter(write_filter(root, 'a.json')),
                       describe_query_filter(write_filter(root, 'b.json', xmax=2.))]
            for actual_index, actual in enumerate(records):
                assets = root / str(actual_index)
                assets.mkdir()
                # Older BBS exports have no query preprocessing status at all.
                if actual_index:
                    write_json(assets / 'asset_status.json', dict(
                        query_self_filter=actual['query_self_filter'],
                        query_self_filter_sha256=actual['query_self_filter_sha256']))
                for expected_index, expected in enumerate(records):
                    with self.subTest(actual=actual_index, requested=expected_index):
                        if actual_index == expected_index:
                            self.assertEqual(validate_bbs_query_filter(assets, expected), str(assets))
                        else:
                            with self.assertRaisesRegex(ValueError, 'self-filter differs'):
                                validate_bbs_query_filter(assets, expected)
            incomplete = root / 'incomplete'
            incomplete.mkdir()
            write_json(incomplete / 'asset_status.json', {'query_self_filter_sha256': 'unresolved'})
            with self.assertRaisesRegex(ValueError, 'incomplete'):
                validate_bbs_query_filter(incomplete, records[0])

    def test_session_reuse_handles_legacy_and_enable_disable_transitions(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            source = write_filter(root)
            requested = describe_query_filter(source)
            disabled = describe_query_filter('')
            output = root / 'snapshot'
            output.mkdir()
            current = freeze_query_filter(source, output)
            self.assertTrue(session.query_filter_matches({}, disabled))
            self.assertTrue(session.query_filter_matches(current, requested))
            self.assertFalse(session.query_filter_matches({}, requested))
            self.assertFalse(session.query_filter_matches(current, disabled))
            changed = describe_query_filter(write_filter(root, 'changed.json', xmax=2.))
            self.assertFalse(session.query_filter_matches(current, changed))

    def test_session_reuse_rechecks_both_frozen_files_for_change_or_loss(self):
        for side in ('current', 'requested'):
            for fault in ('changed', 'missing'):
                with self.subTest(side=side, fault=fault), tempfile.TemporaryDirectory() as folder:
                    root = Path(folder)
                    source = write_filter(root)
                    records = {}
                    for label in ('current', 'requested'):
                        output = root / label
                        output.mkdir()
                        records[label] = freeze_query_filter(source, output)
                    self.assertTrue(session.query_filter_matches(records['current'], records['requested']))
                    frozen = Path(records[side]['query_self_filter_config'])
                    if fault == 'changed':
                        frozen.write_bytes(frozen.read_bytes() + b'\n')
                    else:
                        frozen.unlink()
                    self.assertFalse(session.query_filter_matches(records['current'], records['requested']))

    def test_make_plan_and_snapshot_propagate_frozen_mask_path_hash_and_spec(self):
        for backend in ('sc_kiss', 'bbs'):
            with self.subTest(backend=backend), tempfile.TemporaryDirectory() as folder:
                root = Path(folder)
                fixture = field_fixture(root, masked=True)
                plan = field_trial.make_plan(root, fixture['config'], global_backend=backend)
                original_plan = copy.deepcopy(plan)
                source = fixture['mask']
                before = source.read_bytes()
                self.assertEqual(plan['query_self_filter_config'], str(source))
                self.assertEqual(plan['query_self_filter_sha256'], hash_file(source))
                self.assertEqual(plan['query_self_filter'], fixture['query_filter']['query_self_filter'])
                self.assertEqual(plan['inputs']['query_self_filter_config'], str(source))
                directory, frozen = field_trial.snapshot_plan(root, plan)
                target = directory / 'query_self_filter_config.json'
                self.assertEqual(target.read_bytes(), before)
                self.assertEqual(frozen['query_self_filter_config'], str(target))
                self.assertEqual(frozen['query_self_filter_source'], str(source))
                self.assertEqual(frozen['query_self_filter_sha256'], hash_file(target))
                self.assertEqual(frozen['query_self_filter'], plan['query_self_filter'])
                args = dict(item.split(':=', 1) for item in frozen['launch_args'])
                self.assertEqual(args['query_self_filter_config'], str(target))
                self.assertEqual(args['navigation_autostart'], 'false')
                self.assertEqual(args['supervised'], 'true')
                self.assertEqual(json.loads((directory / 'session.json').read_text()), frozen)
                self.assertEqual(plan, original_plan)
                self.assertEqual(source.read_bytes(), before)

    def test_make_plan_default_off_preserves_legacy_backends(self):
        for backend in ('sc_kiss', 'bbs'):
            with self.subTest(backend=backend), tempfile.TemporaryDirectory() as folder:
                root = Path(folder)
                fixture = field_fixture(root)
                plan = field_trial.make_plan(root, fixture['config'], global_backend=backend)
                self.assertEqual(plan['query_self_filter_config'], '')
                self.assertEqual(plan['query_self_filter_sha256'], '')
                self.assertIsNone(plan['query_self_filter'])
                self.assertNotIn('query_self_filter_config', plan['inputs'])
                self.assertFalse(any(arg.startswith('query_self_filter_config:=') for arg in plan['launch_args']))

    def test_make_plan_checks_both_backend_contracts_before_launch(self):
        for backend in ('sc_kiss', 'bbs'):
            for case in ('enable_on_old_database', 'disable_on_masked_database', 'change_mask'):
                with self.subTest(backend=backend, case=case), tempfile.TemporaryDirectory() as folder:
                    root = Path(folder)
                    fixture = field_fixture(root, masked=case != 'enable_on_old_database')
                    requested = (str(fixture['mask']) if case == 'enable_on_old_database' else
                                 '' if case == 'disable_on_masked_database' else
                                 str(write_filter(root, 'different.json', xmax=2.)))
                    with self.assertRaisesRegex(ValueError, 'self-filter differs'):
                        field_trial.make_plan(root, fixture['config'], global_backend=backend,
                                              query_self_filter_config=requested)
                    self.assertFalse((root / 'runs').exists())

    def test_explicit_plan_override_takes_precedence_over_config_and_map_defaults(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            fixture = field_fixture(root, masked=True)
            different = write_filter(root, 'different.json', xmax=2.)
            config = yaml.safe_load(fixture['config'].read_text())
            config['query_self_filter_config'] = different.name
            fixture['config'].write_text(yaml.safe_dump(config))
            with self.assertRaisesRegex(ValueError, 'self-filter differs'):
                field_trial.make_plan(root, fixture['config'])
            plan = field_trial.make_plan(root, fixture['config'],
                                         query_self_filter_config=fixture['mask'].name)
            self.assertEqual(plan['query_self_filter_sha256'], hash_file(fixture['mask']))

    def test_snapshot_rejects_mask_changed_after_preflight(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            fixture = field_fixture(root, masked=True)
            plan = field_trial.make_plan(root, fixture['config'])
            original_plan = copy.deepcopy(plan)
            write_filter(root, xmax=2.)
            with self.assertRaisesRegex(ValueError, 'configuration changed during startup'):
                field_trial.snapshot_plan(root, plan)
            self.assertFalse(list((root / 'runs').glob('*/session.json')))
            self.assertEqual(plan, original_plan)


if __name__ == '__main__':
    unittest.main()
