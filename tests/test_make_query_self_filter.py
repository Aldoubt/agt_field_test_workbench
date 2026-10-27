"""Candidate generation must preserve current configuration and frame meaning."""
import copy
import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import numpy as np
from scipy.spatial.transform import Rotation
import yaml

REPO = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('make_query_self_filter', REPO/'scripts/make_query_self_filter.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
from agt_field_test_bringup.relocalization_self_filter import load_self_filter


class MakeQuerySelfFilterTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.internal_rotation = Rotation.from_euler('xyz', [4., -6., 13.], degrees=True).as_matrix()
        self.bundle = dict(kind='robot_calibration_bundle', schema_version=2, robot='bunker',
            status='CANDIDATE', base_link_height_m=3.7,
            mount=dict(parent='base_link', child='livox_frame', xyz=[.4, -.1, .75], rpy=[.1, .25, -.08]),
            internal=dict(fastlio2=dict(rotation=self.internal_rotation.reshape(-1).tolist(),
                                       translation=[-.01, -.02, .04])))
        self.profile = dict(robot='bunker', body_frame='base_link',
            navigation_self_mask_boxes=[-.36, .02, -.17, .19, .72, 1.68])
        self.config = dict(robot_profile='config/robots/bunker.yaml',
                           calibration_bundle='calibration/bunker/selected.yaml')
        self._write_yaml('config/field_trial.yaml', self.config)
        self._write_yaml('config/robots/bunker.yaml', self.profile)
        self._write_yaml('calibration/bunker/selected.yaml', self.bundle)

    def _write_yaml(self, path, value):
        dest = self.root / path
        dest.parent.mkdir(parents=True, exist_ok=True)
        dest.write_text(yaml.safe_dump(value, sort_keys=False))

    def test_rotation_translation_formula_and_height_exclusion(self):
        rotation, translation = module.derive_base_from_body(self.bundle)
        rm = Rotation.from_euler('xyz', self.bundle['mount']['rpy']).as_matrix()
        expected_r = rm @ self.internal_rotation.T
        expected_t = np.asarray(self.bundle['mount']['xyz']) - expected_r @ [-.01, -.02, .04]
        np.testing.assert_allclose(rotation, expected_r, atol=1e-12)
        np.testing.assert_allclose(translation, expected_t, atol=1e-12)
        changed = copy.deepcopy(self.bundle)
        changed['base_link_height_m'] = 100.
        r2, t2 = module.derive_base_from_body(changed)
        np.testing.assert_array_equal(r2, rotation)
        np.testing.assert_array_equal(t2, translation)

    def test_generate_navigation_candidate_records_sources_and_preserves_inputs(self):
        before = {str(p):p.read_bytes() for p in self.root.rglob('*.yaml')}
        result = module.generate(self.root, None, 'work/query_mask.json', use_navigation_boxes=True)
        dest = Path(result['output'])
        value = json.loads(dest.read_text())
        self.assertFalse(result['enabled'])
        self.assertEqual(value['boxes'], self.profile['navigation_self_mask_boxes'])
        self.assertFalse(value['provenance']['automatic_activation'])
        self.assertEqual(value['provenance']['lio_backend'], 'fastlio2')
        self.assertEqual(value['provenance']['calibration_bundle']['sha256'],
            hashlib.sha256((self.root/self.config['calibration_bundle']).read_bytes()).hexdigest())
        r, t = module.derive_base_from_body(self.bundle)
        mask = load_self_filter(dest)
        np.testing.assert_allclose(mask['base_from_body_rotation'], r, atol=1e-12)
        np.testing.assert_allclose(mask['base_from_body_translation'], t, atol=1e-12)
        self.assertEqual(before, {str(p):p.read_bytes() for p in self.root.rglob('*.yaml')})

    def test_explicit_boxes_list_and_object_and_alternate_config(self):
        self._write_yaml('configs/other.yaml', self.config)
        for index, data in enumerate(([0., 1., 0., 1., 0., 1.], {'boxes':[0., 1., 0., 1., 0., 1.]})):
            box = self.root / f'boxes_{index}.json'
            box.write_text(json.dumps(data))
            result = module.generate(self.root, 'configs/other.yaml', f'candidate_{index}.json', boxes_file=box.name)
            self.assertEqual(result['boxes'], 1)
            self.assertEqual(json.loads(Path(result['output']).read_text())['provenance']['boxes_source']['kind'],
                             'explicit_boxes_file')

    def test_empty_and_invalid_boxes_are_rejected_without_output(self):
        for index, boxes in enumerate(([], [0,0,0,1,0,1], [0,1,0,1,0], [False,1,0,1,0,1],
                                       [0,1,0,1,0,float('nan')])):
            path = self.root / 'boxes.json'
            path.write_text(json.dumps({'boxes':boxes}))
            output = self.root / f'invalid_{index}.json'
            with self.subTest(boxes=boxes), self.assertRaises(ValueError):
                module.generate(self.root, None, output, boxes_file=path)
            self.assertFalse(output.exists())
        self.profile['navigation_self_mask_boxes']=[]
        self._write_yaml(self.config['robot_profile'], self.profile)
        with self.assertRaisesRegex(ValueError, 'nonempty'):
            module.generate(self.root, None, 'empty.json', use_navigation_boxes=True)

    def test_existing_file_and_dangling_symlink_are_never_overwritten(self):
        existing = self.root / 'keep.json'
        existing.write_bytes(b'operator data')
        with self.assertRaises(FileExistsError):
            module.generate(self.root, None, existing, use_navigation_boxes=True)
        self.assertEqual(existing.read_bytes(), b'operator data')
        link = self.root / 'dangling.json'
        target = self.root / 'must_not_create.json'
        link.symlink_to(target)
        with self.assertRaises(FileExistsError):
            module.generate(self.root, None, link, use_navigation_boxes=True)
        self.assertTrue(link.is_symlink())
        self.assertFalse(target.exists())

    def test_bad_mount_internal_rotation_and_robot_mismatch_are_rejected(self):
        bad = copy.deepcopy(self.bundle)
        bad['mount']['parent']='base_footprint'
        with self.assertRaisesRegex(ValueError, 'base_link'):
            module.derive_base_from_body(bad)
        bad = copy.deepcopy(self.bundle)
        bad['internal']['fastlio2']['rotation']=[2.,0.,0.,0.,1.,0.,0.,0.,1.]
        with self.assertRaisesRegex(ValueError, 'orthonormal'):
            module.derive_base_from_body(bad)
        self.profile['robot']='fwmini'
        self._write_yaml(self.config['robot_profile'], self.profile)
        with self.assertRaisesRegex(ValueError, 'identities differ'):
            module.generate(self.root, None, 'bad.json', use_navigation_boxes=True)

    def test_cli_requires_explicit_box_source_and_creates_only_new_output(self):
        command = [sys.executable, str(REPO/'scripts/make_query_self_filter.py'),
                   '--workspace', str(self.root), '--output', 'work/from_cli.json']
        missing = subprocess.run(command, capture_output=True, text=True)
        self.assertNotEqual(missing.returncode, 0)
        good = subprocess.run([*command, '--use-navigation-boxes'], capture_output=True, text=True)
        self.assertEqual(good.returncode, 0, good.stderr)
        self.assertFalse(json.loads(good.stdout)['enabled'])
        duplicate = subprocess.run([*command, '--use-navigation-boxes'], capture_output=True, text=True)
        self.assertEqual(duplicate.returncode, 2)
        self.assertIn('output exists', duplicate.stderr)


if __name__ == '__main__':
    unittest.main()
