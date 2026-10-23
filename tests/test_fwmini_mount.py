import copy
import json
import math
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
from http.server import ThreadingHTTPServer
from http.client import HTTPConnection
import threading
import shutil
import xml.etree.ElementTree as ET

import yaml
from workbench.fwmini import candidate_path, load_profile, require_mode, verify_profile, digest, robot_description
from agt_field_test_bringup.fwmini import edges, lio_configs, validate

REPO = Path(__file__).resolve().parents[1]
CANDIDATE = REPO/'agt_field_test_bringup/config/fwmini_mid360_candidate.yaml'


class FWminiMountTests(unittest.TestCase):
    def setUp(self):
        self.profile = load_profile(CANDIDATE)

    def test_original_evidence_numbers_and_reference_height_preserved(self):
        source = yaml.safe_load((REPO/'docs/evidence/fwmini_20261022/fwmini_mid360_extrinsic.yaml').read_text())
        self.assertEqual(self.profile['mount']['translation_m'], source['translation_m'])
        self.assertEqual(self.profile['mount']['rotation_rpy_rad'], source['rotation_rpy_rad'])
        self.assertAlmostEqual(self.profile['mount']['translation_m'][2] + self.profile['base_footprint_to_base_z_m'], .44704)
        self.assertEqual(self.profile['status'], 'CALIBRATION_CANDIDATE')
        self.assertFalse(self.profile['source']['original_profile_available_in_cloud'])

    def test_direction_and_nonfinite_values_rejected(self):
        for key, value in [('parent_frame', 'livox_frame'), ('translation_m', [0, 0, float('nan')])]:
            p = copy.deepcopy(self.profile)
            p['mount'][key] = value
            with self.assertRaises(ValueError):
                validate(p)

    def test_candidate_modes_do_not_imply_autonomous_motion(self):
        for mode in ('sensor_test', 'calibration_recording', 'offline_mapping', 'sensor_mapping'):
            require_mode(self.profile, mode)
        with self.assertRaises(ValueError):
            require_mode(self.profile, 'supervised_localization')
        require_mode(self.profile, 'supervised_localization', supervised=True)
        with self.assertRaises(ValueError):
            require_mode(self.profile, 'autonomous_navigation', supervised=True)

    def test_urdf_has_one_mount_and_inverts_internal_imu_transform(self):
        root = ET.fromstring(robot_description(self.profile))
        joints = root.findall('joint')
        children = [j.find('child').attrib['link'] for j in joints]
        self.assertEqual(set(children), {'base_link', 'lidar_link', 'livox_frame', 'imu_link'})
        self.assertEqual(len(children), len(set(children)))
        tree = edges(self.profile)
        self.assertEqual(tree['livox_frame'], ('lidar_link', [0, 0, 0], [0, 0, 0]))
        self.assertEqual(tree['imu_link'][1], [.011, .02329, -.04412])
        self.assertLess(-math.sin(tree['lidar_link'][2][1]), 0)  # +pitch sends sensor x down.

    def test_report_binding_and_immutable_upgrade(self):
        with tempfile.TemporaryDirectory() as folder:
            folder = Path(folder)
            report = folder/'report.json'
            target = folder/'verified.yaml'
            data = {'robot': 'fwmini', 'verification_scope': 'PHYSICAL_MOUNT', 'sensor_id': 'MID360-test', 'calibration_sha256': digest(self.profile),
                    'checks': {k: 'PASS' for k in ('mount_direction', 'rotation_center', 'repeatability', 'geometry', 'tf_ownership')},
                    'operator': 'test fixture', 'observed_at': 'test fixture', 'observations': {'fixture': True}}
            report.write_text(json.dumps(data))
            with self.assertRaises(ValueError):
                verify_profile(CANDIDATE, report, 'wrong-sensor', target)
            verify_profile(CANDIDATE, report, 'MID360-test', target)
            verified = load_profile(target)
            self.assertEqual(verified['status'], 'VERIFIED')
            self.assertEqual(load_profile(CANDIDATE)['status'], 'CALIBRATION_CANDIDATE')
            with self.assertRaises(ValueError):
                require_mode(verified, 'autonomous_navigation')
            verified['mount']['translation_m'][0] += .1
            target.write_text(yaml.safe_dump(verified))
            with self.assertRaises(ValueError):
                load_profile(target)

    def test_backend_internal_parameters_are_not_chassis_mount(self):
        runtime = REPO.parent/'agt_navigation_v3/navigation/nav2/agt_navigation_runtime'
        with tempfile.TemporaryDirectory() as folder:
            fast, batch = lio_configs(self.profile, runtime, folder)
            f = yaml.safe_load(Path(fast).read_text())
            b = yaml.safe_load(Path(batch).read_text())['/**']['ros__parameters']['mapping']
            self.assertEqual(f['t_il'], b['extrinsic_T'])
            self.assertEqual(f['r_il'], b['extrinsic_R'])
            self.assertNotEqual(f['t_il'], self.profile['mount']['translation_m'])

    def test_calibration_http_displays_candidate_and_requires_physical_report(self):
        from workbench import server
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            target = root/'calibration/fwmini/fwmini_mid360_candidate.yaml'
            target.parent.mkdir(parents=True)
            shutil.copyfile(CANDIDATE, target)
            with patch.object(server, 'ROOT', root):
                http = ThreadingHTTPServer(('127.0.0.1', 0), server.Handler)
                worker = threading.Thread(target=http.serve_forever, daemon=True)
                worker.start()
                try:
                    client = HTTPConnection('127.0.0.1', http.server_port)
                    client.request('GET', '/api/fwmini/mounts')
                    response = client.getresponse()
                    data = json.loads(response.read())
                    self.assertEqual(response.status, 200)
                    self.assertEqual(data[0]['mount']['translation_m'], [.146725, .002608, .37704])
                    self.assertEqual(data[0]['status'], 'CALIBRATION_CANDIDATE')
                    report = root/'reports/field.json'
                    report.parent.mkdir()
                    payload = {'profile': str(target), 'report': 'reports/field.json', 'sensor_id': 'fixture'}
                    report.write_text(json.dumps({'status': 'SOFTWARE_TF_PASS'}))
                    client.request('POST', '/api/fwmini/mounts/verify', json.dumps(payload), {'Content-Type': 'application/json'})
                    response = client.getresponse(); response.read()
                    self.assertEqual(response.status, 400)
                    report.write_text(json.dumps({'robot': 'fwmini', 'sensor_id': 'fixture', 'verification_scope': 'PHYSICAL_MOUNT',
                        'calibration_sha256': digest(self.profile), 'operator': 'fixture', 'observed_at': 'fixture',
                        'observations': {'fixture': True}, 'checks': {k: 'PASS' for k in
                        ('mount_direction', 'rotation_center', 'repeatability', 'geometry', 'tf_ownership')}}))
                    client.request('POST', '/api/fwmini/mounts/verify', json.dumps(payload), {'Content-Type': 'application/json'})
                    response = client.getresponse(); result = json.loads(response.read())
                    self.assertEqual(response.status, 200, result)
                    self.assertEqual(load_profile(result['path'])['status'], 'VERIFIED')
                    self.assertEqual(load_profile(target)['status'], 'CALIBRATION_CANDIDATE')
                    client.close()
                finally:
                    http.shutdown(); http.server_close(); worker.join()


if __name__ == '__main__':
    unittest.main()
