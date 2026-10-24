import json
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'agt_field_test_bringup'))
from agt_field_test_bringup.livox import validate_mid360s_config


CONFIG = Path(__file__).resolve().parents[1] / 'agt_field_test_bringup/config/MID360s_config.json'


class LivoxConfigTests(unittest.TestCase):
    def test_dedicated_mid360s_identity_is_explicit(self):
        identity = validate_mid360s_config(CONFIG)
        self.assertEqual(identity['model'], 'MID360S')
        self.assertEqual(identity['sensor_ip'], '192.168.1.164')
        self.assertEqual(identity['host_ip'], '192.168.1.5')
        self.assertEqual(identity['serial_number'], 'NOT_QUERIED')
        self.assertEqual(identity['connection_state'], 'CONFIG_VALIDATED_NOT_CONNECTED')
        self.assertEqual(len(identity['config_sha256']), 64)

    def test_missing_or_legacy_config_never_falls_back(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            with self.assertRaisesRegex(ValueError, 'legacy MID360 fallback is disabled'):
                validate_mid360s_config(root / 'missing.json')
            legacy = root / 'MID360_config.json'
            legacy.write_text(json.dumps({'MID360': {}, 'lidar_configs': []}))
            with self.assertRaisesRegex(ValueError, 'Mid360s block'):
                validate_mid360s_config(legacy)

    def test_wrong_ip_is_rejected_before_startup(self):
        with tempfile.TemporaryDirectory() as temp:
            target = Path(temp) / 'bad.json'
            data = json.loads(CONFIG.read_text())
            data['lidar_configs'][0]['ip'] = '192.168.1.12'
            target.write_text(json.dumps(data))
            with self.assertRaisesRegex(ValueError, '192.168.1.164'):
                validate_mid360s_config(target)
            data = json.loads(CONFIG.read_text())
            data['Mid360s']['host_net_info'][0]['host_ip'] = '192.168.1.6'
            target.write_text(json.dumps(data))
            with self.assertRaisesRegex(ValueError, '192.168.1.5'):
                validate_mid360s_config(target)

    def test_field_launch_has_no_legacy_mid360_fallback(self):
        source = (Path(__file__).resolve().parents[1] /
                  'agt_field_test_bringup/launch/field_test.launch.py').read_text()
        self.assertNotIn('msg_MID360_launch.py', source)
        self.assertIn('validate_mid360s_config', source)
        self.assertIn('MID360s_config.json', source)


if __name__ == '__main__':
    unittest.main()
