"""Check fresh deployment, idempotence, and preservation of local edits."""
import json
from pathlib import Path
import subprocess
import tempfile
import unittest

REPO = Path(__file__).resolve().parents[1]


class ControlPatchTests(unittest.TestCase):
    def test_clean_pinned_checkouts_apply_twice_and_preserve_local_edits(self):
        manifest = json.loads((REPO/'patches/control_manifest.json').read_text())
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for relative, spec in manifest.items():
                source = REPO.parent/relative
                destination = root/'src'/relative
                subprocess.run(['git', 'clone', '-q', '--shared', '--no-checkout', str(source), str(destination)], check=True)
                subprocess.run(['git', '-C', str(destination), 'checkout', '-q', '--detach', spec['commit']], check=True)
            command = ['python3', str(REPO/'scripts/apply_control_integration.py'), '--workspace', str(root)]
            subprocess.run(command, check=True, capture_output=True)
            subprocess.run(command, check=True, capture_output=True)
            relative = 'fwmini_navigation_lab'
            name = 'src/fwmini_chassis_adapter/config/cmd_adapter.yaml'
            path = root/'src'/relative/name
            path.write_text(path.read_text()+'\n# Existing field operator edit\n')
            original = path.read_bytes()
            result = subprocess.run(command, capture_output=True)
            self.assertNotEqual(result.returncode, 0)
            self.assertIn(b'preserved', result.stderr)
            self.assertEqual(path.read_bytes(), original)

    def test_vendor_ctrl_cmd_encoder_alive_counter_and_bcc_are_unchanged(self):
        manifest = json.loads((REPO/'patches/control_manifest.json').read_text())
        spec = manifest['fwmini_navigation_lab']
        relative = 'vendor/FW-mini-ros2/yhs_can_control/src/yhs_can_control_node.cpp'
        root = REPO.parent/'fwmini_navigation_lab'
        before = subprocess.check_output(['git', '-C', str(root), 'show', spec['commit']+':'+relative], text=True)
        after = (root/relative).read_text()
        def encoder(text):
            start = text.index('  void CanControl::ctrl_cmd_callback(')
            end = text.index('  void CanControl::steering_ctrl_cmd_callback(', start)
            return text[start:end]
        self.assertEqual(encoder(before), encoder(after))
