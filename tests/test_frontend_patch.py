import json
from pathlib import Path
import subprocess
import shutil
import tempfile
import unittest

REPO=Path(__file__).resolve().parents[1]


class FrontendPatchTests(unittest.TestCase):
    def test_fresh_pinned_source_idempotence_and_preservation(self):
        manifest=REPO/'patches/frontend_manifest.json'
        spec=json.loads(manifest.read_text())['external/fast_lio2_mapping']
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);destination=root/'src/external/fast_lio2_mapping'
            source=REPO.parent/'external/fast_lio2_mapping'
            subprocess.run(['git','clone','-q','--shared','--no-checkout',str(source),str(destination)],check=True)
            subprocess.run(['git','-C',str(destination),'checkout','-q','--detach',spec['commit']],check=True)
            cmd=['python3',str(REPO/'scripts/apply_control_integration.py'),'--workspace',str(root),'--manifest',str(manifest)]
            subprocess.run(cmd,check=True,capture_output=True)
            subprocess.run(cmd,check=True,capture_output=True)
            header=destination/'fastlio2/src/map_builder/point_plane_jacobian.h'
            self.assertTrue(header.exists())
            header.write_text(header.read_text()+'\n// Operator edit\n')
            before=header.read_bytes()
            result=subprocess.run(cmd,capture_output=True)
            self.assertNotEqual(result.returncode,0)
            self.assertIn(b'preserved',result.stderr)
            self.assertEqual(header.read_bytes(),before)


    def test_initialization_fetch_preserves_frontend_and_fresh_apply_is_exact(self):
        manifest = REPO/'patches/frontend_manifest.json'
        spec = json.loads(manifest.read_text())['external/fast_lio2_mapping']
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            destination = root/'src/external/fast_lio2_mapping'
            subprocess.run(['git', 'clone', '-q', '--shared', '--no-checkout',
                            str(REPO.parent/'external/fast_lio2_mapping'), str(destination)], check=True)
            subprocess.run(['git', '-C', str(destination), 'checkout', '-q', '--detach', spec['commit']], check=True)
            command = ['python3', str(REPO/'scripts/apply_control_integration.py'),
                       '--workspace', str(root), '--manifest', str(manifest)]
            subprocess.run(command, check=True, capture_output=True)
            node = destination/'fastlio2/src/lio_node.cpp'
            before = node.read_bytes()
            # init_workspace.sh calls this fetcher, which must never reset a dirty tree.
            script = root/'src/agt_field_test_workbench/scripts/fetch_locked.py'
            script.parent.mkdir(parents=True)
            shutil.copy2(REPO/'scripts/fetch_locked.py', script)
            (root/'.repos').write_text(json.dumps({'repositories': {
                'external/fast_lio2_mapping': {'version': spec['commit'], 'url': 'unused'}}}))
            subprocess.run(['python3', str(script)], check=True, capture_output=True)
            self.assertEqual(node.read_bytes(), before)
            subprocess.run(command, check=True, capture_output=True)
            self.assertEqual(node.read_bytes(), before)
            for name in ('fastlio2/test/latency_replay.py', 'fastlio2/test/latency_replay_qos.yaml'):
                self.assertTrue((destination/name).is_file())
            self.assertIn(b'fault_reason', before)
            self.assertIn(b'MultiThreadedExecutor', before)
            node.write_bytes(before+b'\n// Operator edit must survive initialization.\n')
            subprocess.run(['python3', str(script)], check=True, capture_output=True)
            edited = node.read_bytes()
            result = subprocess.run(command, capture_output=True)
            self.assertNotEqual(result.returncode, 0)
            self.assertIn(b'preserved', result.stderr)
            self.assertEqual(node.read_bytes(), edited)
