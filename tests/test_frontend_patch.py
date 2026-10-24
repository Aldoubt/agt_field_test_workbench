import json
from pathlib import Path
import subprocess
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
