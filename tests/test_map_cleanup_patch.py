import json
from pathlib import Path
import subprocess
import tempfile
import unittest

REPO=Path(__file__).resolve().parents[1]


class MapCleanupPatchTests(unittest.TestCase):
    def test_fresh_pinned_source_idempotence_and_preservation(self):
        manifest=REPO/'patches/map_cleanup_manifest.json'
        spec=json.loads(manifest.read_text())['agt-lio-pgo-mapping']
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);destination=root/'src/agt-lio-pgo-mapping'
            source=REPO.parent/'agt-lio-pgo-mapping'
            subprocess.run(['git','clone','-q','--shared','--no-checkout',str(source),str(destination)],check=True)
            subprocess.run(['git','-C',str(destination),'checkout','-q','--detach',spec['commit']],check=True)
            cmd=['python3',str(REPO/'scripts/apply_control_integration.py'),'--workspace',str(root),'--manifest',str(manifest)]
            subprocess.run(cmd,check=True,capture_output=True)
            subprocess.run(cmd,check=True,capture_output=True)
            header=destination/'exporters/agt_pcd2grid_exporter/include/agt_pcd2grid_exporter/TraversabilityGridBuilder.hpp'
            self.assertTrue(header.exists())
            header.write_text(header.read_text()+'\n// Operator edit\n')
            before=header.read_bytes()
            result=subprocess.run(cmd,capture_output=True)
            self.assertNotEqual(result.returncode,0)
            self.assertIn(b'preserved',result.stderr)
            self.assertEqual(header.read_bytes(),before)
