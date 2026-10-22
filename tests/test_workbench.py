import base64
from pathlib import Path
import struct
import tempfile
import time
import unittest

import yaml

from workbench.maps import Maps, read_pgm
from workbench.cloud import sample_pcd
from workbench.runtime import Jobs, check_bag


class WorkbenchTests(unittest.TestCase):
    def test_binary_pcd_preview_reads_xyz_offsets(self):
        with tempfile.TemporaryDirectory() as temp:
            pcd = Path(temp) / 'map.pcd'
            header = (b'FIELDS intensity x y z\nSIZE 4 4 4 4\nTYPE F F F F\n'
                      b'COUNT 1 1 1 1\nWIDTH 2\nHEIGHT 1\nPOINTS 2\nDATA binary\n')
            pcd.write_bytes(header + struct.pack('<ffff', 7, 1, 2, 3) +
                            struct.pack('<ffff', 8, -1, -2, -3))
            self.assertEqual(sample_pcd(pcd)['points'], [[1, 2, 3], [-1, -2, -3]])

    def test_map_versions_preserve_parent_and_input(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            source = root / 'runs/source'
            source.mkdir(parents=True)
            (source / 'map.pgm').write_bytes(b'P5\n2 2\n255\n\x00\xff\xcd\xff')
            (source / 'map.yaml').write_text(yaml.safe_dump({
                'image': 'map.pgm', 'resolution': 0.1, 'origin': [1, 2, 0],
                'negate': 0, 'occupied_thresh': 0.65, 'free_thresh': 0.196}))
            maps = Maps(root)
            first = maps.import_yaml(source / 'map.yaml')
            second = maps.revise(first['id'], base64.b64encode(b'\xff\xff\xcd\xff').decode())
            self.assertEqual(first['status'], 'DRAFT')
            self.assertEqual(second['parent'], first['id'])
            self.assertNotEqual(first['sha256'], second['sha256'])
            self.assertEqual(read_pgm(source / 'map.pgm')[2], b'\x00\xff\xcd\xff')
            self.assertEqual(read_pgm(Path(first['pgm']))[2], b'\x00\xff\xcd\xff')
            with self.assertRaises(ValueError):
                maps.import_yaml(source / 'map.yaml' .replace('map', '../outside'))

    def test_incomplete_bag_is_never_pass(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp)
            self.assertEqual(check_bag(path)['state'], 'INCOMPLETE')
            (path / 'metadata.yaml').write_text(yaml.safe_dump({'rosbag2_bagfile_information': {
                'relative_file_paths': ['missing.db3'],
                'topics_with_message_count': [{'topic_metadata': {'name': '/livox/imu'}, 'message_count': 10}]
            }}))
            self.assertEqual(check_bag(path)['state'], 'INCOMPLETE')

    def test_cancel_only_owned_process_group(self):
        with tempfile.TemporaryDirectory() as temp:
            jobs = Jobs(temp)
            first = jobs._new('mapping_bag', ['sleep', '30'], {})
            with self.assertRaises(ValueError):
                jobs._new('mapping_live', ['sleep', '30'], {})
            second = jobs._new('recorder', ['sleep', '30'], {'output': str(Path(temp) / 'bags/test')})
            stopped = jobs.stop(first['id'])
            self.assertEqual(stopped['state'], 'CANCELLED')
            self.assertEqual(next(j for j in jobs.list() if j['id'] == second['id'])['state'], 'RUNNING')
            jobs.stop(second['id'])


if __name__ == '__main__':
    unittest.main()
