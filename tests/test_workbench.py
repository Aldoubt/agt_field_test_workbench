import base64
from pathlib import Path
import struct
import tempfile
import time
import unittest

import yaml

from workbench.maps import Maps, read_pgm, hash_file
from workbench.routes import Routes
from workbench.cloud import sample_pcd
from workbench.artifacts import Clouds
from workbench.runtime import Jobs, check_bag, validate_recording_topics


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
            first = maps.import_yaml(source / 'map.yaml', name='一号温室')
            second = maps.revise(first['id'], base64.b64encode(b'\xff\xff\xcd\xff').decode(),
                                 keepout_polygons_px=[[[0, 0], [1, 0], [1, 1]]])
            self.assertEqual(first['status'], 'DRAFT')
            self.assertEqual(second['parent'], first['id'])
            self.assertEqual(second['name'], '一号温室')
            self.assertNotEqual(first['sha256'], second['sha256'])
            self.assertEqual(read_pgm(source / 'map.pgm')[2], b'\x00\xff\xcd\xff')
            self.assertEqual(read_pgm(Path(first['pgm']))[2], b'\x00\xff\xcd\xff')
            keepout=yaml.safe_load(Path(second['keepout_zones']).read_text())
            self.assertEqual(len(keepout['polygons']), 1)
            self.assertEqual(keepout['zones'][0]['type'],'keepout')
            self.assertEqual(read_pgm(Path(second['pgm']))[2],b'\x00\x00\xcd\x00')
            self.assertTrue(Path(second['keepout_bake']).is_file())
            with self.assertRaises(ValueError):
                maps.import_yaml(source / 'map.yaml' .replace('map', '../outside'))

    def test_photo_route_generates_existing_v3_mission_schema(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);source=root/'runs/source';source.mkdir(parents=True)
            (source/'map.pgm').write_bytes(b'P5\n2 2\n255\n\x00\xff\xcd\xff')
            (source/'map.yaml').write_text(yaml.safe_dump({'image':'map.pgm','resolution':.1,
                'origin':[0,0,0],'negate':0,'occupied_thresh':.65,'free_thresh':.196}))
            map_record=Maps(root).import_yaml(source/'map.yaml')
            route=Routes(root).save(map_record['id'],[{'x':1.,'y':2.,'yaw':.3,'photo':True,
                'heading':-30.,'pitch':10.}],name='inspection')
            mission=yaml.safe_load(Path(route['mission_file']).read_text())
            self.assertEqual(mission['map_id'],map_record['id'])
            self.assertEqual(mission['points'][0]['pose']['frame_id'],'map')
            self.assertEqual(mission['points'][0]['views'][0]['heading'],-30.)
            self.assertTrue(mission['points'][0]['views'][0]['required'])
            self.assertTrue(mission['points'][0]['views'][0]['save_image'])
            self.assertEqual(route['photo_task_executor'],'agt_navigation_runtime/mission_runtime')
            with self.assertRaises(ValueError):
                Routes(root).save(map_record['id'],[{'x':0.,'y':0.,'yaw':0.,'heading':999.}])

    def test_partial_projection_uses_pcd_bounds_and_inherits_name(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);cloud_id='20261024T000000Z-deadbeef'
            folder=root/'maps/clouds'/cloud_id;folder.mkdir(parents=True)
            pcd=folder/'cloud.pcd';pcd.write_bytes(b'PCD')
            (folder/'manifest.json').write_text(__import__('json').dumps({'id':cloud_id,
                'pcd':str(pcd),'sha256':hash_file(pcd),'name':'一号温室',
                'processing':{'bounds_min':[1.,2.,-.2],'bounds_max':[2.,4.,2.]}}))
            clouds=Clouds(root);seen={}
            def fake_process(source,spec):
                seen.update(spec);ident='projection';out=root/'runs'/ident;out.mkdir(parents=True)
                p=spec['projection']
                (out/'map.yaml').write_text(yaml.safe_dump({'image':'map.pgm','resolution':p['resolution'],
                    'origin':p['origin'],'negate':0,'occupied_thresh':.65,'free_thresh':.2}))
                (out/'map.pgm').write_bytes(b'P5\n2 2\n255\n\x00\xff\xcd\xff')
                return ident,out,out/'operations.yaml'
            clouds._process=fake_process
            result=clouds.grid(cloud_id,{'resolution':.5,'z_min':.2})
            self.assertEqual(result['name'],'一号温室')
            self.assertEqual(seen['projection']['origin'],[.5,1.5,0.])
            self.assertEqual(seen['projection']['range'],[2.,3.])
            self.assertEqual(seen['projection']['z_min'],.2)

    def test_custom_recording_topics_are_validated_and_selected(self):
        from unittest.mock import patch
        self.assertEqual(validate_recording_topics(['/livox/imu','/livox/imu','/tf']),['/livox/imu','/tf'])
        self.assertIsNone(validate_recording_topics([]))
        with self.assertRaises(ValueError):validate_recording_topics(['relative/topic'])
        with tempfile.TemporaryDirectory() as temp:
            jobs=Jobs(temp);captured={}
            def fake_new(kind,command,meta):
                captured.update(kind=kind,command=command,meta=meta);return meta
            with patch('workbench.runtime.ros_ready',return_value=True), \
                 patch('workbench.runtime.topics',return_value={'/livox/imu':'sensor_msgs/msg/Imu','/camera/image_raw':'sensor_msgs/msg/Image'}), \
                 patch('workbench.runtime.ros_call',return_value='qos'), \
                 patch('workbench.runtime.source_versions',return_value={}), \
                 patch.object(jobs,'_new',side_effect=fake_new):
                jobs.start_recorder('full_field_test','bunker',requested_topics=['/camera/image_raw','/missing','/livox/imu'])
            self.assertEqual(captured['meta']['topic_source'],'custom')
            self.assertEqual(set(captured['meta']['topics']),{'/camera/image_raw','/livox/imu'})
            self.assertEqual(captured['meta']['missing_topics'],['/missing'])
            self.assertNotIn('/wheel/odom',captured['command'])

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
            recovered = Jobs(temp).list()
            self.assertEqual({j['id'] for j in recovered}, {first['id'], second['id']})
            self.assertEqual(next(j for j in recovered if j['id'] == second['id'])['state'], 'INCOMPLETE')


if __name__ == '__main__':
    unittest.main()
