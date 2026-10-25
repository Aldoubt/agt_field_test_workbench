import csv
import json
import tempfile
import unittest
from pathlib import Path

import numpy as np
from workbench.map_cleanup import prepare_keyframes, base_from_body, quaternion_matrix


class CleanupEvidenceTests(unittest.TestCase):
    def fixture(self, root):
        xyz=np.array([[1,2,3],[2,3,4],[5,6,7]],dtype='<f4')
        xyz.tofile(root/'points.raw')
        (root/'frames.csv').write_text('stamp,stamp_ns,first_point,point_count\n1,1000000000,0,2\n2,2000000000,2,1\n')
        (root/'trajectory.csv').write_text('stamp,x,y,z,qx,qy,qz,qw\n1,1,2,3,0,0,0,1\n2,5,6,7,0,0,0.7071067811865475,0.7071067811865475\n')
        return xyz

    def test_exact_join_covers_all_points_and_restores_body_frame(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp); xyz=self.fixture(root)
            result=prepare_keyframes(root,root/'keyframes')
            self.assertEqual((result['frames'],result['points'],result['exact_pose_matches']),(2,3,2))
            patch=(root/'keyframes/patches/000000.pcd').read_bytes().split(b'DATA binary\n')[1]
            np.testing.assert_allclose(np.frombuffer(patch,dtype='<f4').reshape(-1,3),xyz[:2]-xyz[0])

    def test_missing_pose_or_invalid_index_is_rejected_before_output(self):
        for old,new in [('2,2000000000,2,1','3,3000000000,2,1'),
                        ('2,2000000000,2,1','2,2000000000,1,1')]:
            with tempfile.TemporaryDirectory() as tmp:
                root=Path(tmp);self.fixture(root)
                path=root/'frames.csv';path.write_text(path.read_text().replace(old,new))
                with self.assertRaises(ValueError):prepare_keyframes(root,root/'keyframes')
                self.assertFalse((root/'keyframes').exists())

    def test_internal_extrinsic_is_inverted_not_added(self):
        bundle={'mount':{'xyz':[1,2,3],'rpy':[0,0,0]},'base_link_height_m':.2,
                'internal':{'fastlio2':{'rotation':np.eye(3).ravel().tolist(),'translation':[.1,.2,.3]}}}
        result=base_from_body(bundle)
        np.testing.assert_allclose(result['xyz'],[.9,1.8,2.9])
        np.testing.assert_allclose(result['rpy'],[0,0,0])
        q=[0,0,np.sqrt(.5),np.sqrt(.5)]
        np.testing.assert_allclose(quaternion_matrix(q)@[1,0,0],[0,1,0],atol=1e-15)
