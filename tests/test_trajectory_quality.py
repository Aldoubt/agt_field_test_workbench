import csv
from pathlib import Path
import tempfile
import unittest
from workbench.trajectory import inspect_trajectory


class TrajectoryQualityTests(unittest.TestCase):
    def inspect(self, poses):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'trajectory.csv'
            with path.open('w') as stream:
                writer=csv.writer(stream)
                writer.writerow(('stamp','x','y','z','qx','qy','qz','qw'))
                writer.writerows([*pose,0,0,0,1] for pose in poses)
            return inspect_trajectory(path)

    def test_stationary_and_normal_walk_are_numerically_valid(self):
        for poses in ([(1,0,0,0),(1.1,0,0,0)],[(1,0,0,0),(1.1,.05,0,.001)]):
            result=self.inspect(poses)
            self.assertEqual(result['state'],'PASS')
            self.assertIn('independent evidence',result['scope'])

    def test_divergent_bag_cannot_reach_grid_import(self):
        result=self.inspect([(1,0,0,0),(1.1,-5534,95829,-508164)])
        self.assertEqual(result['state'],'REJECTED')
        self.assertIn('implausible_speed',result['reasons'])
        self.assertIn('position_out_of_bounds',result['reasons'])

    def test_timestamp_reversal_and_nan_are_rejected(self):
        self.assertIn('nonincreasing_pose_stamp',self.inspect([(2,0,0,0),(1,0,0,0)])['reasons'])
        self.assertIn('nonfinite_pose',self.inspect([(1,0,0,0),(2,float('nan'),0,0)])['reasons'])
