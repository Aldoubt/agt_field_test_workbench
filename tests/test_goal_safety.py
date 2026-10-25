"""Regression coverage for field navigation's map/yaw/Unknown contract."""
import math
import tempfile
from pathlib import Path
import unittest

from workbench.goal_safety import NavigableGrid, footprint_clearance, validate_endpoints
from workbench.maps import write_pgm, Maps
from workbench.routes import Routes


class GoalSafetyTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.map_dir = self.root / 'raster'
        self.map_dir.mkdir()
        self.width = self.height = 21
        self.pixels = bytearray([254] * (self.width * self.height))
        self.pgm = self.map_dir / 'map.pgm'
        self.record = dict(pgm=str(self.pgm),width=self.width,height=self.height,
                           resolution=.10,origin=[10.,20.,math.pi/2])
        self.flush()

    def tearDown(self):
        self.temp.cleanup()

    def flush(self):
        write_pgm(self.pgm,self.width,self.height,bytes(self.pixels))

    def pose(self,xcell,ycell):
        # 90-degree rotation; world X decreases as grid Y increases.
        return dict(x=10. - (ycell + .5) * .10,
                    y=20. + (xcell + .5) * .10,yaw=0.)

    def test_yaw_transform_and_occupied_or_unknown_rejected(self):
        grid=NavigableGrid(self.record)
        target=self.pose(10,10)
        self.assertEqual(grid.cell(target['x'],target['y']),(10,10))
        grid.require_clear(target)
        self.pixels[10*self.width+10]=0
        self.flush()
        with self.assertRaisesRegex(ValueError,'occupied/unknown'):
            NavigableGrid(self.record).require_clear(target)
        self.pixels[10*self.width+10]=205
        self.flush()
        with self.assertRaises(ValueError):
            NavigableGrid(self.record).require_clear(target)

    def test_robot_footprint_catches_nearby_unknown(self):
        profile={'footprint_m':[[.42,.33],[.42,-.33],[-.42,-.33],[-.42,.33]]}
        self.assertGreater(footprint_clearance(profile),.63)
        self.pixels[10*self.width+12]=205
        self.flush()
        with self.assertRaisesRegex(ValueError,'waypoint 1'):
            validate_endpoints(self.record,[self.pose(10,10)],profile)
        validate_endpoints(self.record,[self.pose(10,10)])

    def test_outside_map_and_wrong_dimensions_rejected(self):
        with self.assertRaisesRegex(ValueError,'outside'):
            NavigableGrid(self.record).require_clear({'x':100.,'y':100.})
        bad=dict(self.record,width=200)
        with self.assertRaisesRegex(ValueError,'dimensions'):
            NavigableGrid(bad)

    def test_route_save_checks_free_center_before_writing_mission(self):
        meta=dict(image='map.pgm',resolution=.10,origin=[10.,20.,math.pi/2],
                  negate=0,occupied_thresh=.65,free_thresh=.196)
        created=Maps(self.root)._save(self.width,self.height,bytes(self.pixels),meta,{})
        valid=self.pose(10,10)
        self.assertEqual(len(Routes(self.root).save(created['id'],[valid])['points']),1)
        occupied=self.pose(1,1)
        self.pixels[(self.height-1-1)*self.width+1]=205
        self.flush()
        # A separate version must be created when map pixels change.
        updated=Maps(self.root)._save(self.width,self.height,bytes(self.pixels),meta,{})
        with self.assertRaisesRegex(ValueError,'waypoint 1'):
            Routes(self.root).save(updated['id'],[occupied])


if __name__=='__main__':
    unittest.main()
