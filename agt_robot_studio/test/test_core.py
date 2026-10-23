import math
from types import SimpleNamespace
import numpy as np
import pytest
from agt_robot_studio.control import wrap_pi, forward_distance, ClosedLoopMotion
from agt_robot_studio.map_io import occupancy_to_gray, grayscale_to_occupancy, save_nav2_map
from agt_robot_studio.cloud import xyz_from_cloud


def test_distance_projected_along_initial_heading():
    assert forward_distance((0, 0, math.pi/2), (2, 3, 0)) == pytest.approx(3)
    assert forward_distance((0, 0, 0), (-1, 0, 0)) == -1


def test_reverse_and_rotation_wrap():
    c = ClosedLoopMotion()
    c.begin('distance', -1, (0, 0, 0))
    v, _, _, done = c.tick((0, 0, 0), .02)
    assert v < 0 and not done
    c.begin('rotation', math.radians(30), (0, 0, math.radians(170)))
    _, w, progress, _ = c.tick((0, 0, math.radians(-175)), .02)
    assert w > 0 and progress == pytest.approx(math.radians(15))


def test_occupancy_flip_unknown_and_write(tmp_path):
    pix = occupancy_to_gray([0, -1, 100, 0], 2, 2)
    assert pix.tolist() == [[0, 254], [254, 205]]
    assert grayscale_to_occupancy(pix) == [0, -1, 100, 0]
    path = save_nav2_map(tmp_path/'new', pix, 0.05, (0., -1., 0.))
    assert path.endswith('map.yaml')
    assert (tmp_path/'new/map.pgm').read_bytes().endswith(pix.tobytes())
    with pytest.raises(FileExistsError):
        save_nav2_map(tmp_path/'new', pix, 0.05, (0., 0., 0.))


def test_cloud_xyz_and_padding():
    fields = [SimpleNamespace(name=k, datatype=7, offset=i*4) for i, k in enumerate('xyz')]
    rows = np.array([[1,2,3],[4,5,6]], dtype='<f4').tobytes()
    cloud = SimpleNamespace(width=2,height=1,point_step=12,row_step=24,fields=fields,
                            is_bigendian=False,data=rows)
    assert xyz_from_cloud(cloud).tolist() == [[1,2,3],[4,5,6]]
    cloud.data = rows[:-2]
    assert xyz_from_cloud(cloud).size == 0
