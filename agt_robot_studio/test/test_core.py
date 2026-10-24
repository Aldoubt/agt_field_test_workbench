from types import SimpleNamespace
import numpy as np
from agt_robot_studio.map_io import occupancy_to_gray, grayscale_to_occupancy, save_nav2_map
from agt_robot_studio.cloud import xyz_from_cloud
from agt_robot_studio.ros_worker import TopicRate


def test_occupancy_flip_unknown_and_write(tmp_path):
    pix = occupancy_to_gray([0, -1, 100, 0], 2, 2)
    assert pix.tolist() == [[0, 254], [254, 205]]
    assert pix.flags.c_contiguous
    assert grayscale_to_occupancy(pix) == [0, -1, 100, 0]
    path = save_nav2_map(tmp_path/'new', pix, 0.05, (0., -1., .2))
    assert path.endswith('map.yaml')
    assert 'origin: [0.00000000, -1.00000000, 0.20000000]' in (tmp_path/'new/map.yaml').read_text()
    assert (tmp_path/'new/map.pgm').read_bytes().endswith(pix.tobytes())


def test_topic_rate_uses_measured_arrivals_and_expires():
    rate=TopicRate(window_sec=3.)
    assert rate.hz(now=0.) is None
    for stamp in (0.,.1,.2,.3,.4):rate.add(stamp)
    assert rate.hz(now=.4)==10.
    assert rate.hz(now=4.) is None


def test_cloud_xyz_and_padding():
    fields = [SimpleNamespace(name=k, datatype=7, offset=i*4) for i, k in enumerate('xyz')]
    rows = np.array([[1,2,3],[4,5,6]], dtype='<f4').tobytes()
    cloud = SimpleNamespace(width=2,height=1,point_step=12,row_step=24,fields=fields,
                            is_bigendian=False,data=rows)
    assert xyz_from_cloud(cloud).tolist() == [[1,2,3],[4,5,6]]
    cloud.data = rows[:-2]
    assert xyz_from_cloud(cloud).size == 0
