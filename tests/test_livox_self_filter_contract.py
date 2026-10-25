"""Protect the time-preserving ROS contract in the mapping launch."""
from pathlib import Path
import yaml

ROOT = Path(__file__).resolve().parents[1]


def test_livox_self_filter_default_on_and_core_reused():
    launch = (ROOT / 'agt_field_test_bringup/launch/no_pgo.launch.py').read_text()
    cpp = (ROOT / 'agt_field_test_bringup/src/livox_self_filter.cpp').read_text()
    cfg = yaml.safe_load(
        (ROOT / 'agt_field_test_bringup/config/livox_self_filter.yaml').read_text())
    params = cfg['field_livox_self_filter']['ros__parameters']
    assert "('self_filter_enabled','true')" in launch
    assert "'/field/self_filtered/lidar' if filtered" in launch
    assert "for n in nodes if isinstance(n,Node)" in launch
    assert "agt_pointcloud_core/filters.hpp" in cpp
    assert "offset_time" in cpp and "max_end_time_trim_ns" in cpp
    assert "p.tag=static_cast<std::uint8_t>" in cpp
    assert "0x20U" in cpp
    assert "output.points.clear()" not in cpp
    assert "output.point_num=" not in cpp
    assert "i+=filter_num" in cpp
    assert "i+=static_cast<std::size_t>(lio_filter_num_)" in cpp
    assert "valid_lio(output.points[i])" in cpp
    assert "BYPASS_LIO_SAMPLES" in cpp
    for param in ('lidar_filter_num','lidar_min_range','lidar_max_range'):
        assert "data['" + param + "']" in launch
    assert params['input_topic'] == '/field/aligned/lidar'
    assert params['output_topic'] == '/field/self_filtered/lidar'
    assert params['target_frame'] == 'base_link'
    assert 0. <= params['range_min_m'] < params['range_max_m']
    assert params['max_end_time_trim_ns'] == 1000000
    assert params['max_removed_fraction'] < .5
    assert 'rear_sector' not in str(params)
