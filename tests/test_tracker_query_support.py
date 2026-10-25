"""A fixed spatial support contract; never crop by correspondence/inlier status."""
import numpy as np
import pytest
from agt_map_tracker.map_tracker import crop_query_to_local_map


def test_query_uses_map_gravity_and_preserves_base_coordinates():
    # With a 90-degree pitch, base +X is vertical in the map frame.
    rotation = np.array([[0.,0.,1.],[0.,1.,0.],[-1.,0.,0.]])
    points = np.array([[4.,0.,0.],[6.,0.,0.],[0.,19.,0.],[0.,21.,0.],
                       [0.,0.,19.],[0.,0.,21.],[0.,0.,0.]])
    result = crop_query_to_local_map(points, rotation, 20., 5.)
    np.testing.assert_array_equal(result, points[[0,2,4,6]])


def test_crop_does_not_hide_unmatched_points_inside_search_volume():
    # No target map or NN information enters this operation. Unmatched points
    # within the crop remain in the denominator and can still reject alignment.
    points = np.array([[1.,2.,3.],[-19.,0.,0.],[0.,0.,-5.],[20.,0.,0.],
                       [20.01,0.,0.],[0.,0.,5.01]])
    result = crop_query_to_local_map(points, np.eye(3), 20., 5.)
    np.testing.assert_array_equal(result, points[:4])
    assert crop_query_to_local_map(np.empty((0,3)), np.eye(3), 20., 5.).shape == (0,3)


@pytest.mark.parametrize('case', ['accept', 'too_few', 'low_overlap', 'large_innovation'])
def test_tracker_crop_preserves_rejection_gates(tmp_path, monkeypatch, case):
    import json
    import os
    from types import SimpleNamespace
    import rclpy
    from geometry_msgs.msg import TransformStamped
    from agt_map_tracker import map_tracker as module
    if os.environ.get('ROS_DOMAIN_ID') != '183' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        pytest.skip('requires isolated ROS domain 183 on localhost')
    rclpy.init(args=[])
    node = module.MapTracker()
    try:
        from rclpy.parameter import Parameter
        map_path = tmp_path/'fixture_map.pcd'
        map_path.touch()
        node.set_parameters([Parameter('global_map', value=str(map_path)),
                             Parameter('query_voxel_leaf_m', value=0.)])
        inside = np.random.default_rng(42).uniform([-10.,-10.,-2.],[10.,10.,2.],
                                                  size=(500 if case == 'too_few' else 2000,3))
        outside = np.random.default_rng(43).uniform([22.,0.,0.],[28.,1.,1.], size=(3000,3))
        points = np.concatenate([inside,outside])
        node._clouds.append(module.CloudSample(node.get_clock().now().nanoseconds, points, np.eye(4)))
        node._localized = node._correction_valid = True
        transform = TransformStamped()
        transform.transform.rotation.w = 1.
        monkeypatch.setattr(node._tf, 'lookup_transform', lambda *a,**k:transform)
        statuses, poses, calls = [], [], []
        monkeypatch.setattr(node, '_publish_status', lambda state,**fields:statuses.append((state,fields)))
        node._pose_pub = SimpleNamespace(publish=poses.append)
        def backend(command, **kwargs):
            path = command[command.index('--scan')+1]
            text = open(path).read()
            assert 'POINTS 2000\n' in text  # outside target volume never enters native query
            calls.append(command)
            result = dict(success=True, x=1. if case == 'large_innovation' else 0.,y=0.,z=0.,
                          qx=0.,qy=0.,qz=0.,qw=1.,fitness=.1,
                          overlap=.2 if case == 'low_overlap' else .8,
                          map_points=5000,hessian_eigenvalues=[1.,2.,3.,4.,5.,6.],
                          hessian_condition_number=6.,hessian_degenerate=False)
            return SimpleNamespace(returncode=0,stdout=json.dumps(result),stderr='')
        monkeypatch.setattr(module.subprocess, 'run', backend)
        node._track()
        state, fields = statuses[-1]
        assert fields['query_points_before_map_crop'] == len(points)
        if case == 'accept':
            assert state == 'TRACKING_OK' and len(poses) == 1
        else:
            assert state == 'HOLD' and not poses
            expected = {'too_few':'MIN_QUERY_POINTS', 'low_overlap':'OVERLAP_LOW',
                        'large_innovation':'INNOVATION_HIGH'}[case]
            assert fields['reason_codes'] == [expected]
        assert len(calls) == (0 if case == 'too_few' else 1)
        assert node.get_parameter('min_overlap').value == .4
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_patches_reapply_after_clean_initialization_and_preserve_conflicts(tmp_path):
    import subprocess
    from pathlib import Path
    repo = Path(__file__).resolve().parents[1]
    native = tmp_path/'src/agt_navigation_v3'
    native.parent.mkdir(parents=True)
    subprocess.run(['git','clone','-q','--shared','--no-checkout',str(repo.parent/'agt_navigation_v3'),str(native)],check=True)
    subprocess.run(['git','-C',str(native),'checkout','-q','--detach',
                    'b3a60487dff973c6326aca2ca2e9db1e78daeaaa'],check=True)
    def apply(name, check=False):
        args=['python3',str(repo/'scripts/apply_control_integration.py'),
              '--workspace',str(tmp_path),'--manifest',str(repo/f'patches/{name}_manifest.json')]
        return subprocess.run(args+(['--check'] if check else []),capture_output=True,text=True)
    for name in ['photo_mission','map_tracking_support']:
        assert apply(name).returncode == 0
        assert apply(name).returncode == 0
        assert apply(name,check=True).returncode == 0
    source = native/'navigation/localization/agt_map_tracker/agt_map_tracker/map_tracker.py'
    source.write_text(source.read_text()+'\n# unrelated operator edit\n')
    previous = source.read_bytes()
    result=apply('map_tracking_support')
    assert result.returncode != 0 and 'preserved' in result.stderr
    assert source.read_bytes() == previous
