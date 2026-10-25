import os
import json
from pathlib import Path
import numpy as np
import pytest
pytestmark=pytest.mark.skipif(os.environ.get('AGT_RUN_SC_TESTS')!='1',reason='requires locked native SC/KISS and Humble overlay')


@pytest.fixture(scope='module')
def data(tmp_path_factory):
    from synthetic import fixture
    return fixture(tmp_path_factory.mktemp('sc_fixed_map'))


def test_upstream_sc_retrieves_place_and_yaw(data):
    from agt_sc_kiss.database import Database
    database=Database(data['database'],data['map'],data['spec'])
    _,_,candidates=database.query(data['points'],data['rotation'].as_quat(),3)
    assert candidates and candidates[0]['name']=='000001.pcd'
    delta=np.arctan2(np.sin(candidates[0]['yaw_seed']-np.radians(36)),np.cos(candidates[0]['yaw_seed']-np.radians(36)))
    assert abs(delta)<np.radians(7)


def test_common_preprocessing_and_gravity_contract(data):
    from agt_sc_kiss.geometry import preprocess,level_rotation,assert_contract
    from agt_sc_kiss.database import Database
    a=preprocess(data['points'],data['spec'])
    assert np.array_equal(a,preprocess(a,data['spec']))
    wrong=dict(data['spec'],body_from_lidar_translation=[.2,0.,0.])
    with pytest.raises(ValueError,match='mismatch'):Database(data['database'],data['map'],wrong)
    level,_=level_rotation(data['rotation'].as_quat())
    from scipy.spatial.transform import Rotation
    assert np.allclose(level,Rotation.from_euler('xyz',[-.05,.08,0.]).as_matrix())


def test_real_kiss_known_se3(data):
    from agt_sc_kiss.localizer import require_kiss
    km=require_kiss();matcher=km.KISSMatcher(km.KISSMatcherConfig(.3))
    # A local target offset is restored once, as in the production adapter.
    result=matcher.estimate(np.ascontiguousarray(data['points']),np.ascontiguousarray(data['map_points']-data['translation']))
    rotation=np.array(result.rotation,copy=True);translation=np.array(result.translation,copy=True).reshape(3)
    from scipy.spatial.transform import Rotation
    assert matcher.get_num_final_inliers()>=20
    assert np.linalg.norm(translation)<.2
    assert Rotation.from_matrix(rotation@data['rotation'].as_matrix().T).magnitude()<np.radians(5)


def test_full_real_sc_kiss_existing_gicp(data):
    from agt_sc_kiss.localizer import localize
    result=localize(data['map'],data['query'],data['database'],data['spec'],data['rotation'].as_quat(),30.)
    assert result['success'] and result['backend']=='sc_kiss' and result['kiss_final_inliers']>=20
    assert np.linalg.norm(np.array([result[k] for k in ('x','y','z')])-data['translation'])<.2
    from scipy.spatial.transform import Rotation
    actual=Rotation.from_quat([result[k] for k in ('qx','qy','qz','qw')])
    assert (actual*data['rotation'].inv()).magnitude()<np.radians(5)
    assert result['candidate_patch']=='000001.pcd' and result['gicp_elapsed_sec']>0


def test_hash_mismatch_fails_without_registration(data,tmp_path):
    from agt_sc_kiss.database import Database
    wrong=tmp_path/'wrong.pcd';wrong.write_bytes(b'not the fixed source map')
    with pytest.raises(ValueError,match='map/version'):Database(data['database'],wrong,data['spec'])


def test_database_rejects_pose_metadata_and_grid_edits(data,tmp_path):
    from agt_sc_kiss.database import Database
    import shutil
    copy=tmp_path/'database';shutil.copytree(data['database'],copy)
    path=copy/'manifest.json';meta=json.loads(path.read_text());meta['entries'][0]['translation'][0]+=10
    path.write_text(json.dumps(meta))
    with pytest.raises(ValueError,match='metadata SHA'):Database(copy,data['map'],data['spec'])
    original=data['map_yaml'].read_bytes()
    try:
        data['map_yaml'].write_bytes(original+b'\n# changed\n')
        with pytest.raises(ValueError,match='navigation map'):Database(data['database'],data['map'],data['spec'])
    finally:data['map_yaml'].write_bytes(original)
