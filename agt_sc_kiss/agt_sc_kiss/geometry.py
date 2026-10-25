"""One coordinate/preprocessing contract for static DB and live queries."""
import hashlib
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation


def sha(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda:stream.read(1024*1024),b''):h.update(block)
    return h.hexdigest()


def contract(translation,quaternion,min_range=.5,max_range=30.,voxel=.25):
    r=Rotation.from_quat(quaternion).as_matrix()
    t=np.asarray(translation,dtype=float)
    if t.shape!=(3,) or not np.isfinite(t).all() or not 0<=min_range<max_range<=30. or not 0<voxel<=1:
        raise ValueError('invalid Scan Context preprocessing contract')
    return dict(frame='mapping_body',gravity='remove_roll_pitch_preserve_yaw',
        body_from_lidar_rotation=np.round(r,12).tolist(),body_from_lidar_translation=t.tolist(),
        min_range=float(min_range),max_range=float(max_range),voxel=float(voxel),
        rings=20,sectors=60,descriptor_radius=30.,height_offset=0.)


def contract_from_lio(path):
    from agt_batch_lio_adapter.extrinsics import load_lio_body_to_lidar
    return contract(*load_lio_body_to_lidar(str(path)))


def assert_contract(actual,expected):
    for key in expected:
        a,b=actual.get(key),expected[key]
        if isinstance(b,(float,list)):
            if np.shape(a)!=np.shape(b) or not np.allclose(a,b,atol=1e-9,rtol=0):raise ValueError('preprocessing/calibration mismatch: '+key)
        elif a!=b:raise ValueError('preprocessing/calibration mismatch: '+key)


def preprocess(body_points,spec):
    a=np.asarray(body_points,dtype=float)[:,:3]
    a=a[np.isfinite(a).all(axis=1)]
    # Range is measured in the sensor frame, exactly as the original runtime.
    lidar=(a-np.array(spec['body_from_lidar_translation']))@np.array(spec['body_from_lidar_rotation'])
    radius=np.linalg.norm(lidar,axis=1)
    a=a[(radius>=spec['min_range'])&(radius<=spec['max_range'])]
    if not len(a):raise ValueError('no observed points after common preprocessing')
    _,indices=np.unique(np.floor(a/spec['voxel']).astype(np.int64),axis=0,return_index=True)
    return np.ascontiguousarray(a[np.sort(indices)])


def level_rotation(quaternion):
    r=Rotation.from_quat(quaternion)
    yaw=r.as_euler('xyz')[2]
    return Rotation.from_euler('z',-yaw).as_matrix()@r.as_matrix(),float(yaw)


def write_pcd(path,points):
    a=np.asarray(points,dtype='<f4')
    with Path(path).open('wb') as stream:
        stream.write(('VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n'
            f'WIDTH {len(a)}\nHEIGHT 1\nPOINTS {len(a)}\nDATA binary\n').encode());stream.write(a.tobytes())


def write_json(path,value):
    Path(path).write_text(json.dumps(value,indent=2,allow_nan=False)+'\n')
