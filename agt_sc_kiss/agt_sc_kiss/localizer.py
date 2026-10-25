"""SC retrieval -> real KISS coarse -> existing native small_gicp. No ROS publisher."""
import argparse
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import numpy as np
from scipy.spatial.transform import Rotation
from .database import Database
from .geometry import write_pcd


def require_kiss():
    # Reuse existing isolated deployment, if not already installed/importable.
    root=Path(os.environ.get('AGT_FIELD_TEST_WS',str(Path.home()/'agt_field_test_ws')))
    for path in (root/'work/kiss_python_locked',root/'work/kiss_python'):
        if path.is_dir():
            sys.path.insert(0,str(path));break
    import kiss_matcher
    return kiss_matcher


def native_gicp(map_pcd,scan,coarse,budget,radius=35.,half_height=8.):
    from ament_index_python.packages import get_package_prefix
    executable=Path(get_package_prefix('agt_global_relocalization_native'))/'lib/agt_global_relocalization_native/map_gicp_tracker'
    xyz=coarse[:3,3];q=Rotation.from_matrix(coarse[:3,:3]).as_quat()
    args=[str(executable),'--map',str(map_pcd),'--scan',str(scan),'--radius',str(radius),'--half-height',str(half_height)]
    for name,value in zip(('x','y','z','qx','qy','qz','qw'),[*xyz,*q]):args+=['--'+name,str(float(value))]
    started=time.monotonic();proc=subprocess.run(args,capture_output=True,text=True,timeout=budget)
    rows=[row for row in proc.stdout.splitlines() if row.startswith('{')]
    if not rows:raise RuntimeError('native GICP returned no JSON: '+proc.stderr[-500:])
    result=json.loads(rows[-1]);result['gicp_elapsed_sec']=time.monotonic()-started
    if proc.returncode!=0:result['success']=False
    return result


def localize(map_pcd,scan,db,spec,query_quaternion,timeout=15.,top_k=3,radius=35.,half_height=8.):
    from . import _scancontext as sc
    started=time.monotonic();deadline=started+timeout
    database=Database(db,map_pcd,spec)
    body,level,candidates=database.query(sc.load_pcd(str(scan)),query_quaternion,top_k)
    retrieval_time=time.monotonic()-started
    if len(body)<2000:raise ValueError('SC query has fewer than the existing 2000-point minimum')
    if not candidates:raise ValueError('SCAN_CONTEXT_NO_CANDIDATE')
    km=require_kiss();map_points=sc.load_pcd(str(map_pcd));results=[]
    with tempfile.TemporaryDirectory(prefix='sc_kiss_') as temporary:
        query_path=Path(temporary)/'query.pcd';write_pcd(query_path,body)
        for candidate in candidates:
            if deadline-time.monotonic()<.1:raise TimeoutError('SC_KISS_TOTAL_TIMEOUT')
            center=np.array(candidate['translation'])
            mask=(np.linalg.norm(map_points[:,:2]-center[:2],axis=1)<=radius)&(np.abs(map_points[:,2]-center[2])<=half_height)
            target=map_points[mask]-center
            if len(target)<1000:continue
            seed=Rotation.from_euler('z',candidate['yaw_seed']).as_matrix()@level
            matcher=km.KISSMatcher(km.KISSMatcherConfig(.3))
            before=time.monotonic()
            # Source -> target. SC seed and candidate center are applied exactly once.
            estimate=matcher.estimate(np.ascontiguousarray(body@seed.T),np.ascontiguousarray(target))
            rot=np.array(estimate.rotation,copy=True);translation=np.array(estimate.translation,copy=True).reshape(3)
            inliers=int(matcher.get_num_final_inliers());rotation_inliers=int(matcher.get_num_rotation_inliers())
            if (inliers<20 or not np.isfinite(rot).all() or not np.isfinite(translation).all()
                    or not np.allclose(rot.T@rot,np.eye(3),atol=1e-3) or abs(np.linalg.det(rot)-1)>1e-3
                    or np.linalg.norm(translation)>radius):continue
            coarse=np.eye(4);coarse[:3,:3]=rot@seed;coarse[:3,3]=translation+center
            kiss_time=time.monotonic()-before
            remaining=deadline-time.monotonic()
            if remaining<=0:raise TimeoutError('SC_KISS_TOTAL_TIMEOUT')
            result=native_gicp(map_pcd,query_path,coarse,remaining,radius,half_height)
            result.update(candidate_patch=candidate['name'],descriptor_similarity=1-candidate['distance'],
                descriptor_yaw_seed_deg=math.degrees(candidate['yaw_seed']),kiss_final_inliers=inliers,
                kiss_rotation_inliers=rotation_inliers,kiss_elapsed_sec=kiss_time,
                coarse_elapsed_sec=retrieval_time+kiss_time,sc_elapsed_sec=retrieval_time)
            for name,value in zip(('coarse_x','coarse_y','coarse_z','coarse_qx','coarse_qy','coarse_qz','coarse_qw'),
                                  [*coarse[:3,3],*Rotation.from_matrix(coarse[:3,:3]).as_quat()]):result[name]=float(value)
            # Never present descriptor similarity alone as pose quality.
            result['score']=min(result['descriptor_similarity'],float(result.get('overlap',0)))
            values=[result.get(k,float('nan')) for k in ('x','y','z','qx','qy','qz','qw','fitness','overlap','score')]
            if (result.get('success') and not result.get('hessian_degenerate',True) and np.isfinite(values).all() and result['score']>=.5
                    and result['fitness']<=1. and result['overlap']>=.2):results.append(result)
    if not results:raise ValueError('SC_KISS_GICP_QUALITY_REJECTED')
    results.sort(key=lambda value:value['fitness'])
    best=results[0]
    if any(abs(other['fitness']-best['fitness'])<.02 and np.linalg.norm(np.array([other['x'],other['y']])-np.array([best['x'],best['y']]))>2 for other in results[1:]):
        raise ValueError('SC_KISS_AMBIGUOUS_CANDIDATES')
    from .geometry import sha
    if sha(map_pcd)!=database.meta['map_sha256']:raise ValueError('map changed during sc_kiss; result discarded')
    database.verify_navigation()
    best.update(backend='sc_kiss',total_elapsed_sec=time.monotonic()-started,
        sc_candidates=[dict(name=c['name'],distance=c['distance'],yaw_seed=c['yaw_seed']) for c in candidates])
    return best


def main():
    p=argparse.ArgumentParser();p.add_argument('--map',required=True);p.add_argument('--scan',required=True)
    p.add_argument('--database',required=True);p.add_argument('--contract',required=True)
    p.add_argument('--query-roll',type=float,required=True);p.add_argument('--query-pitch',type=float,required=True)
    p.add_argument('--timeout',type=float,default=15.);p.add_argument('--radius',type=float,default=35.)
    p.add_argument('--half-height',type=float,default=8.);a=p.parse_args()
    try:
        if not all(math.isfinite(v) for v in (a.query_roll,a.query_pitch,a.timeout,a.radius,a.half_height)) or min(a.timeout,a.radius,a.half_height)<=0:raise ValueError('invalid experimental limits/attitude')
        result=localize(a.map,a.scan,a.database,json.loads(Path(a.contract).read_text()),
            Rotation.from_euler('xyz',[a.query_roll,a.query_pitch,0.]).as_quat(),a.timeout,radius=a.radius,half_height=a.half_height)
    except Exception as exc:result=dict(success=False,message=str(exc),backend='sc_kiss')
    print(json.dumps(result,allow_nan=False));return 0 if result.get('success') else 2
