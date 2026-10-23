#!/usr/bin/env python3
"""Common immutable PCD benchmark. KISS coarse registration + locked native GICP.
Queries must already be in the same mapping_body convention as the baseline.
No retrieval DB is assumed: overlapping tiles are ranked by radial descriptors.
"""
import argparse,hashlib,json,math,os,resource,subprocess,sys,time
from pathlib import Path
os.environ.setdefault('OMP_NUM_THREADS','4');os.environ.setdefault('OPENBLAS_NUM_THREADS','1')
import numpy as np
REPO=Path(__file__).resolve().parents[1];ROOT=REPO.parent.parent
sys.path.insert(0,str(REPO));sys.path.insert(0,str(ROOT/'work/kiss_python'));sys.path.insert(0,str(ROOT/'work/kiss_python_locked'))
from workbench.runtime import write_json
from workbench.maps import hash_file

def points(path):
    with open(path,'rb') as f:
        header={}
        while True:
            line=f.readline().decode('ascii').strip()
            if line and not line.startswith('#'):
                key,*values=line.split();header[key]=values
            if line.startswith('DATA '):break
        if header['DATA']==['ascii']:
            a=np.loadtxt(f,ndmin=2);idx=[header['FIELDS'].index(k) for k in ('x','y','z')];return a[:,idx]
        if header['DATA']!=['binary']:raise ValueError('normalize compressed PCD with full-resolution PCD import first')
        dtype=[]
        for k,s,t,c in zip(header['FIELDS'],header['SIZE'],header['TYPE'],header.get('COUNT',['1']*len(header['FIELDS']))):
            base={'F':'f','U':'u','I':'i'}[t]+s;dtype.append((k,'<'+base) if c=='1' else (k,'<'+base,(int(c),)))
        data=np.frombuffer(f.read(),dtype=np.dtype(dtype));a=np.column_stack([data[k] for k in ('x','y','z')])
        return a[np.isfinite(a).all(axis=1)]

def command(cmd,timeout):
    start=time.monotonic();before=resource.getrusage(resource.RUSAGE_CHILDREN)
    try:
        result=subprocess.run(cmd,capture_output=True,text=True,timeout=timeout)
        lines=[line for line in result.stdout.splitlines() if line.startswith('{')]
        value=json.loads(lines[-1]) if lines else dict(success=False,reason='no_json',stderr=result.stderr[-4000:])
        value['exit_code']=result.returncode
    except subprocess.TimeoutExpired:value=dict(success=False,reason='timeout')
    after=resource.getrusage(resource.RUSAGE_CHILDREN)
    value.update(elapsed_sec=time.monotonic()-start,cpu_sec=(after.ru_utime+after.ru_stime-before.ru_utime-before.ru_stime))
    return value

def descriptor(a):
    r=np.linalg.norm(a-np.median(a,axis=0),axis=1);h=np.histogram(r,bins=np.linspace(0,40,33))[0].astype(float);return h/max(h.sum(),1)

def save_pcd(path,a):
    with path.open('wb') as f:
        f.write(('VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH %d\nHEIGHT 1\nPOINTS %d\nDATA binary\n'%(len(a),len(a))).encode());f.write(a.astype('<f4').tobytes())

def kiss(map_points,query,map_path,directory,spec):
    try:import kiss_matcher as km
    except ImportError:return dict(success=False,reason='KISS_BUILD_UNAVAILABLE',state='BLOCKED')
    from scipy.spatial.transform import Rotation
    width=float(spec.get('tile_width_m',30));stride=float(spec.get('tile_stride_m',15))
    if width<=0 or stride<=0:raise ValueError('positive tile geometry required')
    lo,hi=map_points.min(axis=0),map_points.max(axis=0);tiles=[];qd=descriptor(query)
    for x in np.arange(lo[0],hi[0]+stride,stride):
        for y in np.arange(lo[1],hi[1]+stride,stride):
            tile=map_points[(abs(map_points[:,0]-x)<=width/2)&(abs(map_points[:,1]-y)<=width/2)]
            if len(tile)>=100:tiles.append((float(np.linalg.norm(descriptor(tile)-qd)),float(x),float(y),tile))
    candidates=[]
    for rank,(_,x,y,tile) in enumerate(sorted(tiles,key=lambda t:(t[0],t[1],t[2]))[:int(spec.get('top_k',8))]):
        start=time.monotonic();matcher=km.KISSMatcher(km.KISSMatcherConfig(float(spec.get('voxel_m',.3))))
        try:
            result=matcher.estimate(query.astype(float),tile.astype(float));rot=np.array(result.rotation,copy=True);trans=np.asarray(result.translation).reshape(3)
            q=Rotation.from_matrix(rot).as_quat()
            cmd=['ros2','run','agt_global_relocalization_native','map_gicp_tracker','--map',str(map_path),'--scan',str(directory/'query.pcd')]
            for key,value in zip(('x','y','z','qx','qy','qz','qw'),[*trans,*q]):cmd+=['--'+key,str(value)]
            refined=command(cmd,float(spec.get('timeout_sec',15)))
            refined.update(tile_center=[x,y],rank=rank,coarse_xyz=trans.tolist(),coarse_quaternion=q.tolist(),coarse_elapsed_sec=time.monotonic()-start,
                rotation_inliers=matcher.get_num_rotation_inliers(),final_inliers=matcher.get_num_final_inliers())
            candidates.append(refined)
        except Exception as exc:candidates.append(dict(success=False,reason=str(exc),rank=rank,tile_center=[x,y]))
    good=sorted((c for c in candidates if c.get('success') and c.get('fitness',float('inf'))<=spec.get('max_fitness',.3) and c.get('overlap',0)>=spec.get('min_overlap',.6)),key=lambda c:c.get('fitness',float('inf')))
    if not good:return dict(success=False,reason='quality_or_registration_failed',candidates=candidates)
    if len(good)>1 and abs(good[1]['fitness']-good[0]['fitness'])<.02 and math.hypot(good[1]['x']-good[0]['x'],good[1]['y']-good[0]['y'])>2:
        return dict(success=False,reason='ambiguous_repeated_rows',candidates=candidates)
    return dict(good[0],candidates=candidates,accept_reason='quality_and_ambiguity_gate')

def main():
    p=argparse.ArgumentParser();p.add_argument('--spec',required=True);p.add_argument('--output',required=True);a=p.parse_args()
    spec=json.loads(Path(a.spec).read_text());out=Path(a.output);out.mkdir(parents=True,exist_ok=False)
    if spec.get('query_frame')!='mapping_body':raise ValueError('explicit mapping_body query convention required')
    if not spec.get('calibration_sha256'):raise ValueError('common extrinsic SHA required')
    if not spec.get('queries'):raise ValueError('at least one real or explicitly synthetic query required')
    map_path=Path(spec['map']);map_sha=hash_file(map_path);mp=points(map_path);results=[]
    for i,item in enumerate(spec['queries']):
        qpath=Path(item['pcd']);sha=hash_file(qpath);query=points(qpath);d=out/str(i);d.mkdir();save_pcd(d/'query.pcd',query)
        baseline=command(['ros2','run','agt_global_relocalization_native','bbs_gicp_localizer','--map',str(map_path),'--scan',str(d/'query.pcd'),'--timeout',str(spec.get('timeout_sec',15))],float(spec.get('timeout_sec',15))+10)
        started=time.monotonic();before_self=resource.getrusage(resource.RUSAGE_SELF);before_children=resource.getrusage(resource.RUSAGE_CHILDREN)
        experimental=kiss(mp,query,map_path,d,spec)
        after_self=resource.getrusage(resource.RUSAGE_SELF);after_children=resource.getrusage(resource.RUSAGE_CHILDREN)
        experimental.update(elapsed_sec=time.monotonic()-started,cpu_sec=sum(getattr(after_self,k)+getattr(after_children,k)-getattr(before_self,k)-getattr(before_children,k) for k in ('ru_utime','ru_stime')))
        for backend,result in [('bbs',baseline),('kiss',experimental)]:
            result.update(backend=backend,query_sha256=sha,map_sha256=map_sha,calibration_sha256=spec['calibration_sha256'])
            truth=item.get('ground_truth')
            if truth and item.get('truth_evidence_sha256')==hash_file(item['truth_evidence']):
                if result.get('success'):
                    result['translation_error_m']=math.dist([result['x'],result['y'],result.get('z',0)],truth['xyz'])
                    yaw=math.atan2(2*(result['qw']*result['qz']+result['qx']*result['qy']),1-2*(result['qy']**2+result['qz']**2))
                    result['yaw_error_rad']=abs(math.atan2(math.sin(yaw-truth['yaw']),math.cos(yaw-truth['yaw'])))
                    result['false_acceptance']=bool(result['translation_error_m']>spec.get('truth_translation_limit_m',.5) or result['yaw_error_rad']>spec.get('truth_yaw_limit_rad',.17))
            else:result['accuracy_state']='NOT_RUN'
            results.append(result)
        if hash_file(qpath)!=sha:raise RuntimeError('query changed')
    if hash_file(map_path)!=map_sha:raise RuntimeError('map changed')
    write_json(out/'benchmark.json',dict(scope='OFFLINE_PCD_REGISTRATION',spec=spec,results=results,real_bag='NOT_RUN',tracker_ab='NOT_RUN',map_odom_continuity='NOT_RUN'))
if __name__=='__main__':main()
