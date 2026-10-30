#!/usr/bin/env python3
"""Same-query, isolated-process BBS vs SC/KISS/GICP comparison; no ROS publishers."""
import argparse
import json
import math
from pathlib import Path
import subprocess
import time
import sys
import os
import signal
import numpy as np
from scipy.spatial.transform import Rotation
REPO=Path(__file__).resolve().parents[1];sys.path.insert(0,str(REPO))
from workbench.maps import hash_file
from workbench.runtime import write_json


def run(command,timeout,directory):
    stats=directory/'resources.json';before=time.monotonic()
    # GNU time measures each fresh process (and waited-for descendants), avoiding
    # the cumulative ru_maxrss problem when comparing runs in one Python process.
    proc=None
    try:
        proc=subprocess.Popen(['/usr/bin/time','-f','{"cpu_user_sec":%U,"cpu_system_sec":%S,"max_rss_kib":%M}','-o',str(stats),*command],
            stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True,start_new_session=True)
        stdout,stderr=proc.communicate(timeout=timeout)
        rows=[line for line in stdout.splitlines() if line.startswith('{')]
        result=json.loads(rows[-1]) if rows else dict(success=False,message=stderr[-3000:])
        result['exit_code']=proc.returncode
    except subprocess.TimeoutExpired:result=dict(success=False,message='OFFLINE_TIMEOUT')
    finally:
        if proc is not None:
            try:os.killpg(proc.pid,signal.SIGTERM)
            except ProcessLookupError:pass
            try:proc.wait(timeout=1)
            except subprocess.TimeoutExpired:
                try:os.killpg(proc.pid,signal.SIGKILL)
                except ProcessLookupError:pass
                proc.wait(timeout=2)
    result['total_elapsed_sec']=time.monotonic()-before
    if stats.is_file():
        rows=[line for line in stats.read_text().splitlines() if line.startswith('{')]
        if rows:result.update(json.loads(rows[-1]))
    return result


def main():
    p=argparse.ArgumentParser();group=p.add_mutually_exclusive_group(required=True)
    group.add_argument('--spec');group.add_argument('--synthetic',action='store_true');p.add_argument('--output',required=True)
    a=p.parse_args();output=Path(a.output);output.mkdir(parents=True,exist_ok=False)
    if a.synthetic:
        sys.path.insert(0,str(REPO/'tests/sc_kiss'))
        from synthetic import fixture
        data=fixture(output/'synthetic_fixture')
        spec=dict(query_frame='mapping_body',map=str(data['map']),database=str(data['database']),lio_config=str(data['config']),
            queries=[dict(pcd=str(data['query']),roll=-.05,pitch=.08,truth_kind='SYNTHETIC_KNOWN_SE3',
                truth=dict(xyz=data['translation'].tolist(),yaw=float(data['rotation'].as_euler('xyz')[2])))])
    else:spec=json.loads(Path(a.spec).read_text())
    if spec.get('query_frame')!='mapping_body':raise ValueError('explicit mapping_body queries required')
    from agt_sc_kiss.geometry import contract_from_lio
    contract=contract_from_lio(spec['lio_config']);write_json(output/'contract.json',contract)
    write_json(output/'spec.json',spec)
    source_sha=hash_file(Path(spec['map']));results=[]
    from ament_index_python.packages import get_package_prefix
    native=Path(get_package_prefix('agt_global_relocalization_native'))/'lib/agt_global_relocalization_native/bbs_gicp_localizer'
    budget=float(spec.get('timeout_sec',30.))
    for i,item in enumerate(spec['queries']):
        query_sha=hash_file(Path(item['pcd']))
        for mode in ('bbs','sc_kiss'):
            directory=output/f'{i}_{mode}';directory.mkdir()
            command=([str(native),'--map',spec['map'],'--scan',item['pcd'],'--timeout',str(budget)] if mode=='bbs' else
                ['ros2','run','agt_sc_kiss','sc_kiss_localizer','--map',spec['map'],'--scan',item['pcd'],
                 '--database',spec['database'],'--contract',str(output/'contract.json'),'--query-roll',str(item['roll']),
                 '--query-pitch',str(item['pitch']),'--timeout',str(budget)])
            assets=spec.get('relocalization_assets')
            if mode=='bbs' and assets and (Path(assets)/'polar_context.db').is_file():
                command=[str(native.with_name('candidate_bbs_gicp_localizer')),'--map',spec['map'],'--scan',item['pcd'],
                    '--assets-dir',assets,'--timeout',str(budget),'--candidate-top-k','4','--descriptor-prefilter','80',
                    '--candidate-xy-radius','5.0','--candidate-z-radius','2.5','--candidate-yaw-range-deg','0.0',
                    '--roll-pitch-range-deg','0.0','--bbs-query-frame-mode','mapping_body',
                    '--per-candidate-timeout','6.0','--bbs-score-threshold','0.03','--gicp-max-corr','2.5']
            result=run(command,budget+5,directory)
            if mode=='bbs':result['backend']='polar_context_bbs_gicp' if assets and (Path(assets)/'polar_context.db').is_file() else 'whole_map_bbs_gicp'
            if mode=='bbs':
                result['coarse_elapsed_sec']=float(result['bbs_elapsed_ms'])/1000 if 'bbs_elapsed_ms' in result else 'NOT_REPORTED'
                result['gicp_elapsed_sec']='NOT_REPORTED_BY_EXISTING_EXECUTABLE'
            result.update(mode=mode,query_sha256=query_sha,map_sha256=source_sha,accuracy_state='NOT_RUN',false_acceptance='NOT_RUN')
            truth=item.get('truth')
            if item.get('truth_kind')=='EXTERNAL_SURVEY' and (not item.get('truth_evidence') or
                    hash_file(Path(item['truth_evidence']))!=item.get('truth_evidence_sha256')):truth=None
            if truth and item.get('truth_kind') in ('SYNTHETIC_KNOWN_SE3','EXTERNAL_SURVEY') and result.get('success'):
                r=Rotation.from_quat([result[k] for k in ('qx','qy','qz','qw')]);yaw=r.as_euler('xyz')[2]
                result.update(accuracy_state=item['truth_kind'],position_error_m=float(np.linalg.norm(np.array([result[k] for k in ('x','y','z')])-truth['xyz'])),
                    yaw_error_rad=abs(math.atan2(math.sin(yaw-truth['yaw']),math.cos(yaw-truth['yaw']))))
                result['false_acceptance']=result['position_error_m']>.5 or result['yaw_error_rad']>math.radians(10)
            results.append(result)
        if hash_file(Path(item['pcd']))!=query_sha:raise ValueError('query changed during comparison')
    if hash_file(Path(spec['map']))!=source_sha:raise ValueError('map changed during comparison')
    write_json(output/'comparison.json',dict(results=results,REAL_BAG_VERIFIED='NOT_RUN',HARDWARE_VERIFIED='NOT_RUN',
        memory_metric='GNU time maximum resident set; not aggregate simultaneous process-tree RAM'))
    print(json.dumps(results,indent=2))

if __name__=='__main__':main()
