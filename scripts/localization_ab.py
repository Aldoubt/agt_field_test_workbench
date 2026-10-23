#!/usr/bin/env python3
"""Sequential tracker on/off replay; no chassis, sensors or external TF replay."""
import argparse,json,os,signal,subprocess,time
from pathlib import Path
import sys
REPO=Path(__file__).resolve().parents[1];ROOT=REPO.parent.parent;sys.path.insert(0,str(REPO))
from workbench.runtime import write_json
from workbench.maps import Maps,hash_file
from agt_mapping_bringup.preflight import inspect_bag

def stop(p):
    if p and p.poll() is None:
        os.killpg(p.pid,signal.SIGINT)
        try:p.wait(timeout=12)
        except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=5)
def main():
    p=argparse.ArgumentParser();p.add_argument('--bag',required=True);p.add_argument('--map-id',required=True);p.add_argument('--robot',choices=['bunker','fwmini'],required=True);p.add_argument('--calibration',required=True);p.add_argument('--output',required=True);a=p.parse_args()
    bag=inspect_bag(a.bag);m=Maps(ROOT).get(a.map_id);output=Path(a.output);output.mkdir(parents=True,exist_ok=False)
    if not m.get('source_pcd') or hash_file(m['source_pcd'])!=m['source_pcd_sha256']:raise ValueError('map PCD identity mismatch')
    signature=lambda:{str(f.relative_to(Path(bag.path))):hash_file(f) for f in Path(bag.path).rglob('*') if f.is_file()}
    inputs=signature();cal_sha=hash_file(a.calibration);results=[]
    for enabled in (False,True):
        run=output/('tracker_on' if enabled else 'tracker_off');run.mkdir();children=[];handles=[]
        env=dict(os.environ,ROS_DOMAIN_ID='91')
        def spawn(cmd,name):
            h=(run/(name+'.log')).open('w');handles.append(h);proc=subprocess.Popen(cmd,env=env,stdout=h,stderr=subprocess.STDOUT,start_new_session=True);children.append(proc);return proc
        try:
            args=['ros2','launch','agt_field_test_bringup','field_test.launch.py','robot:='+a.robot,'enable_chassis:=false','enable_mid360:=false','enable_localization:=true','use_sim_time:=true','lio_backend:=fastlio2','global_map:='+m['source_pcd'],'enable_map_tracking:='+str(enabled).lower()]
            if a.robot=='bunker':args+=['calibration_bundle:='+a.calibration]
            else:args+=['fwmini_mount_file:='+a.calibration,'fwmini_mode:=supervised_localization','supervised:=true']
            runtime=spawn(args,'runtime');time.sleep(3.)
            if runtime.poll() is not None:raise RuntimeError('localization runtime startup failed')
            recorder=spawn(['ros2','bag','record','-o',str(run/'observations'),'/tf','/tf_static','/agt/odometry/local','/agt/localization/status','/agt/localization/metrics','/agt/map_tracking/pose','/agt/map_tracking/status'],'record')
            player=spawn(['ros2','bag','play',str(bag.path),'--clock','--topics',bag.lidar_topic,bag.imu_topic],'play')
            last_request=0.
            while player.poll() is None:
                if runtime.poll() is not None:raise RuntimeError('runtime exited during playback')
                if time.monotonic()-last_request>8:
                    # Explicit requests are logged, and failed/not-ready calls are retained.
                    try:subprocess.run(['ros2','service','call','/agt/localization/relocalize','std_srvs/srv/Trigger','{}'],env=env,timeout=2,stdout=handles[-1],stderr=subprocess.STDOUT)
                    except subprocess.TimeoutExpired:pass
                    last_request=time.monotonic()
                time.sleep(.1)
            if player.returncode:raise RuntimeError('bag play failed')
            time.sleep(1.);stop(recorder)
            subprocess.run(['python3',str(REPO/'scripts/analyze_bag.py'),'--bag',str(run/'observations'),'--output',str(run/'analysis')],env=env,check=True)
            results.append(dict(tracker=enabled,state='PASS',scope='REPLAY_EXECUTION_ONLY',accuracy='NOT_RUN',analysis=str(run/'analysis/diagnostic.json')))
        finally:
            for proc in reversed(children):stop(proc)
            for h in handles:h.close()
    if signature()!=inputs or hash_file(a.calibration)!=cal_sha:raise RuntimeError('replay inputs changed')
    write_json(output/'ab.json',dict(results=results,source_bag_sha256=inputs,map_id=m['id'],map_pcd_sha256=m['source_pcd_sha256'],calibration_sha256=cal_sha,formal_benchmark='NOT_RUN',ground_truth='NOT_RUN'))
if __name__=='__main__':main()
