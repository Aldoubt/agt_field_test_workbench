#!/usr/bin/env python3
"""Own one FAST-LIO-only session; finish live via SIGUSR1, cancel via SIGINT."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import shutil
import yaml

REPO=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(REPO))
from workbench.runtime import ros_call, write_json, check_bag
from workbench.maps import hash_file
from workbench.trajectory import inspect_trajectory

finish=False
cancel=False

def stop(p):
    if p and p.poll() is None:
        os.killpg(p.pid,signal.SIGINT)
        try:p.wait(timeout=15)
        except subprocess.TimeoutExpired:
            os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=5)

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--output',required=True);parser.add_argument('--config',required=True)
    parser.add_argument('--bag');parser.add_argument('--live',action='store_true');parser.add_argument('--livox-config')
    parser.add_argument('--reuse-sensors',action='store_true');parser.add_argument('--base-adapter',action='store_true')
    parser.add_argument('--imu-offset',type=float,default=0.);parser.add_argument('--lidar-offset',type=float,default=0.)
    args=parser.parse_args();output=Path(args.output).resolve();output.mkdir(parents=True,exist_ok=False)
    shutil.copyfile(args.config,output/'fastlio.yaml')
    config=yaml.safe_load((output/'fastlio.yaml').read_text());config['esti_il']=False
    (output/'fastlio.yaml').write_text(yaml.safe_dump(config,sort_keys=False))
    lidar,imu='/livox/lidar','/livox/imu';bag=None
    if not args.live:
        from agt_mapping_bringup.preflight import inspect_bag
        bag=inspect_bag(args.bag);lidar,imu=bag.lidar_topic,bag.imu_topic
    signature=lambda folder:{str(p.relative_to(Path(folder))):hash_file(p) for p in Path(folder).rglob('*') if p.is_file()}
    source_hashes=signature(bag.path) if bag else None
    launch=['ros2','launch','agt_field_test_bringup','no_pgo.launch.py',f'output:={output}',
            f'lio_config:={output}/fastlio.yaml',f'use_sim_time:={str(not args.live).lower()}',
            f'lidar_topic:={lidar}',f'imu_topic:={imu}',f'imu_offset_sec:={args.imu_offset}',f'lidar_offset_sec:={args.lidar_offset}']
    launch += ['base_adapter:='+str(args.base_adapter).lower()]
    if args.live and not args.reuse_sensors:launch += [f'livox_config:={Path(args.livox_config).resolve()}']
    children=[];handles=[]
    def spawn(cmd,label):
        h=(output/(label+'.log')).open('w');handles.append(h)
        p=subprocess.Popen(cmd,stdout=h,stderr=subprocess.STDOUT,start_new_session=True);children.append(p);return p
    try:
        recorder_name='field_raw_'+output.name.replace('-','_')
        recorder=spawn(['ros2','bag','record','--node-name',recorder_name,'-o',str(output/'raw_bag'),lidar,imu],'recorder') if args.live else None
        if args.live and args.reuse_sensors:
            deadline=time.monotonic()+20
            while True:
                if recorder.poll() is not None or cancel or time.monotonic()>deadline:raise RuntimeError('raw recorder did not subscribe before mapping')
                try:
                    info=ros_call(['ros2','node','info','/'+recorder_name],timeout=3)
                    if lidar+':' in info and imu+':' in info:break
                except (subprocess.CalledProcessError,subprocess.TimeoutExpired):pass
                time.sleep(.1)
        process=spawn(launch,'frontend')
        deadline=time.monotonic()+20
        while '/mapping/fastlio/export' not in ros_call(['ros2','service','list'],timeout=4):
            if cancel or process.poll() is not None or time.monotonic()>deadline: raise RuntimeError('no-PGO frontend failed startup')
        player=spawn(['ros2','bag','play',str(bag.path),'--clock','--topics',lidar,imu],'playback') if bag else None
        while not finish and not cancel:
            if process.poll() is not None: raise RuntimeError('frontend stopped unexpectedly')
            if player and player.poll() is not None:
                if player.returncode: raise RuntimeError('bag playback failed')
                break
            time.sleep(.1)
        if cancel: raise RuntimeError('cancelled; raw inputs retained, no accepted map')
        stop(recorder)
        time.sleep(1.)
        result=ros_call(['ros2','service','call','/mapping/fastlio/export','std_srvs/srv/Trigger','{}'],timeout=90)
        if 'success=True' not in result and 'success: true' not in result.lower(): raise RuntimeError('artifact export failed: '+result)
        if bag and signature(bag.path)!=source_hashes: raise RuntimeError('input bag changed')
        if recorder and check_bag(output/'raw_bag')['state']!='PASS': raise RuntimeError('raw bag incomplete')
        package=output/'map_package'
        quality=inspect_trajectory(package/'trajectory.csv')
        write_json(output/'trajectory_quality.json',quality)
        if quality['state']!='PASS':
            raise RuntimeError('LIO trajectory rejected before cloud/grid import: '+','.join(quality['reasons']))
        from ament_index_python.packages import get_package_prefix
        frontend_binary=Path(get_package_prefix('fastlio2'))/'lib/fastlio2/lio_node'
        alignment_binary=Path(get_package_prefix('agt_field_test_bringup'))/'lib/agt_field_test_bringup/time_align'
        manifest=dict(backend='FASTLIO_NO_PGO',pgo_enabled=False,frame_id='map',lio_world_frame=config['world_frame'],
            frame_convention='map numeric axes equal initial LIO world; no GNSS alignment assumed',
            input_bag=str(bag.path) if bag else str(output/'raw_bag'),input_sha256=source_hashes or signature(output/'raw_bag'),
            config_sha256=hash_file(output/'fastlio.yaml'),imu_offset_sec=args.imu_offset,lidar_offset_sec=args.lidar_offset,
            files={p.name:hash_file(p) for p in package.iterdir() if p.is_file()},status='DRAFT',
            trajectory_quality=quality,frontend_binary_sha256=hash_file(frontend_binary),
            alignment_binary_sha256=hash_file(alignment_binary))
        write_json(package/'manifest.json',manifest)
        print(json.dumps(manifest),flush=True)
    finally:
        for p in reversed(children):stop(p)
        for h in handles:h.close()

if __name__=='__main__':
    def request_finish(*_):
        global finish;finish=True
    def request_cancel(*_):
        global cancel;cancel=True
    signal.signal(signal.SIGUSR1,request_finish);signal.signal(signal.SIGINT,request_cancel);signal.signal(signal.SIGTERM,request_cancel)
    main()
