#!/usr/bin/env python3
"""Read-only motion/time diagnostics on recorded messages, never on live CAN."""
import argparse
import json
import math
from pathlib import Path
import sys
import numpy as np
from scipy.signal import correlate
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
REPO=Path(__file__).resolve().parents[1];sys.path.insert(0,str(REPO))
from workbench.runtime import write_json
from workbench.maps import hash_file


def analyze(series):
    result={'series':series,'metrics':{},'classification':[],'scope':'RECORDED_TELEMETRY_ONLY'}
    metrics=result['metrics'];timing={}
    for topic,rows in series.items():
        t=np.array([r['t'] for r in rows]);dt=np.diff(t)
        timing[topic]=dict(samples=len(rows),gaps_over_100ms=int(sum(dt>.1)),backwards=int(sum(dt<0)),
            repeats=int(sum(dt==0)),max_gap_sec=float(dt.max()) if len(dt) else None,
            median_receive_minus_header_sec=float(np.median([r['receive_t']-r['t'] for r in rows])))
    result['timing']=timing
    # Retain original order for timing evidence; interpolation uses unique ascending stamps.
    series={topic:sorted({row['t']:row for row in rows}.values(),key=lambda row:row['t']) for topic,rows in series.items()}
    cmd=series.get('/mux/cmd_vel') or series.get('/fwmini/cmd_vel_guarded') or series.get('/cmd_vel',[])
    lio=series.get('/agt/odometry/local',[]);wheel=series.get('/wheel/odom',[])
    corrections=[dict(t=row['t'],**tf) for row in series.get('/tf',[]) for tf in row.get('map_odom',[])]
    result['map_odom_corrections']=corrections
    if len(corrections)>1:
        metrics['max_map_odom_translation_step_m']=max(math.hypot(b['x']-a['x'],b['y']-a['y']) for a,b in zip(corrections,corrections[1:]))
        ys=np.unwrap([math.atan2(2*r['qw']*r['qz'],1-2*r['qz']**2) for r in corrections]);metrics['max_map_odom_yaw_step_rad']=float(max(abs(np.diff(ys))))
    else:metrics['max_map_odom_translation_step_m']=metrics['max_map_odom_yaw_step_rad']=None
    if cmd and lio:
        lo=max(cmd[0]['t'],lio[0]['t']);hi=min(cmd[-1]['t'],lio[-1]['t'])
        step=max(.02,(hi-lo)/100000);grid=np.arange(lo,hi,step)
        if len(grid)>=50:
            interp=lambda rows,key:np.interp(grid,[r['t'] for r in rows],[r.get(key,0.) for r in rows])
            requested=interp(cmd,'wz');yaw=np.unwrap([r['yaw'] for r in lio]);rates=np.gradient(yaw,[r['t'] for r in lio]) if len(lio)>1 else [0.]
            for row,rate in zip(lio,rates):row['raw_twist_wz']=row.get('wz');row['wz']=float(rate)
            measured=np.interp(grid,[r['t'] for r in lio],rates);a=requested-requested.mean();b=measured-measured.mean()
            if np.std(a)>1e-4 and np.std(b)>1e-4:
                corr=correlate(b,a,mode='full',method='fft');lags=np.arange(-len(a)+1,len(a))*step;valid=abs(lags)<=2.
                metrics['angular_response_delay_sec']=float(lags[valid][np.argmax(corr[valid])])
            else:metrics['angular_response_delay_sec']=None
            freq=np.fft.rfftfreq(len(grid),step);power=abs(np.fft.rfft(b))**2;mask=(freq>=.1)&(freq<=5.)
            metrics['oscillation_hz']=float(freq[mask][np.argmax(power[mask])]) if np.std(b)>1e-4 and mask.any() else None
            metrics['angular_rmse_radps']=float(np.sqrt(np.mean((requested-measured)**2)))
            if wheel:
                wy=np.unwrap([r['yaw'] for r in wheel]);ly=np.unwrap([r['yaw'] for r in lio])
                diff=np.interp(grid,[r['t'] for r in wheel],wy)-np.interp(grid,[r['t'] for r in lio],ly)
                metrics['wheel_lio_heading_change_rmse_rad']=float(np.sqrt(np.mean((diff-diff[0])**2)))
    path=series.get('/plan',[])
    if path and lio:
        errors=[]
        # Only compare map-frame poses with a map-frame path; never compare odom directly.
        map_poses=series.get('/field/map_pose',[])
        for r in map_poses:
            candidates=[p for p in path if p['t']<=r['t'] and r['t']-p['t']<5 and p.get('frame')=='map']
            if not candidates:continue
            pts=np.asarray(candidates[-1]['points']);point=np.array([r['x'],r['y']])
            if len(pts)<2:continue
            a,b=pts[:-1],pts[1:];ab=b-a;u=np.clip(np.sum((point-a)*ab,axis=1)/np.maximum(np.sum(ab*ab,axis=1),1e-12),0,1)
            errors.append(float(np.min(np.linalg.norm(point-(a+u[:,None]*ab),axis=1))))
        metrics['max_lateral_error_m']=max(errors) if errors else None
    metrics.setdefault('max_lateral_error_m',None)
    result['missing_metrics']=[k for k,v in metrics.items() if v is None]
    if any(v['backwards'] or v['max_gap_sec'] and v['max_gap_sec']>.5 for v in timing.values()):result['classification'].append('TIME_OR_DATA_GAPS: inspect timestamps before controller tuning')
    if metrics.get('wheel_lio_heading_change_rmse_rad',0)>.1:result['classification'].append('WHEEL_LIO_DISAGREEMENT: inspect chassis, slip and TF')
    if metrics.get('angular_rmse_radps',0)>.1:result['classification'].append('COMMAND_RESPONSE_DISAGREEMENT: inspect controller versus measured motion')
    result['series']={topic:rows[::max(1,len(rows)//5000)] for topic,rows in series.items()}
    result['next_tests']=['stationary','supervised_manual_straight','supervised_manual_turn','Nav2_straight','Nav2_arc']
    return result


def main():
    p=argparse.ArgumentParser();p.add_argument('--bag',required=True);p.add_argument('--output',required=True);args=p.parse_args()
    bag=Path(args.bag).resolve();output=Path(args.output);output.mkdir(parents=True,exist_ok=False)
    source={str(f.relative_to(bag)):hash_file(f) for f in bag.rglob('*') if f.is_file()}
    reader=rosbag2_py.SequentialReader();reader.open(rosbag2_py.StorageOptions(uri=str(bag)),rosbag2_py.ConverterOptions('',''))
    types={t.name:t.type for t in reader.get_all_topics_and_types()};series={}
    while reader.has_next():
        topic,data,recv=reader.read_next();typ=types[topic]
        if not any(token in typ for token in ('Twist','Odometry','PoseStamped','PoseWithCovarianceStamped','TFMessage','Path','Imu','CustomMsg','LocalizationStatus','LocalizationMetrics','NavSatFix','INSStatus','BunkerStatus','CtrlCmd','String')):continue
        msg=deserialize_message(data,get_message(typ));header=getattr(msg,'header',None)
        stamp=header.stamp if header else getattr(msg,'stamp',None)
        t=stamp.sec+stamp.nanosec/1e9 if stamp else recv/1e9
        row=dict(t=t,receive_t=recv/1e9)
        if typ.endswith('/Twist') or typ.endswith('/TwistStamped'):
            v=msg.twist if hasattr(msg,'twist') else msg;row.update(vx=v.linear.x,vy=v.linear.y,wz=v.angular.z)
        elif typ.endswith('/Odometry'):
            q=msg.pose.pose.orientation;pos=msg.pose.pose.position
            row.update(x=pos.x,y=pos.y,yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)),wz=msg.twist.twist.angular.z,frame=msg.header.frame_id)
        elif typ.endswith('/PoseStamped') or typ.endswith('/PoseWithCovarianceStamped'):
            p=msg.pose.pose if hasattr(msg.pose,'pose') else msg.pose;q=p.orientation
            row.update(x=p.position.x,y=p.position.y,yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)),frame=msg.header.frame_id)
        elif typ.endswith('/Path'):row.update(frame=msg.header.frame_id,points=[[p.pose.position.x,p.pose.position.y] for p in msg.poses])
        elif typ.endswith('/TFMessage'):
            row['map_odom']=[dict(x=x.transform.translation.x,y=x.transform.translation.y,qz=x.transform.rotation.z,qw=x.transform.rotation.w) for x in msg.transforms if x.header.frame_id=='map' and x.child_frame_id=='odom']
        elif typ.endswith('/LocalizationMetrics'):
            row['manager']={k:getattr(msg,k) for k in ('state','tracking_health','reason','correction_translation_delta','correction_yaw_delta','tracking_innovation_translation','tracking_innovation_yaw')}
        elif typ.endswith('/LocalizationStatus'):
            row['manager']={k:getattr(msg,k) for k in ('state','reason','local_odom_fresh','global_correction_valid')}
        elif typ.endswith('/INSStatus'):
            row['quality']={k:getattr(msg,k) for k in ('rtk_fixed','position_status_valid','position_std_valid','east_std','north_std','up_std','position_type')}
        elif typ.endswith('/String'):row['data']=msg.data
        series.setdefault(topic,[]).append(row)
    result=analyze(series);result.update(input_bag=str(bag),input_sha256=source,state='PASS' if series else 'NOT_RUN',
        physical_accuracy='NOT_RUN',tracker_candidates=series.get('/agt/map_tracking/pose',[]))
    write_json(output/'diagnostic.json',result)
    if source!={str(f.relative_to(bag)):hash_file(f) for f in bag.rglob('*') if f.is_file()}:raise RuntimeError('input bag changed')
if __name__=='__main__':main()
