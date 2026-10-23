"""GPS is an optional quality prior. It never owns TF or replaces LIO odometry."""
import math


def prior(fix,status,alignment,now,robot_pose=None,gate_innovation=True):
    reasons=[]
    for key in ('fix_stamp','status_stamp','imu_stamp'):
        age=now-status.get(key,float('-inf'))
        if not 0<=age<=.5:reasons.append(key+'_stale')
    if status.get('rtk_fixed') is not True:reasons.append('not_rtk_fixed')
    if not status.get('position_status_valid') or not status.get('position_std_valid'):reasons.append('invalid_solution_or_std')
    std=[status.get(k,float('inf')) for k in ('east_std','north_std','up_std')]
    if any(not math.isfinite(v) or v<0 or v>.15 for v in std):reasons.append('position_std_high')
    if not -90<=fix.get('latitude',float('nan'))<=90 or not -180<=fix.get('longitude',float('nan'))<=180 or fix.get('status',-1)<0 or not all(math.isfinite(fix.get(k,float('nan'))) for k in ('latitude','longitude','altitude')):reasons.append('invalid_fix')
    if alignment.get('status')!='VERIFIED' or alignment.get('lever_arm_status')!='VERIFIED':reasons.append('map_enu_or_lever_arm_unverified')
    if reasons:return dict(usable=False,weight=0.,reasons=reasons,stage='OBSERVATION_ONLY',map_prior=None)
    # WGS84 ECEF -> local ENU avoids using /ins/odom or /ins/velocity as truth.
    def ecef(lat,lon,h):
        lat,lon=math.radians(lat),math.radians(lon);a=6378137.;e2=6.69437999014e-3;n=a/math.sqrt(1-e2*math.sin(lat)**2)
        return [(n+h)*math.cos(lat)*math.cos(lon),(n+h)*math.cos(lat)*math.sin(lon),(n*(1-e2)+h)*math.sin(lat)]
    origin=alignment['origin_lla'];xyz=ecef(fix['latitude'],fix['longitude'],fix['altitude']);o=ecef(*origin)
    d=[a-b for a,b in zip(xyz,o)];la,lo=map(math.radians,origin[:2])
    east=-math.sin(lo)*d[0]+math.cos(lo)*d[1];north=-math.sin(la)*math.cos(lo)*d[0]-math.sin(la)*math.sin(lo)*d[1]+math.cos(la)*d[2]
    yaw=alignment['enu_to_map_yaw'];tx,ty=alignment['enu_to_map_xy'];c,s=math.cos(yaw),math.sin(yaw)
    x,y=c*east-s*north+tx,s*east+c*north+ty
    if robot_pose is None:return dict(usable=False,weight=0.,reasons=['body_heading_required_for_lever_arm'],map_prior=None)
    arm=alignment['lever_arm_base_xyz'];h=robot_pose['yaw'];x-=math.cos(h)*arm[0]-math.sin(h)*arm[1];y-=math.sin(h)*arm[0]+math.cos(h)*arm[1]
    if gate_innovation and math.hypot(x-robot_pose['x'],y-robot_pose['y'])>alignment.get('max_innovation_m',2.):
        return dict(usable=False,weight=0.,reasons=['gps_jump_rejected'],map_prior=None)
    return dict(usable=True,weight=1/max(std[0]**2+std[1]**2,.0001),reasons=[],map_prior={'x':x,'y':y,'std_m':max(std[:2])},stage='CANDIDATE_GATE_ONLY')


def load_alignment(path,map_sha256,calibration_sha256):
    import json,hashlib
    from pathlib import Path
    a=json.loads(Path(path).read_text());v=a.get('verification',{})
    digest=hashlib.sha256(json.dumps({k:x for k,x in a.items() if k not in ('status','verification')},sort_keys=True).encode()).hexdigest()
    report_path=Path(v.get('report',''));r=json.loads(report_path.read_text())
    if (a.get('status')!='VERIFIED' or a.get('map_sha256')!=map_sha256 or a.get('calibration_sha256')!=calibration_sha256 or
        r.get('scope')!='PHYSICAL_GEOREFERENCE' or r.get('alignment_sha256')!=digest or not r.get('operator') or not r.get('observed_at') or
        any(r.get('checks',{}).get(k)!='PASS' for k in ('map_origin','map_heading','lever_arm','fixed_solution','timestamps','repeatability')) or
        hashlib.sha256(report_path.read_bytes()).hexdigest()!=v.get('report_sha256')):
        raise ValueError('verified GPS map/lever-arm physical evidence required')
    for key,n in (('origin_lla',3),('enu_to_map_xy',2),('lever_arm_base_xyz',3)):
        if len(a.get(key,[]))!=n or not all(math.isfinite(x) for x in a[key]):raise ValueError('finite GPS alignment geometry required')
    if not math.isfinite(a['enu_to_map_yaw']) or not -90<=a['origin_lla'][0]<=90 or not -180<=a['origin_lla'][1]<=180:raise ValueError('invalid GPS origin/yaw')
    if not math.isfinite(a.get('max_innovation_m',2.)) or a.get('max_innovation_m',2.)<=0:raise ValueError('positive innovation gate required')
    if not math.isfinite(a.get('max_candidate_distance_m',3.)) or a.get('max_candidate_distance_m',3.)<=0:raise ValueError('positive candidate gate required')
    return a
