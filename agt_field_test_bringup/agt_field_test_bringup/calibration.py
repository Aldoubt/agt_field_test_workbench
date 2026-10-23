"""Separate mounting, backend internal extrinsics, lever arm and timestamp policy."""
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml
from .fwmini import quaternion, internal as validate_internal

CHECKS=('static','rotation','straight','figure_eight','repeatability','offline_replay','time_sync','geometry')

def digest(p):
    return hashlib.sha256(json.dumps({k:v for k,v in p.items() if k not in ('status','verification')},sort_keys=True).encode()).hexdigest()

def vector(v,n):
    if not isinstance(v,list) or len(v)!=n or any(isinstance(x,bool) or not isinstance(x,(int,float)) or not math.isfinite(x) for x in v):
        raise ValueError('finite calibration vector required')
    return v

def load(path,robot=None):
    p=yaml.safe_load(Path(path).read_text())
    if p.get('kind')!='robot_calibration_bundle' or p['robot'] not in ('bunker','fwmini') or (robot and p['robot']!=robot):
        raise ValueError('calibration bundle robot mismatch')
    if p['status'] not in ('CANDIDATE','VERIFIED'): raise ValueError('invalid calibration status')
    if (p['mount']['parent'],p['mount']['child'])!=('base_link','livox_frame'): raise ValueError('mount direction must be base_link -> livox_frame')
    vector(p['mount']['xyz'],3);vector(p['mount']['rpy'],3)
    for key in ('fastlio2','batch_lio'):
        item=p['internal'][key];validate_internal({'lidar_imu':{'rotation_row_major':vector(item['rotation'],9),'translation_m':vector(item['translation'],3)}})
    for key in ('imu_offset_sec','lidar_offset_sec'):
        vector([p['time_sync'][key]],1)
    if p['rtk_arm'].get('enabled'):
        vector(p['rtk_arm']['xyz'],3)
        if p['rtk_arm']['status']!='VERIFIED':raise ValueError('RTK lever arm must be verified before publishing')
    if p['status']=='VERIFIED':
        v=p['verification'];r=json.loads(Path(v['report']).read_text())
        if (r.get('calibration_sha256')!=digest(p) or r.get('robot')!=p['robot'] or
            r.get('sensor_id')!=p['sensor_id'] or not r.get('operator') or not r.get('observed_at') or not r.get('observations') or r.get('scope')!='PHYSICAL_CALIBRATION' or
            any(r.get('checks',{}).get(k)!='PASS' for k in CHECKS) or
            hashlib.sha256(Path(v['report']).read_bytes()).hexdigest()!=v['report_sha256']):
            raise ValueError('physical calibration evidence mismatch')
    return p

def description(p,backend='fastlio2'):
    root=ET.Element('robot',name=p['robot']+'_selected_calibration')
    for link in ('base_footprint','base_link','lidar_link','livox_frame','imu_link'):ET.SubElement(root,'link',name=link)
    def joint(parent,child,xyz,rpy):
        j=ET.SubElement(root,'joint',name=parent+'_to_'+child,type='fixed')
        ET.SubElement(j,'parent',link=parent);ET.SubElement(j,'child',link=child)
        ET.SubElement(j,'origin',xyz=' '.join(map(str,xyz)),rpy=' '.join(map(str,rpy)))
    joint('base_footprint','base_link',[0,0,p['base_link_height_m']],[0,0,0])
    joint('base_link','lidar_link',p['mount']['xyz'],p['mount']['rpy']);joint('lidar_link','livox_frame',[0,0,0],[0,0,0])
    # Physical IMU TF uses the explicit FAST internal interpretation; backend
    # configuration signs are preserved independently until measured.
    data=p['internal']['fastlio2'];r,t=data['rotation'],data['translation']
    invt=[-sum(r[j*3+i]*t[j] for j in range(3)) for i in range(3)]
    rpy=[math.atan2(r[5],r[8]),math.atan2(-r[2],math.hypot(r[0],r[1])),math.atan2(r[1],r[0])]
    joint('lidar_link','imu_link',invt,rpy)
    if p['rtk_arm'].get('enabled'):
        ET.SubElement(root,'link',name='rtk_antenna_link');joint('base_link','rtk_antenna_link',p['rtk_arm']['xyz'],[0,0,0])
    return ET.tostring(root,encoding='unicode')

def configs(p,runtime_share,mapping_source,directory):
    directory=Path(directory);directory.mkdir(parents=True,exist_ok=True)
    fast=yaml.safe_load((Path(runtime_share)/'config/fastlio2_mid360_navigation.yaml').read_text());fi=p['internal']['fastlio2']
    fast.update(r_il=fi['rotation'],t_il=fi['translation'],esti_il=False)
    batch=yaml.safe_load((Path(runtime_share)/'config/batch_lio_mid360.yaml').read_text());bi=p['internal']['batch_lio']
    batch['/**']['ros__parameters']['mapping'].update(extrinsic_R=bi['rotation'],extrinsic_T=bi['translation'],extrinsic_est_en=False)
    for name,data in (('fastlio.yaml',fast),('batch.yaml',batch)):(directory/name).write_text(yaml.safe_dump(data,sort_keys=False))
    return str(directory/'fastlio.yaml'),str(directory/'batch.yaml')
