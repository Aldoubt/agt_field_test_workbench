"""FW-mini mount evidence, mode gates and the single robot-description TF tree."""
import copy
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml

MOUNT_CHECKS = ('mount_direction', 'rotation_center', 'repeatability', 'geometry', 'tf_ownership')
CANDIDATE_MODES = ('sensor_test', 'calibration_recording', 'sensor_mapping', 'offline_mapping', 'supervised_localization')


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def digest(profile):
    content = {k: v for k, v in profile.items() if k not in ('status', 'validation', 'verification')}
    return hashlib.sha256(json.dumps(content, sort_keys=True).encode()).hexdigest()


def vector(value, size, name):
    if not isinstance(value, list) or len(value) != size or any(
            isinstance(v, bool) or not isinstance(v, (float, int)) or not math.isfinite(v) for v in value):
        raise ValueError(f'{name} requires {size} finite numbers')
    return [float(v) for v in value]


def internal(profile, config=None):
    data = profile['lidar_imu']
    if config:
        data = yaml.safe_load(Path(config).read_text())
        data = {'rotation_row_major': data['r_il'], 'translation_m': data['t_il']}
    r = vector(data['rotation_row_major'], 9, 'LiDAR/IMU rotation')
    t = vector(data['translation_m'], 3, 'LiDAR/IMU translation')
    if any(abs(sum(r[k*3+i]*r[k*3+j] for k in range(3)) - (i == j)) > 1e-6
           for i in range(3) for j in range(3)):
        raise ValueError('LiDAR/IMU rotation must be orthonormal')
    det = r[0]*(r[4]*r[8]-r[5]*r[7])-r[1]*(r[3]*r[8]-r[5]*r[6])+r[2]*(r[3]*r[7]-r[4]*r[6])
    if abs(det-1) > 1e-6:
        raise ValueError('LiDAR/IMU rotation must have determinant +1')
    return r, t


def validate(profile):
    if profile.get('robot') != 'fwmini' or profile.get('status') not in ('CALIBRATION_CANDIDATE', 'VERIFIED'):
        raise ValueError('a FW-mini candidate or verified mount is required')
    mount = profile['mount']
    if (mount['parent_frame'], mount['child_frame']) != ('base_link', 'livox_frame'):
        raise ValueError('mount must be T_base_link_livox_frame, not its inverse')
    vector(mount['translation_m'], 3, 'mount translation')
    vector(mount['rotation_rpy_rad'], 3, 'mount RPY')
    if not profile.get('source') or not profile.get('calibration_date') or not profile.get('assumptions'):
        raise ValueError('mount provenance, date and assumptions are required')
    for key in ('length_m', 'width_m', 'height_m', 'wheelbase_m', 'track_m'):
        value = profile['body'][key]
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            raise ValueError(f'invalid body.{key}')
    height = profile['base_footprint_to_base_z_m']
    if isinstance(height, bool) or not isinstance(height, (int, float)) or not math.isfinite(height) or height < 0:
        raise ValueError('invalid base_link height')
    internal(profile)
    return profile


def check_report(profile, report_path, sensor_id):
    report_path = Path(report_path).expanduser().resolve()
    report = json.loads(report_path.read_text())
    if (report.get('robot') != 'fwmini' or report.get('verification_scope') != 'PHYSICAL_MOUNT'
            or not sensor_id or report.get('sensor_id') != sensor_id
            or report.get('calibration_sha256') != digest(profile)):
        raise ValueError('PHYSICAL_MOUNT report must bind FW-mini, sensor identity and calibration_sha256')
    if any(report.get('checks', {}).get(name) != 'PASS' for name in MOUNT_CHECKS):
        raise ValueError('mount_direction, rotation_center, repeatability, geometry and tf_ownership must PASS')
    if not report.get('observations') or not report.get('operator') or not report.get('observed_at'):
        raise ValueError('report requires operator, observed_at and actual observations')
    return report_path, report


def load_profile(path):
    profile = validate(yaml.safe_load(Path(path).expanduser().read_text()))
    if profile['status'] == 'VERIFIED':
        verification = profile['verification']
        report_path, _ = check_report(profile, verification['report'], verification['sensor_id'])
        if sha256(report_path) != verification['report_sha256']:
            raise ValueError('verification report SHA-256 changed')
    return profile


def verify_profile(profile_path, report_path, sensor_id, output):
    profile = copy.deepcopy(load_profile(profile_path))
    report_path, report = check_report(profile, report_path, sensor_id)
    output = Path(output)
    if output.exists():
        raise ValueError('verified version already exists')
    profile['status'] = 'VERIFIED'
    profile['validation'].update({k: report['checks'][k] for k in MOUNT_CHECKS})
    profile['verification'] = {'report': str(report_path), 'report_sha256': sha256(report_path),
                               'sensor_id': sensor_id, 'source_file_sha256': sha256(profile_path)}
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open('x') as stream:
        yaml.safe_dump(profile, stream, sort_keys=False, allow_unicode=True)
    return output


def require_mode(profile, mode, supervised=False):
    validate(profile)
    if mode not in (*CANDIDATE_MODES, 'autonomous_navigation'):
        raise ValueError('unknown FW-mini operation mode')
    if mode == 'supervised_localization' and not supervised:
        raise ValueError('localization observation requires supervised:=true')
    if mode == 'autonomous_navigation':
        if profile['status'] != 'VERIFIED' or any(profile.get('validation', {}).get(k) != 'PASS'
                for k in (*MOUNT_CHECKS, 'localization', 'motion_safety')):
            raise ValueError('formal navigation requires VERIFIED mount plus localization, geometry and motion safety PASS')


def quaternion(rpy):
    r, p, y = (v/2 for v in rpy)
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return [sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy]


def edges(profile, config=None):
    validate(profile)
    r, t = internal(profile, config)
    inv_t = [-sum(r[j*3+i]*t[j] for j in range(3)) for i in range(3)]
    # R_lidar_imu = R_imu_lidar^T; URDF fixed-axis roll/pitch/yaw.
    inv_rpy = [math.atan2(r[5], r[8]), math.atan2(-r[2], math.hypot(r[0], r[1])), math.atan2(r[1], r[0])]
    mount = profile['mount']
    return {
        'base_link': ('base_footprint', [0, 0, profile['base_footprint_to_base_z_m']], [0, 0, 0]),
        'lidar_link': ('base_link', mount['translation_m'], mount['rotation_rpy_rad']),
        'livox_frame': ('lidar_link', [0, 0, 0], [0, 0, 0]),
        'imu_link': ('lidar_link', inv_t, inv_rpy),
    }


def robot_description(profile, config=None):
    root = ET.Element('robot', name='fwmini_field_test')
    ET.SubElement(root, 'link', name='base_footprint')
    for child, (parent, xyz, rpy) in edges(profile, config).items():
        link = ET.SubElement(root, 'link', name=child)
        if child == 'base_link':
            visual = ET.SubElement(link, 'visual')
            ET.SubElement(visual, 'origin', xyz=f"0 0 {profile['body']['height_m']/2}")
            geometry = ET.SubElement(visual, 'geometry')
            ET.SubElement(geometry, 'box', size=' '.join(str(profile['body'][k]) for k in ('length_m', 'width_m', 'height_m')))
        joint = ET.SubElement(root, 'joint', name=parent+'_to_'+child, type='fixed')
        ET.SubElement(joint, 'parent', link=parent)
        ET.SubElement(joint, 'child', link=child)
        ET.SubElement(joint, 'origin', xyz=' '.join(map(str, xyz)), rpy=' '.join(map(str, rpy)))
    return ET.tostring(root, encoding='unicode')


def lio_configs(profile, runtime_share, output):
    """One internal T_imu_lidar for both backends, separate from mount TF."""
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    r, t = internal(profile)
    fast = yaml.safe_load((Path(runtime_share)/'config/fastlio2_mid360_navigation.yaml').read_text())
    fast.update(r_il=r, t_il=t, esti_il=False)
    batch = yaml.safe_load((Path(runtime_share)/'config/batch_lio_mid360.yaml').read_text())
    batch['/**']['ros__parameters']['mapping'].update(extrinsic_R=r, extrinsic_T=t, extrinsic_est_en=False)
    for name, data in (('fastlio.yaml', fast), ('batch.yaml', batch)):
        (output/name).write_text(yaml.safe_dump(data, sort_keys=False))
    (output/'mount.json').write_text(json.dumps({'calibration_sha256': digest(profile), 'mount': profile['mount']}, indent=2))
    return str(output/'fastlio.yaml'), str(output/'batch.yaml')
