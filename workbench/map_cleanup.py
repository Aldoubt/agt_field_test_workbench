"""Preserve scan provenance and reuse AGT's existing world-voxel cleanup.

This is a map export operation. It never changes measurements fed to the LIO.
"""
import csv
import json
import math
from pathlib import Path

import numpy as np
import yaml

from .maps import hash_file
from .runtime import ros_call, write_json


def rotation_rpy(rpy):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]])


def base_from_body(bundle):
    # T_base_body = T_base_lidar * inverse(T_body_lidar).
    mount = bundle['mount']
    internal = bundle['internal']['fastlio2']
    rm = rotation_rpy(mount['rpy'])
    ri = np.array(internal['rotation'], dtype=float).reshape(3, 3)
    rotation = rm @ ri.T
    translation = np.array(mount['xyz'], dtype=float) - rotation @ np.array(internal['translation'])
    translation[2] += float(bundle['base_link_height_m'])
    rpy = [math.atan2(rotation[2, 1], rotation[2, 2]),
           math.atan2(-rotation[2, 0], math.hypot(rotation[0, 0], rotation[1, 0])),
           math.atan2(rotation[1, 0], rotation[0, 0])]
    return dict(xyz=translation.tolist(), rpy=rpy)


def quaternion_matrix(q):
    q = np.asarray(q, dtype=float)
    norm = np.linalg.norm(q)
    if not np.isfinite(norm) or not .99 < norm < 1.01:
        raise ValueError('invalid trajectory quaternion')
    x, y, z, w = q / norm
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def write_pcd(path, xyz):
    xyz = np.asarray(xyz, dtype='<f4')
    with Path(path).open('wb') as stream:
        stream.write(('VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n'
                      f'WIDTH {len(xyz)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(xyz)}\nDATA binary\n').encode())
        stream.write(xyz.tobytes())


def prepare_keyframes(artifact, destination):
    """Reconstruct body-frame patches with an exact cloud/pose timestamp join.

    Retain EVERY recorded world scan; never guess scan boundaries in a fused PCD.
    """
    artifact, destination = Path(artifact), Path(destination)
    with (artifact/'frames.csv').open() as stream:
        frames = list(csv.DictReader(stream))
    with (artifact/'trajectory.csv').open() as stream:
        poses = list(csv.DictReader(stream))
    by_stamp = {row['stamp']: row for row in poses}
    if len(frames) < 2 or len(by_stamp) != len(poses):
        raise ValueError('empty frames or duplicate trajectory timestamp')
    raw_path = artifact/'points.raw'
    if raw_path.stat().st_size % 12:
        raise ValueError('invalid XYZ spool size')
    raw = np.memmap(raw_path, dtype='<f4', mode='r').reshape(-1, 3)
    cursor, previous_ns = 0, None
    matched = []
    for row in frames:
        first, count, stamp_ns = int(row['first_point']), int(row['point_count']), int(row['stamp_ns'])
        if first != cursor or count <= 0 or first+count > len(raw):
            raise ValueError('non-contiguous or invalid frame index')
        if previous_ns is not None and stamp_ns <= previous_ns:
            raise ValueError('non-monotonic cloud timestamps')
        pose = by_stamp.get(row['stamp'])
        if pose is None:
            raise ValueError('cloud has no exact timestamp-matched pose: '+row['stamp'])
        values = [float(pose[k]) for k in ('x','y','z','qx','qy','qz','qw')]
        if not np.isfinite(values).all():
            raise ValueError('non-finite trajectory')
        quaternion_matrix(values[3:])
        matched.append((row, values))
        cursor, previous_ns = first+count, stamp_ns
    if cursor != len(raw):
        raise ValueError('frame index does not cover the entire spool')
    destination.mkdir(parents=True, exist_ok=False)
    (destination/'patches').mkdir()
    with (destination/'poses_timed.txt').open('w') as stream:
        for index, (row, v) in enumerate(matched):
            start, count = int(row['first_point']), int(row['point_count'])
            world = raw[start:start+count].astype(np.float64)
            if not np.isfinite(world).all():
                raise ValueError('non-finite spool point')
            body = (world - np.array(v[:3])) @ quaternion_matrix(v[3:])
            name = f'{index:06d}.pcd'
            write_pcd(destination/'patches'/name, body)
            x,y,z,qx,qy,qz,qw = v
            stream.write(f'{name} {row["stamp"]} {x:.17g} {y:.17g} {z:.17g} {qw:.17g} {qx:.17g} {qy:.17g} {qz:.17g}\n')
    stamps = np.array([int(row['stamp_ns']) for row in frames], dtype=np.int64)
    info = dict(frames=len(frames), points=len(raw), exact_pose_matches=len(matched),
                median_scan_interval_sec=float(np.median(np.diff(stamps)))*1e-9,
                inputs={p.name: hash_file(p) for p in (artifact/'frames.csv', artifact/'trajectory.csv', raw_path)})
    write_json(destination/'scan_provenance.json', info)
    return info


def clean_mapping(artifact, workspace, bundle_path, runner=ros_call):
    artifact, workspace, bundle_path = Path(artifact), Path(workspace), Path(bundle_path)
    destination = artifact/'temporal_cleanup'
    if destination.exists():
        raise ValueError('cleanup output already exists; retain immutable result')
    bundle = yaml.safe_load(bundle_path.read_text())
    if bundle.get('robot') != 'bunker' or bundle.get('kind') != 'robot_calibration_bundle':
        raise ValueError('cleanup requires the selected Bunker calibration bundle')
    info = prepare_keyframes(artifact, destination/'keyframes')
    defaults = workspace/'src/agt-lio-pgo-mapping/exporters/agt_pcd2grid_exporter/config/projection_traversability.yaml'
    config = yaml.safe_load(defaults.read_text())
    config['robot']['base_from_body'] = base_from_body(bundle)
    # The existing exporter counted separated PGO keyframes. Raw LIO scans are
    # 10 Hz: require >=1 second of support instead of only two adjacent scans.
    config['temporal_filter'].update(enabled=True, min_observations=3,
        min_keyframe_span=max(2, math.ceil(1.0/info['median_scan_interval_sec'])))
    snapshot = destination/'projection.yaml'
    snapshot.write_text(yaml.safe_dump(config, sort_keys=False))
    runner(['ros2','run','agt_pcd2grid_exporter','pcd2grid_exporter',
            '--package',str(destination/'keyframes'), '--config',str(snapshot),
            '--output',str(destination/'navigation'), '--cloud-output',str(destination/'clouds')], timeout=3600)
    cleaned = destination/'clouds/static_map.pcd'
    removed = destination/'clouds/removed_candidates.pcd'
    metrics = yaml.safe_load((destination/'clouds/cloud_cleanup.yaml').read_text())
    if metrics['retained_points'] + metrics['removed_candidate_points'] + metrics['self_filtered_points'] != info['points']:
        raise ValueError('cleanup point accounting mismatch')
    result = dict(state='CANDIDATE', method='agt_traversability_world_voxel_persistence_and_near_field_evidence',
                  input_map=str(artifact/'map_package/map.pcd'),
                  input_map_sha256=hash_file(artifact/'map_package/map.pcd'),
                  cleaned_pcd=str(cleaned), cleaned_sha256=hash_file(cleaned),
                  removed_pcd=str(removed), removed_sha256=hash_file(removed),
                  calibration_bundle=str(bundle_path), calibration_sha256=hash_file(bundle_path),
                  config=str(snapshot), config_sha256=hash_file(snapshot), scan_provenance=info,
                  metrics=metrics, semantic_person_detection=False, physical_geometry='NOT_VERIFIED')
    write_json(destination/'manifest.json', result)
    return result
