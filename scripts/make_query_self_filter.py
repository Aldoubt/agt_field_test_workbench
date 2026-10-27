#!/usr/bin/env python3
"""Export an explicit localization-mask candidate from the selected field inputs.

This reads the workspace configuration and writes only a new output JSON. It does
not enable the mask, modify a robot profile, build assets, or start ROS nodes.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import sys

import numpy as np
from scipy.spatial.transform import Rotation
import yaml

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / 'agt_field_test_bringup'))
from agt_field_test_bringup.relocalization_self_filter import (
    make_self_filter_config, normalize_self_filter,
)


def _vector(value, length, name):
    if (not isinstance(value, list) or len(value) != length
            or any(isinstance(v, bool) or not isinstance(v, (int, float)) for v in value)):
        raise ValueError('invalid calibration ' + name)
    result = np.asarray(value, dtype=float)
    if not np.isfinite(result).all():
        raise ValueError('non-finite calibration ' + name)
    return result


def derive_base_from_body(bundle):
    """T_base_body = T_base_lidar * inverse(T_body_lidar), FAST-LIO field path.

    base_link_height_m is intentionally absent. That belongs to the distinct
    base_footprint -> base_link edge, which is not part of this transform.
    """
    if not isinstance(bundle, dict) or bundle.get('kind') != 'robot_calibration_bundle':
        raise ValueError('selected calibration must be a robot_calibration_bundle')
    mount = bundle.get('mount', {})
    if (mount.get('parent'), mount.get('child')) != ('base_link', 'livox_frame'):
        raise ValueError('selected mount must be base_link -> livox_frame')
    internal = bundle.get('internal', {}).get('fastlio2', {})
    rm = Rotation.from_euler('xyz', _vector(mount.get('rpy'), 3, 'mount.rpy')).as_matrix()
    tm = _vector(mount.get('xyz'), 3, 'mount.xyz')
    ri = _vector(internal.get('rotation'), 9, 'internal.fastlio2.rotation').reshape((3, 3))
    ti = _vector(internal.get('translation'), 3, 'internal.fastlio2.translation')
    if (not np.allclose(ri.T @ ri, np.eye(3), atol=1e-6, rtol=0)
            or not np.isclose(np.linalg.det(ri), 1., atol=1e-6, rtol=0)):
        raise ValueError('internal.fastlio2.rotation must be proper orthonormal')
    rotation = rm @ ri.T
    return rotation, tm - rotation @ ti


def _resolve(workspace, value):
    path = Path(value).expanduser()
    return (path if path.is_absolute() else workspace / path).resolve()


def _read_yaml(path):
    data = path.read_bytes()
    value = yaml.safe_load(data)
    if not isinstance(value, dict):
        raise ValueError('expected a YAML mapping: ' + str(path))
    return value, dict(path=str(path), sha256=hashlib.sha256(data).hexdigest())


def generate(workspace, config, output, *, use_navigation_boxes=False, boxes_file=None):
    if bool(use_navigation_boxes) == bool(boxes_file):
        raise ValueError('choose exactly one of --use-navigation-boxes or --boxes-file')
    workspace = Path(workspace).expanduser().resolve()
    config_path = _resolve(workspace, config or 'config/field_trial.yaml')
    selected, config_source = _read_yaml(config_path)
    for key in ('robot_profile', 'calibration_bundle'):
        if not isinstance(selected.get(key), str) or not selected[key].strip():
            raise ValueError('field configuration requires ' + key)
    profile, profile_source = _read_yaml(_resolve(workspace, selected['robot_profile']))
    bundle, calibration_source = _read_yaml(_resolve(workspace, selected['calibration_bundle']))
    if (profile.get('robot') not in ('bunker', 'fwmini')
            or profile.get('robot') != bundle.get('robot')):
        raise ValueError('robot profile and calibration identities differ')
    if profile.get('body_frame', 'base_link') != 'base_link':
        raise ValueError('selected profile body_frame must be base_link')
    if use_navigation_boxes:
        boxes = profile.get('navigation_self_mask_boxes', [])
        box_source = dict(kind='navigation_self_mask_boxes', **profile_source)
    else:
        box_path = _resolve(workspace, boxes_file)
        data = box_path.read_bytes()
        loaded = json.loads(data)
        boxes = loaded.get('boxes') if isinstance(loaded, dict) else loaded
        box_source = dict(kind='explicit_boxes_file', path=str(box_path),
                          sha256=hashlib.sha256(data).hexdigest())
    # A candidate generator should not produce a deceptively empty experiment.
    if not isinstance(boxes, list) or not boxes:
        raise ValueError('at least one nonempty localization self-mask box is required')
    rotation, translation = derive_base_from_body(bundle)
    value = make_self_filter_config(boxes, translation.tolist(),
                                    Rotation.from_matrix(rotation).as_quat().tolist())
    normalize_self_filter(value)
    value['provenance'] = dict(
        state='CANDIDATE_NOT_FIELD_VERIFIED', automatic_activation=False,
        robot=profile['robot'], lio_backend='fastlio2',
        field_config=config_source, robot_profile=profile_source,
        calibration_bundle=calibration_source, boxes_source=box_source,
        transform='T_base_lidar * inverse(T_body_lidar); base_link_height_m excluded',
    )
    destination = Path(output).expanduser()
    if not destination.is_absolute():
        destination = workspace / destination
    # Do not follow a pre-existing output symlink, including a dangling one.
    if destination.exists() or destination.is_symlink():
        raise FileExistsError('output exists; choose a new candidate path: ' + str(destination))
    encoded = json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False) + '\n'
    destination.parent.mkdir(parents=True, exist_ok=True)
    with destination.open('x', encoding='utf-8') as stream:
        stream.write(encoded)
    return dict(output=str(destination.resolve()), boxes=len(boxes) // 6,
                sha256=hashlib.sha256(encoded.encode('utf-8')).hexdigest(),
                enabled=False, state='CANDIDATE_NOT_FIELD_VERIFIED')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace', type=Path,
        default=Path(os.environ.get('AGT_FIELD_TEST_WS', str(REPO.parent.parent))),
        help='field workspace; relative input/output paths are resolved here')
    parser.add_argument('--config', type=Path, default=Path('config/field_trial.yaml'),
        help='selected field YAML, absolute or relative to --workspace')
    choice = parser.add_mutually_exclusive_group(required=True)
    choice.add_argument('--use-navigation-boxes', action='store_true',
        help='explicitly copy the selected profile navigation_self_mask_boxes as a candidate')
    choice.add_argument('--boxes-file', type=Path,
        help='JSON flat box list or {"boxes": [...]} in base_link')
    parser.add_argument('--output', type=Path, required=True,
        help='new JSON path; existing files are never overwritten')
    args = parser.parse_args(argv)
    try:
        result = generate(args.workspace, args.config, args.output,
                          use_navigation_boxes=args.use_navigation_boxes, boxes_file=args.boxes_file)
    except (OSError, ValueError, TypeError, KeyError) as exc:
        parser.exit(2, 'query self-filter candidate: ' + str(exc) + '\n')
    print(json.dumps(result, ensure_ascii=False, allow_nan=False))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
