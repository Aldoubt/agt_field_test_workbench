"""Explicit, calibration-bound self masks for localization queries and keyframes.

This module never publishes or modifies Livox messages. Empty boxes disable the
feature. Boxes are in base_link; body-frame keyframes and raw live sensor points
are transformed only for the inclusion test, and their original coordinates are
retained for registration.
"""
import copy
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

# Shared numerical tolerance for an exact box face after rigid transforms.
# This is one nanometre, not a physical expansion of the measured mask.
_BOUNDARY_EPSILON = 1e-9


def _numbers(value, shape, name):
    values = np.asarray(value, dtype=object)
    if values.shape != shape or any(
        isinstance(v, (bool, np.bool_)) or not isinstance(v, (int, float, np.number))
        for v in values.flat
    ):
        raise ValueError('invalid self-filter ' + name)
    result = np.asarray(value, dtype=float)
    if not np.isfinite(result).all():
        raise ValueError('non-finite self-filter ' + name)
    return result


def _boxes(value):
    if not isinstance(value, list) or len(value) % 6:
        raise ValueError('self-filter boxes require xmin/xmax/ymin/ymax/zmin/zmax')
    boxes = _numbers(value, (len(value),), 'boxes').reshape((-1, 6))
    if len(boxes) and np.any(boxes[:, ::2] >= boxes[:, 1::2]):
        raise ValueError('self-filter box minimum must be below maximum')
    return boxes


def validate_self_filter_spec(spec):
    """Validate the normalized SC/PCD contract, returning an owned copy."""
    if spec is None:
        return None
    if not isinstance(spec, dict) or spec.get('frame') != 'base_link':
        raise ValueError('self-filter frame must be base_link')
    boxes = _boxes(spec.get('boxes'))
    if not len(boxes):
        return None
    rotation = _numbers(spec.get('base_from_body_rotation'), (3, 3), 'rotation')
    translation = _numbers(spec.get('base_from_body_translation'), (3,), 'translation')
    if (not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-8, rtol=0)
            or not np.isclose(np.linalg.det(rotation), 1., atol=1e-8, rtol=0)):
        raise ValueError('self-filter base_from_body rotation must be proper orthonormal')
    return dict(frame='base_link', boxes=boxes.reshape(-1).tolist(),
                base_from_body_rotation=np.round(rotation, 12).tolist(),
                base_from_body_translation=translation.tolist())


def normalize_self_filter(config):
    """Convert a versioned JSON configuration into the descriptor contract."""
    if (not isinstance(config, dict) or isinstance(config.get('schema_version'), bool)
            or config.get('schema_version') != 1 or config.get('frame') != 'base_link'):
        raise ValueError('self-filter requires schema_version=1 and frame=base_link')
    boxes = _boxes(config.get('boxes'))
    if not len(boxes):
        return None
    transform = config.get('base_from_body')
    if not isinstance(transform, dict):
        raise ValueError('self-filter requires the recorded base_from_body transform')
    translation = _numbers(transform.get('translation'), (3,), 'translation')
    quaternion = _numbers(transform.get('quaternion_xyzw'), (4,), 'quaternion')
    norm = np.linalg.norm(quaternion)
    if norm <= 1e-12:
        raise ValueError('self-filter quaternion has zero norm')
    return validate_self_filter_spec(dict(frame='base_link', boxes=boxes.reshape(-1).tolist(),
        base_from_body_rotation=Rotation.from_quat(quaternion / norm).as_matrix().tolist(),
        base_from_body_translation=translation.tolist()))


def make_self_filter_config(boxes, base_from_body_translation, base_from_body_quaternion_xyzw):
    """Build a JSON-serializable candidate; does not enable it or write files."""
    config = dict(schema_version=1, frame='base_link', boxes=list(boxes),
        base_from_body=dict(translation=list(base_from_body_translation),
                            quaternion_xyzw=list(base_from_body_quaternion_xyzw)))
    normalize_self_filter(config)
    return config


def load_self_filter(path):
    return normalize_self_filter(json.loads(Path(path).expanduser().read_text()))


def self_filter_keep_mask(body_points, spec):
    """Closed boxes match CropBox deletion; input coordinates are never changed."""
    spec = validate_self_filter_spec(spec)
    points = np.asarray(body_points, dtype=float)
    if points.size == 0:
        return np.zeros(0, dtype=bool)
    if points.ndim != 2 or points.shape[1] < 3:
        raise ValueError('self-filter points must have XYZ columns')
    if spec is None:
        return np.ones(len(points), dtype=bool)
    base = (points[:, :3] @ np.asarray(spec['base_from_body_rotation']).T
            + np.asarray(spec['base_from_body_translation']))
    keep = np.isfinite(base).all(axis=1)
    for box in np.asarray(spec['boxes']).reshape((-1, 6)):
        keep &= ~np.all((base >= box[::2] - _BOUNDARY_EPSILON)
                       & (base <= box[1::2] + _BOUNDARY_EPSILON), axis=1)
    return keep


class QuerySelfFilter:
    """One frozen request. TF is supplied by the existing node per input cloud."""
    def __init__(self, spec, report):
        self.spec = spec
        self.report = report
        self.boxes = np.asarray(spec['boxes']).reshape((-1, 6)).tolist()

    def predicate(self, transform):
        translation, quaternion = transform.translation, transform.rotation
        t = _numbers([translation.x, translation.y, translation.z], (3,), 'sensor translation')
        q = _numbers([quaternion.x, quaternion.y, quaternion.z, quaternion.w], (4,), 'sensor quaternion')
        if np.linalg.norm(q) <= 1e-12:
            raise ValueError('self-filter sensor TF quaternion has zero norm')
        r = Rotation.from_quat(q).as_matrix().tolist()
        tx, ty, tz = t.tolist()
        self.report['clouds'] += 1

        def keep(x, y, z):
            self.report['input_points'] += 1
            bx = r[0][0]*x + r[0][1]*y + r[0][2]*z + tx
            by = r[1][0]*x + r[1][1]*y + r[1][2]*z + ty
            bz = r[2][0]*x + r[2][1]*y + r[2][2]*z + tz
            for xmin, xmax, ymin, ymax, zmin, zmax in self.boxes:
                if (xmin - _BOUNDARY_EPSILON <= bx <= xmax + _BOUNDARY_EPSILON
                        and ymin - _BOUNDARY_EPSILON <= by <= ymax + _BOUNDARY_EPSILON
                        and zmin - _BOUNDARY_EPSILON <= bz <= zmax + _BOUNDARY_EPSILON):
                    self.report['removed_points'] += 1
                    return False
            return True
        return keep


def _verify_mapping_body_query(node):
    """An enabled mask must use the active LIO's actual query coordinates.

    The legacy BBS path can retain mapping_body_livox parameter defaults even
    after selecting another internal calibration. SC configures those values
    before collecting a query. Check both paths without modifying either one.
    """
    mode, _frame, fixed = node.query_frame_contract()
    if mode == 'base_link':
        return  # Compatibility mode obtains its point transform directly via TF.
    if mode != 'mapping_body' or not isinstance(fixed, dict):
        raise ValueError('unsupported self-filter query frame contract')
    selected = (str(node.get_parameter('body_to_base_calibration_file').value).strip()
                or str(node.get_parameter('batch_lio_config_file').value).strip())
    if not selected:
        raise ValueError('self-filter requires selected active LIO calibration')
    from agt_batch_lio_adapter.extrinsics import load_lio_body_to_lidar
    translation, quaternion = load_lio_body_to_lidar(str(Path(selected).expanduser()))
    expected_t = _numbers(translation, (3,), 'active LIO translation')
    expected_q = _numbers(quaternion, (4,), 'active LIO quaternion')
    actual_t = _numbers([fixed[k] for k in ('x', 'y', 'z')], (3,), 'query translation')
    actual_q = _numbers([fixed[k] for k in ('qx', 'qy', 'qz', 'qw')], (4,), 'query quaternion')
    if np.linalg.norm(expected_q) <= 1e-12 or np.linalg.norm(actual_q) <= 1e-12:
        raise ValueError('self-filter query/active LIO quaternion has zero norm')
    if (not np.allclose(actual_t, expected_t, atol=1e-9, rtol=0)
            or not np.allclose(Rotation.from_quat(actual_q).as_matrix(),
                               Rotation.from_quat(expected_q).as_matrix(), atol=1e-9, rtol=0)):
        raise ValueError('self-filter mapping_body_livox calibration mismatch with active LIO')


def prepare_query_self_filter(node):
    """Reject an enabled mask if its mapping-body calibration differs at runtime."""
    node._query_self_filter_spec = None
    node.last_query_self_filter = None
    path = str(node.get_parameter('query_self_filter_config').value).strip()
    if not path:
        return None
    data = Path(path).expanduser().read_bytes()
    spec = normalize_self_filter(json.loads(data))
    if spec is None:
        return None
    if str(node.get_parameter('mount_base_frame').value) != spec['frame']:
        raise ValueError('self-filter frame differs from mount_base_frame')
    # body_to_base_pose() returns T_body_base despite its method name.
    actual = node.inverse_pose(node.body_to_base_pose())
    t = [actual[key] for key in ('x', 'y', 'z')]
    r = Rotation.from_quat([actual[key] for key in ('qx', 'qy', 'qz', 'qw')]).as_matrix()
    if (not np.allclose(t, spec['base_from_body_translation'], atol=1e-6, rtol=0)
            or not np.allclose(r, spec['base_from_body_rotation'], atol=1e-6, rtol=0)):
        raise ValueError('self-filter base_from_body calibration mismatch; rebuild candidate assets')
    _verify_mapping_body_query(node)
    node._query_self_filter_spec = copy.deepcopy(spec)
    report = dict(enabled=True, frame='base_link', config_sha256=hashlib.sha256(data).hexdigest(),
                  boxes=len(spec['boxes']) // 6, clouds=0, input_points=0, removed_points=0)
    node.last_query_self_filter = report
    return QuerySelfFilter(spec, report)
