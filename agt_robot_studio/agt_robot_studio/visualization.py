"""Bounded display snapshots. Decode/TF/projection happen outside Qt's GUI thread."""
import threading
import time
import numpy as np


class LatestFrames:
    """One value per fixed stream; slow consumers never build a frame backlog."""
    def __init__(self):
        self._values = {}
        self._lock = threading.Lock()

    def put(self, stream, value):
        if stream not in ('cloud', 'grid', 'camera'):
            raise ValueError('unknown display stream')
        with self._lock:
            self._values[stream] = value

    def take(self):
        with self._lock:
            values, self._values = self._values, {}
            return values


def transform_xyz(xyz, transform):
    q = transform.rotation
    quat = np.array([q.x, q.y, q.z, q.w], dtype=float)
    norm = np.linalg.norm(quat)
    if not np.isfinite(quat).all() or norm < 1e-9:
        raise ValueError('invalid TF quaternion')
    x, y, z, w = quat / norm
    rotation = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                         [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                         [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
    t = transform.translation
    offset = np.array([t.x, t.y, t.z])
    if not np.isfinite(offset).all():
        raise ValueError('invalid TF translation')
    return xyz @ rotation.T + offset


def cloud_snapshot(xyz, frame_id, source, received=None):
    points = np.asarray(xyz, dtype=np.float32)
    if points.ndim != 2 or points.shape[1] != 3 or not len(points):
        raise ValueError('empty/invalid point cloud')
    points = points[np.isfinite(points).all(axis=1)][:6500].copy()
    if not len(points):
        raise ValueError('no finite cloud points')
    center = np.median(points, axis=0)
    span = max(float(np.percentile(np.abs(points-center), 97)), 1.)
    bins = np.digitize(points[:, 2]-center[2], [-.4, .12, .8, 1.8])
    projections = {}
    for name, axes in (('top', (0, 1)), ('side', (0, 2))):
        plane = points[:, axes].copy()
        plane.flags.writeable = False
        projections[name] = plane
    points.flags.writeable = False
    bins.flags.writeable = False
    return dict(xyz=points, projections=projections, bins=bins, center=center,
                span=span, frame_id=frame_id, source=source,
                received=time.monotonic() if received is None else received)
