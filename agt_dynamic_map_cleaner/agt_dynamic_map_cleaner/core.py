"""Bounded candidate-only clearance test for start-of-run stationary obstacles.

A voxel is removed only if:
* occupied during initial scans and present in the post-temporal static cloud;
* elevated above nearby ground and within the configurable start radius;
* later rays from separated robot viewpoints pass THROUGH the voxel and hit
  surfaces beyond it, AFTER the last observed occupancy.

No semantic person label is claimed. No footprint, near-field, or simple
temporal hit count is sufficient evidence on its own. The scan source must
have known timestamps and sensor origins in the same map frame.
"""
from dataclasses import dataclass
from collections import defaultdict
import math
import numpy as np


@dataclass(frozen=True)
class Settings:
    voxel_m: float = 0.25
    start_radius_m: float = 4.0
    initial_window_sec: float = 20.0
    min_early_frames: int = 3
    ground_clearance_m: float = 0.45
    max_above_ground_m: float = 2.3
    later_frame_stride: int = 5
    max_rays_per_frame: int = 300
    max_observer_distance_m: float = 15.0
    ray_tolerance_m: float = 0.13
    min_penetration_m: float = 0.65
    min_clear_after_hit_sec: float = 0.5
    min_distinct_viewpoints: int = 3
    viewpoint_cell_m: float = 0.40
    max_candidate_voxels: int = 15000
    max_removed_fraction: float = 0.08

    def validate(self):
        positive = ('voxel_m', 'start_radius_m', 'initial_window_sec',
                    'ground_clearance_m', 'max_above_ground_m',
                    'max_observer_distance_m', 'ray_tolerance_m',
                    'min_penetration_m', 'viewpoint_cell_m')
        for key in positive:
            value = getattr(self, key)
            if not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
                raise ValueError('invalid positive parameter ' + key)
        for key in ('min_early_frames', 'later_frame_stride', 'max_rays_per_frame',
                    'min_distinct_viewpoints', 'max_candidate_voxels'):
            val = getattr(self, key)
            if type(val) is not int or val < 1:
                raise ValueError('invalid positive integer parameter ' + key)
        if not 0 <= self.max_removed_fraction <= 1 or self.max_above_ground_m <= self.ground_clearance_m:
            raise ValueError('invalid candidate height or removal fraction')
        if self.min_clear_after_hit_sec < 0 or not math.isfinite(self.min_clear_after_hit_sec):
            raise ValueError('invalid clearance delay')


def cells(xyz, size):
    return np.floor(np.asarray(xyz, dtype=np.float64) / size).astype(np.int32)


def orientation(q):
    q = np.asarray(q, dtype=np.float64)
    if q.shape != (4,) or not np.isfinite(q).all() or not .99 < np.linalg.norm(q) < 1.01:
        raise ValueError('invalid pose quaternion xyzw')
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([
        [1 - 2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1 - 2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1 - 2*(x*x+y*y)],
    ], dtype=np.float64)


def world_points(frame):
    """Frame: (seconds, body origin xyz, quaternion xyzw, body XYZ)."""
    stamp, origin, quaternion, body = frame
    origin = np.asarray(origin, dtype=np.float64)
    body = np.asarray(body, dtype=np.float64)
    if origin.shape != (3,) or not np.isfinite(origin).all():
        raise ValueError('invalid body pose')
    if body.ndim != 2 or body.shape[1] != 3 or not np.isfinite(body).all():
        raise ValueError('invalid body point cloud')
    return float(stamp), origin, body @ orientation(quaternion).T + origin


def classify(static_xyz, frames, settings=Settings()):
    """Return (keep_mask, report); frames is a one-pass chronological iterable.

    The return is intentionally conservative: no later independent clearance
    views => keep every static-map point. Caller may render removed candidates
    for manual QA but must never auto-activate a navigation map.
    """
    settings.validate()
    points = np.asarray(static_xyz, dtype=np.float32)
    if points.ndim != 2 or points.shape[1] != 3 or len(points) == 0 or not np.isfinite(points).all():
        raise ValueError('nonempty finite static PCD required')
    it = iter(frames)
    try:
        first = next(it)
    except StopIteration:
        raise ValueError('no keyframes')
    t0, start, xyz = world_points(first)
    if not math.isfinite(t0):
        raise ValueError('invalid timestamp')
    near = np.linalg.norm(points[:, :2] - start[None, :2], axis=1) < settings.start_radius_m
    if not near.any():
        return np.ones(len(points), bool), {'state': 'NO_START_CLOUD', 'candidate_voxels': 0, 'removed_points': 0}
    # Local ground floor is estimated from a low percentile in each XY cell;
    # ground is never removed by this package.
    nearby = points[near]
    ground_xy = cells(nearby[:, :2], .75)
    xy_keys, inv = np.unique(ground_xy, axis=0, return_inverse=True)
    local_z = {}
    for k, key in enumerate(xy_keys):
        local_z[tuple(key)] = float(np.quantile(nearby[inv == k, 2], .08))
    start_keys = cells(nearby, settings.voxel_m)
    eligible = np.zeros(len(nearby), dtype=bool)
    for i, pt in enumerate(nearby):
        floor = local_z[tuple(ground_xy[i])]
        eligible[i] = floor + settings.ground_clearance_m <= pt[2] <= floor + settings.max_above_ground_m
    all_keys = sorted({tuple(int(v) for v in row) for row in start_keys[eligible]})
    if len(all_keys) > settings.max_candidate_voxels:
        raise ValueError('too many initial elevated voxels; narrow start ROI')
    available = set(all_keys)
    support = defaultdict(int)
    early_last_seen = {}
    def observe_early(stamp, cloud):
        if stamp < t0 or not math.isfinite(stamp):
            raise ValueError('non-monotonic keyframe time')
        cloud = cloud[np.linalg.norm(cloud[:, :2] - start[None, :2], axis=1) < settings.start_radius_m]
        for item in np.unique(cells(cloud, settings.voxel_m), axis=0):
            key = tuple(int(v) for v in item)
            if key in available:
                support[key] += 1
                early_last_seen[key] = stamp
    observe_early(t0, xyz)
    late = None
    last_time = t0
    for frame in it:
        stamp, origin, world = world_points(frame)
        if not math.isfinite(stamp) or stamp <= last_time:
            raise ValueError('non-monotonic keyframe timestamp')
        last_time = stamp
        if stamp - t0 > settings.initial_window_sec:
            late = (stamp, origin, world)
            break
        observe_early(stamp, world)
    selected = [k for k in all_keys if support[k] >= settings.min_early_frames]
    keep = np.ones(len(points), dtype=bool)
    if not selected or late is None:
        return keep, {'state': 'INSUFFICIENT_EVIDENCE', 'candidate_voxels': len(selected),
                      'removed_points': 0, 'early_supported_voxels': len(selected)}
    indices = {key: i for i, key in enumerate(selected)}
    center = (np.array(selected, dtype=np.float64) + .5) * settings.voxel_m
    last_hit = np.array([early_last_seen[k] for k in selected], dtype=float)
    # Last free observation time for each spatially distinct observer.
    frees = [dict() for _ in selected]
    total_later, ray_frames = 0, 0
    from itertools import chain
    for stamp, origin, world in chain((late,), (world_points(frame) for frame in it)):
        if stamp <= last_time and total_later:
            raise ValueError('non-monotonic late keyframe timestamp')
        last_time = stamp
        total_later += 1
        if np.linalg.norm(origin[:2] - start[:2]) > settings.max_observer_distance_m:
            continue
        # Sample scan occupancy without sub-sampling: late returns veto old
        # false-free rays, even if visualization/ray evidence is throttled.
        cloud = world[np.linalg.norm(world[:, :2] - start[None, :2], axis=1) < settings.start_radius_m]
        for row in np.unique(cells(cloud, settings.voxel_m), axis=0):
            i = indices.get(tuple(int(v) for v in row))
            if i is not None:
                last_hit[i] = stamp
        if total_later % settings.later_frame_stride:
            continue
        rays = world - origin
        ranges = np.linalg.norm(rays, axis=1)
        valid = np.isfinite(ranges) & (ranges > settings.min_penetration_m + .5) & (ranges < 35.0)
        ray_ids = np.flatnonzero(valid)
        if len(ray_ids) > settings.max_rays_per_frame:
            ray_ids = ray_ids[np.linspace(0, len(ray_ids)-1, settings.max_rays_per_frame).astype(int)]
        if not len(ray_ids):
            continue
        directions = rays[ray_ids] / ranges[ray_ids, None]
        rel = center - origin
        along = directions @ rel.T
        # Center-to-ray distance; only rays ending well beyond the candidate
        # constitute clearance evidence.
        perp2 = np.maximum(0., np.sum(rel**2, axis=1)[None, :] - along**2)
        free = np.any((along > .3) &
                      (along < ranges[ray_ids, None] - settings.min_penetration_m) &
                      (perp2 < settings.ray_tolerance_m**2), axis=0)
        viewpoint = tuple(np.floor(origin[:2] / settings.viewpoint_cell_m).astype(int))
        for i in np.flatnonzero(free):
            frees[i][viewpoint] = stamp
        ray_frames += 1
    cleared = []
    for i, observed in enumerate(frees):
        if sum(stamp > last_hit[i] + settings.min_clear_after_hit_sec for stamp in observed.values()) >= settings.min_distinct_viewpoints:
            cleared.append(selected[i])
    cleared_keys = set(cleared)
    # Only remove points corresponding to fully supported candidate voxels.
    for j in np.flatnonzero(near):
        if tuple(int(v) for v in start_keys[np.searchsorted(np.flatnonzero(near), j)]) in cleared_keys:
            keep[j] = False
    removed = int(np.count_nonzero(~keep))
    if removed / len(points) > settings.max_removed_fraction:
        raise ValueError('clearance candidate removes too much static map; manual review required')
    report = {
        'state': 'CANDIDATE' if removed else 'INSUFFICIENT_EVIDENCE',
        'input_points': int(len(points)), 'start_radius_m': settings.start_radius_m,
        'candidate_voxels': len(selected), 'cleared_voxels': len(cleared),
        'removed_points': removed, 'retained_points': int(keep.sum()),
        'observed_later_frames': total_later, 'evaluated_ray_frames': ray_frames,
        'semantic_person_detection': False, 'automatic_activation': False,
        'method': 'initial_elevated_occupancy_plus_late_multi_view_free_rays',
    }
    return keep, report
