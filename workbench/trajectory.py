"""Reject numerical LIO divergence before importing a cloud or allocating a grid.

Passing this check is not a geometric accuracy or navigation acceptance claim.
"""
import csv
import math
from pathlib import Path


def inspect_trajectory(path, max_speed_mps=10.0, max_step_m=10.0,
                       max_position_m=10000.0):
    count, length, max_step, max_speed = 0, 0.0, 0.0, 0.0
    previous = None
    minimum, maximum = [math.inf]*3, [-math.inf]*3
    reasons = []
    with Path(path).open(newline='') as stream:
        for row in csv.DictReader(stream):
            values = [float(row[key]) for key in ('stamp','x','y','z','qx','qy','qz','qw')]
            count += 1
            if not all(math.isfinite(v) for v in values):
                reasons.append('nonfinite_pose')
                continue
            stamp, *pose = values
            point = pose[:3]
            if max(abs(v) for v in point) > max_position_m:
                reasons.append('position_out_of_bounds')
            for axis in range(3):
                minimum[axis] = min(minimum[axis], point[axis])
                maximum[axis] = max(maximum[axis], point[axis])
            if previous is not None:
                dt = stamp-previous[0]
                step = math.dist(point, previous[1])
                length += step
                max_step = max(max_step, step)
                if dt <= 0:
                    reasons.append('nonincreasing_pose_stamp')
                else:
                    max_speed = max(max_speed, step/dt)
            previous = (stamp, point)
    if count < 2:
        reasons.append('too_few_poses')
    if max_step > max_step_m:
        reasons.append('implausible_step')
    if max_speed > max_speed_mps:
        reasons.append('implausible_speed')
    return dict(state='REJECTED' if reasons else 'PASS', reasons=sorted(set(reasons)),
                pose_count=count, path_length_m=length, max_step_m=max_step,
                max_speed_mps=max_speed,
                xyz_span_m=[b-a for a,b in zip(minimum,maximum)] if previous else None,
                limits=dict(max_step_m=max_step_m,max_speed_mps=max_speed_mps,
                            max_position_m=max_position_m),
                scope='Numerical plausibility only; accuracy and navigation acceptance require independent evidence')
