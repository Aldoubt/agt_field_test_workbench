#!/usr/bin/env python3
"""Prepare existing inspection missions; never send navigation or camera goals."""
import argparse
import csv
import json
import math
from pathlib import Path
import re
import time

import yaml

ROOT = Path(__file__).resolve().parents[3]


def finite(value):
    value = float(value)
    if not math.isfinite(value):
        raise ValueError('coordinates and angles must be finite')
    return value


def load_points(path):
    data = yaml.safe_load(Path(path).read_text())
    if not data.get('map_id') or not data.get('points'):
        raise ValueError('teach at least one point first')
    return data


def read_pose(map_id, timeout=8.0):
    import rclpy
    from rclpy.node import Node
    from rclpy.time import Time
    from tf2_ros import Buffer, TransformListener, TransformException
    from agt_robot_interfaces.msg import LocalizationStatus

    rclpy.init(args=[])
    node = Node('field_tasks_read_pose')
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    statuses = []
    node.create_subscription(
        LocalizationStatus, '/agt/localization/status', statuses.append, 10)
    deadline = time.monotonic() + timeout
    error = 'waiting for fresh localization and map -> base_link TF'
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
            if not statuses:
                continue
            status = statuses[-1]
            if not status.local_odom_fresh or not status.global_correction_valid:
                error = 'localization is not valid: ' + status.reason
                continue
            if status.map_id and status.map_id != map_id:
                raise ValueError('live localization map differs from selected map')
            now = node.get_clock().now().nanoseconds
            age = (now - Time.from_msg(status.stamp).nanoseconds) / 1e9
            if not -.05 <= age <= .5:
                error = 'localization status is stale'
                continue
            try:
                transform = buffer.lookup_transform('map', 'base_link', Time())
            except TransformException as exc:
                error = str(exc)
                continue
            tf_age = (now - Time.from_msg(transform.header.stamp).nanoseconds) / 1e9
            if not -.05 <= tf_age <= .20:
                error = 'map -> base_link TF is stale'
                continue
            t, q = transform.transform.translation, transform.transform.rotation
            values = [finite(v) for v in (t.x, t.y, q.x, q.y, q.z, q.w)]
            if abs(sum(v*v for v in values[2:]) - 1) > .001:
                raise ValueError('invalid robot orientation quaternion')
            return dict(x=float(t.x), y=float(t.y),
                        yaw=math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)),
                        frame_id='map', stamp_sec=transform.header.stamp.sec,
                        stamp_nanosec=transform.header.stamp.nanosec)
        raise ValueError(error)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def save_new(path, document):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('x', encoding='utf8') as stream:
        yaml.safe_dump(document, stream, sort_keys=False, allow_unicode=True)


def point(ident, pose, views=None):
    return dict(id=ident, pose={k: pose[k] for k in ('x', 'y', 'yaw', 'frame_id')},
                settle_time=1.0, views=views or [])


def accuracy_mission(data, distance):
    home = data['points']['home']['pose']
    yaw = finite(home['yaw'])
    target = dict(x=finite(home['x'])+distance*math.cos(yaw),
                  y=finite(home['y'])+distance*math.sin(yaw), yaw=yaw, frame_id='map')
    return dict(version=1, mission_id=f'accuracy_{distance}m', map_id=data['map_id'],
                points=[point('HOME_START', home), point(f'D{distance}m', target)])


def photo_mission(data, ids):
    if not ids or len(set(ids)) != len(ids):
        raise ValueError('photo point IDs must be nonempty and unique')
    points = []
    for ident in ids:
        saved = data['points'][ident]
        view = dict(tag=ident+'_view', **saved['view'], tolerance=1.5, timeout=8.0,
                    stable_samples=3, settle_time=.5, image_timeout=2.0,
                    required=True, save_image=True)
        points.append(point(ident, saved['pose'], [view]))
    return dict(version=1, mission_id='photo_route', map_id=data['map_id'], points=points)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--points', type=Path, default=ROOT/'config/field_test_points.yaml')
    sub = parser.add_subparsers(dest='command', required=True)
    teach = sub.add_parser('teach', help='read the parked robot pose; no goals are sent')
    teach.add_argument('id')
    teach.add_argument('--map-id')
    for angle in ('heading', 'pitch', 'roll'):
        teach.add_argument('--'+angle, type=finite, default=0.)
    accuracy = sub.add_parser('accuracy', help='generate fixed 5/7/10 m targets and a separate return mission')
    accuracy.add_argument('--trials', type=int, default=5)
    accuracy.add_argument('--output', type=Path, required=True)
    photos = sub.add_parser('photos', help='generate a multi-point AcquireView mission')
    photos.add_argument('ids', nargs='+')
    photos.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(argv)
    if args.command == 'teach':
        if not re.fullmatch(r'[A-Za-z0-9_-]{1,48}', args.id):
            raise ValueError('use a short point ID such as home or P01')
        for angle, limit in (('heading', 160), ('pitch', 90), ('roll', 40)):
            if abs(getattr(args, angle)) > limit:
                raise ValueError(angle+' exceeds gimbal range')
        cfg = yaml.safe_load((ROOT/'config/field_trial.yaml').read_text())
        map_id = args.map_id or cfg['map_id']
        data = load_points(args.points) if args.points.exists() else dict(map_id=map_id, points={})
        if data['map_id'] != map_id:
            raise ValueError('point book belongs to another map; use a new --points file')
        if args.id in data['points']:
            raise ValueError('point ID already recorded; use a new ID or a new point book')
        data['points'][args.id] = dict(pose=read_pose(map_id),
            view={angle: getattr(args, angle) for angle in ('heading', 'roll', 'pitch')})
        args.points.parent.mkdir(parents=True, exist_ok=True)
        temporary = args.points.with_suffix('.yaml.tmp')
        temporary.write_text(yaml.safe_dump(data, sort_keys=False), encoding='utf8')
        temporary.replace(args.points)
        print(json.dumps(data['points'][args.id], ensure_ascii=False, indent=2))
    elif args.command == 'accuracy':
        if not 1 <= args.trials <= 20:
            raise ValueError('trials must be between 1 and 20')
        data = load_points(args.points)
        if 'home' not in data['points']:
            raise ValueError('teach home before generating accuracy missions')
        args.output.mkdir(parents=True, exist_ok=False)
        with (args.output/'physical_measurements.csv').open('x', newline='') as stream:
            fields = ['distance_m', 'trial', 'goal_x', 'goal_y', 'measured_x', 'measured_y',
                      'longitudinal_error_m', 'lateral_error_m', 'yaw_error_deg',
                      'nav_result', 'truth_source', 'evidence', 'notes']
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            for distance in (5, 7, 10):
                mission = accuracy_mission(data, distance)
                save_new(args.output/f'accuracy_{distance}m.yaml', mission)
                target = mission['points'][1]['pose']
                for trial in range(1, args.trials+1):
                    writer.writerow(dict(distance_m=distance, trial=trial,
                                         goal_x=target['x'], goal_y=target['y']))
        save_new(args.output/'return_home.yaml', dict(
            version=1, mission_id='accuracy_return_home', map_id=data['map_id'],
            points=[point('HOME', data['points']['home']['pose'])]))
    else:
        save_new(args.output, photo_mission(load_points(args.points), args.ids))
    print('Prepared only; no navigation or camera action was sent.')


if __name__ == '__main__':
    try:
        main()
    except (ValueError, KeyError, OSError) as exc:
        raise SystemExit(str(exc))
