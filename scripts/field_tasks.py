#!/usr/bin/env python3
"""Distance tools: teach/generate only; run/home explicitly submit a prepared mission."""
import argparse
import csv
import json
import math
import os
from pathlib import Path
import re
import time
from datetime import datetime, timezone
import uuid

import yaml

ROOT = Path(os.environ.get('AGT_FIELD_TEST_WS', str(Path(__file__).resolve().parents[3])))


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
    distance = finite(distance)
    if distance <= 0:
        raise ValueError('distance must be positive')
    label = str(float(distance)).removesuffix('.0')
    home = data['points']['home']['pose']
    yaw = finite(home['yaw'])
    target = dict(x=finite(home['x'])+distance*math.cos(yaw),
                  y=finite(home['y'])+distance*math.sin(yaw), yaw=yaw, frame_id='map')
    return dict(version=1, mission_id=f'accuracy_{label}m', map_id=data['map_id'],
                points=[point('HOME_START', home), point(f'D{label}m', target)])


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


def execute_distance(path):
    """Only the explicit run/home commands can submit an Action goal."""
    from agt_navigation_runtime.mission_schema import load_mission
    from workbench import field_trial
    import rclpy
    from workbench.ros_parameters import read_parameters
    from workbench.mode_state import require_mode_ready
    path = Path(path).resolve()
    mission = load_mission(str(path))
    if any(point.frame_id != 'map' for point in mission.points):
        raise ValueError('distance missions must use the map frame')
    if any(point.views for point in mission.points):
        raise ValueError('distance missions cannot contain camera views')
    config = yaml.safe_load((ROOT/'config/field_trial.yaml').read_text())
    if mission.map_id != config['map_id']:
        raise ValueError('mission belongs to another map; use the matching field_trial config')
    require_mode_ready(ROOT)
    rclpy.init(args=[])
    node = rclpy.create_node('field_distance_submit')
    try:
        field_trial.require_navigation_ready(node)
        tree = read_parameters(node,'mission_runtime',['nav_behavior_tree'])['nav_behavior_tree']
        if tree != '':
            raise ValueError('当前为拍照执行器；距离模式需要原精度到达判定')
    finally:
        node.destroy_node()
        rclpy.shutdown()
    goal = yaml.safe_dump(dict(mission_file=str(path),resume=False))
    print('提交距离任务：'+str(path),flush=True)
    os.execvp('ros2',['ros2','action','send_goal','/agt/mission/execute',
                     'agt_robot_interfaces/action/ExecuteInspectionMission',goal,'--feedback'])


def run_from_current_pose(distance):
    """One target relative to this invocation's pose; never visits an old home."""
    from workbench.mode_state import require_mode_ready
    require_mode_ready(ROOT)
    config = yaml.safe_load((ROOT/'config/field_trial.yaml').read_text())
    start = read_pose(config['map_id'])
    target = dict(x=start['x'] + distance * math.cos(start['yaw']),
                  y=start['y'] + distance * math.sin(start['yaw']),
                  yaw=start['yaw'], frame_id='map')
    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    directory = ROOT/'missions'/f'forward_{stamp}_{uuid.uuid4().hex[:8]}'
    mission = dict(version=1, mission_id=directory.name, map_id=config['map_id'],
                   points=[point('FORWARD', target)])
    save_new(directory/'start_pose.yaml', dict(map_id=config['map_id'], pose=start, distance_m=distance))
    save_new(directory/'mission.yaml', mission)
    print(f'从当前位姿向前 {distance:g} m：起点 ({start["x"]:.3f}, {start["y"]:.3f})，'
          f'目标 ({target["x"]:.3f}, {target["y"]:.3f})；到点结束，不自动返程。', flush=True)
    print(f'诊断任务文件：{directory.resolve()}', flush=True)
    return execute_distance(directory/'mission.yaml')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--points', type=Path, default=ROOT/'config/field_test_points.yaml')
    sub = parser.add_subparsers(dest='command', required=True)
    teach = sub.add_parser('teach', help='read the parked robot pose; no goals are sent')
    teach.add_argument('id')
    teach.add_argument('--map-id')
    for angle in ('heading', 'pitch', 'roll'):
        teach.add_argument('--'+angle, type=finite, default=0.)
    accuracy = sub.add_parser('accuracy', help='capture the current parked pose and generate fixed targets for this batch')
    accuracy.add_argument('--trials', type=int, default=5)
    accuracy.add_argument('--output', type=Path, help='new batch directory; defaults to a unique directory')
    accuracy.add_argument('--distances', type=finite, nargs='+', default=[5.,7.,10.])
    photos = sub.add_parser('photos', help='generate a multi-point AcquireView mission')
    photos.add_argument('ids', nargs='+')
    photos.add_argument('--output', type=Path, required=True)
    run = sub.add_parser('run', help='navigate forward from the current pose; no preparation required')
    run.add_argument('distance', type=finite)
    run.add_argument('--directory',type=Path,help='explicitly use a legacy fixed-anchor batch instead of the current pose')
    home = sub.add_parser('home', help='explicitly execute the prepared return-home mission')
    home.add_argument('--directory',type=Path,help='prepared batch; defaults to the last successfully generated batch')
    sub.add_parser('cancel',help='request parent mission cancellation')
    args = parser.parse_args(argv)
    if args.command == 'cancel':
        return os.execvp('ros2',['ros2','service','call','/agt/task/cancel','std_srvs/srv/Trigger','{}'])
    if args.command in ('run','home'):
        if args.command == 'run' and args.distance <= 0:
            raise ValueError('distance must be positive')
        if args.command == 'run' and args.directory is None:
            return run_from_current_pose(args.distance)
        filename = ('return_home.yaml' if args.command == 'home' else
                    'accuracy_'+str(float(args.distance)).removesuffix('.0')+'m.yaml')
        directory = args.directory
        if directory is None:
            active = ROOT/'work/active_distance_batch.json'
            if not active.is_file():
                raise ValueError('尚未选择本批任务；请先执行 accuracy 记录当前起点，或用 --directory 明确指定已有批次。不会自动使用旧 missions/distance。')
            directory = Path(json.loads(active.read_text())['directory'])
            if not directory.is_absolute() or not (directory/'start_pose.yaml').is_file():
                raise ValueError('本批任务记录无效；请重新执行 accuracy 或明确指定 --directory。')
        print(f'使用距离批次：{directory.resolve()}', flush=True)
        return execute_distance(directory/filename)

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
        if not args.distances or any(d <= 0 for d in args.distances) or len(set(args.distances)) != len(args.distances):
            raise ValueError('distances must be positive and unique')
        if args.output is None:
            stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
            args.output = ROOT/'missions'/f'distance_{stamp}_{uuid.uuid4().hex[:8]}'
        if args.output.exists():
            raise FileExistsError(f'任务目录已存在：{args.output}；新一批测试请用 --output 指定新目录，run/home 使用同一 --directory。')
        config = yaml.safe_load((ROOT/'config/field_trial.yaml').read_text())
        home = read_pose(config['map_id'])
        data = dict(map_id=config['map_id'], points=dict(home=dict(pose=home)))
        args.output.mkdir(parents=True, exist_ok=False)
        save_new(args.output/'start_pose.yaml', data)
        with (args.output/'physical_measurements.csv').open('x', newline='') as stream:
            fields = ['distance_m', 'trial', 'goal_x', 'goal_y', 'measured_x', 'measured_y',
                      'longitudinal_error_m', 'lateral_error_m', 'yaw_error_deg',
                      'nav_result', 'truth_source', 'evidence', 'notes']
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            for distance in args.distances:
                label = str(float(distance)).removesuffix('.0')
                mission = accuracy_mission(data, distance)
                save_new(args.output/f'accuracy_{label}m.yaml', mission)
                target = mission['points'][1]['pose']
                for trial in range(1, args.trials+1):
                    writer.writerow(dict(distance_m=distance, trial=trial,
                                         goal_x=target['x'], goal_y=target['y']))
        save_new(args.output/'return_home.yaml', dict(
            version=1, mission_id='accuracy_return_home', map_id=data['map_id'],
            points=[point('HOME', data['points']['home']['pose'])]))
        active = ROOT/'work/active_distance_batch.json'
        active.parent.mkdir(parents=True, exist_ok=True)
        temporary = active.with_suffix('.tmp')
        temporary.write_text(json.dumps(dict(directory=str(args.output.resolve())), ensure_ascii=False))
        temporary.replace(active)
        print('已保存固定起点批次；执行本批任务请使用 run/home --directory 指定此目录。')
        print(f'本批起点（当前位姿）：x={home["x"]:.3f}, y={home["y"]:.3f}, yaw={home["yaw"]:.3f} rad')
        print(f'任务目录：{args.output.resolve()}')
        print('本批所有轮次复用此起点；run 先到 HOME_START，再到距离终点；home 返回本批起点。')
    else:
        save_new(args.output, photo_mission(load_points(args.points), args.ids))
    print('Prepared only; no navigation or camera action was sent.')


if __name__ == '__main__':
    try:
        main()
    except (ValueError, KeyError, OSError) as exc:
        raise SystemExit(str(exc))
