#!/usr/bin/env python3
"""One-shot client for the existing C1 AcquireView capability; never commands base motion."""
import argparse
import json
import math
import os
from pathlib import Path
import sys
import time
from datetime import datetime, timezone


def write_json(path, value):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    temporary=path.with_suffix(path.suffix+'.tmp')
    temporary.write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
    temporary.replace(path)


def stamp(value):
    return {'sec':int(value.sec),'nanosec':int(value.nanosec),
            'nanoseconds':int(value.sec)*1_000_000_000+int(value.nanosec)}


def append_manifest(path, row):
    import fcntl
    path=Path(path)
    path.parent.mkdir(parents=True,exist_ok=True)
    with path.open('a',encoding='utf8') as stream:
        fcntl.flock(stream.fileno(),fcntl.LOCK_EX)
        stream.write(json.dumps(row,ensure_ascii=False,sort_keys=True)+'\n')
        stream.flush();os.fsync(stream.fileno())
        fcntl.flock(stream.fileno(),fcntl.LOCK_UN)


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--request',required=True)
    parser.add_argument('--result',required=True)
    parser.add_argument('--capture-root',required=True)
    args=parser.parse_args()
    request=json.loads(Path(args.request).read_text(encoding='utf8'))
    root=Path(args.capture_root).expanduser().resolve()
    if not root.is_absolute() or not root.is_dir():
        raise ValueError('camera capture session directory missing')
    heading=float(request['heading']);pitch=float(request['pitch']);roll=float(request['roll'])
    if not all(math.isfinite(v) for v in (heading,pitch,roll)):
        raise ValueError('non-finite gimbal angle')
    if not -160<=heading<=160 or not -90<=pitch<=90 or not -40<=roll<=40:
        raise ValueError('gimbal angle exceeds C1 mechanical limits')
    try:
        import rclpy
        from rclpy.action import ActionClient
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from camera_gimbal_interfaces.action import AcquireView
    except Exception as exc:
        raise RuntimeError('C1 AcquireView interface unavailable in sourced ROS environment: '+str(exc)) from exc
    rclpy.init(args=None)
    node=Node('agt_workbench_camera_capture_client')
    client=ActionClient(node,AcquireView,'/camera_gimbal/acquire_view')
    executor=SingleThreadedExecutor();executor.add_node(node)
    handle=None
    result_doc={'success':False,'tag':request.get('tag'),'requested_angles':{'heading':heading,'pitch':pitch,'roll':roll},
                'save_image':bool(request.get('save_image',True)),'metadata':request.get('metadata',{}),
                'camera_session_id':request.get('camera_session_id'),'requested_at':request.get('requested_at'),
                'finished_at':datetime.now(timezone.utc).isoformat()}
    try:
        if not client.wait_for_server(timeout_sec=3.0):
            raise RuntimeError('/camera_gimbal/acquire_view action server not ready')
        goal=AcquireView.Goal()
        goal.heading=heading;goal.pitch=pitch;goal.roll=roll
        goal.tolerance=1.5;goal.timeout=8.0;goal.stable_samples=3
        goal.settle_time=0.25;goal.image_timeout=2.0
        goal.save_image=bool(request.get('save_image',True));goal.tag=str(request.get('tag','capture'))
        if hasattr(goal,'capture_name'):goal.capture_name=goal.tag
        sent=client.send_goal_async(goal)
        deadline=time.monotonic()+3.0
        while rclpy.ok() and not sent.done() and time.monotonic()<deadline:executor.spin_once(timeout_sec=.05)
        if not sent.done():raise TimeoutError('AcquireView goal handshake timed out')
        handle=sent.result()
        if handle is None or not handle.accepted:raise RuntimeError('AcquireView goal rejected')
        future=handle.get_result_async()
        deadline=time.monotonic()+16.0
        while rclpy.ok() and not future.done() and time.monotonic()<deadline:executor.spin_once(timeout_sec=.05)
        if not future.done():
            cancel=handle.cancel_goal_async()
            cancel_deadline=time.monotonic()+1.0
            while rclpy.ok() and not cancel.done() and time.monotonic()<cancel_deadline:executor.spin_once(timeout_sec=.03)
            raise TimeoutError('AcquireView timed out; cancellation requested')
        wrapper=future.result()
        if wrapper is None:raise RuntimeError('AcquireView returned no result')
        result=wrapper.result
        result_doc.update(success=bool(result.success),error_code=int(result.error_code),
            message=str(result.message),image_path=str(result.image_path),
            reached_stamp=stamp(result.reached_stamp),image_stamp=stamp(result.image_stamp),
            actual_angles={'heading':float(result.actual_heading),'roll':float(result.actual_roll),'pitch':float(result.actual_pitch)},
            finished_at=datetime.now(timezone.utc).isoformat())
        if result_doc['success']:
            if result_doc['image_stamp']['nanoseconds']<=result_doc['reached_stamp']['nanoseconds']:
                raise RuntimeError('capability returned image not newer than stable gimbal arrival')
            if result_doc['save_image']:
                image=Path(result.image_path).resolve()
                if not image.is_relative_to(root) or not image.is_file() or image.stat().st_size<=0:
                    raise RuntimeError('AcquireView reported success but image is missing/outside capture session')
                result_doc['image_path']=str(image)
                result_doc['manifest_path']=str(image.parent/'manifest.jsonl')
                append_manifest(image.parent/'manifest.jsonl',result_doc)
            else:
                append_manifest(root/'actions.jsonl',result_doc)
        write_json(args.result,result_doc)
        print(json.dumps(result_doc,ensure_ascii=False))
        return 0 if result_doc['success'] else 3
    except KeyboardInterrupt:
        if handle is not None:
            try:
                cancel=handle.cancel_goal_async()
                deadline=time.monotonic()+1.2
                while rclpy.ok() and not cancel.done() and time.monotonic()<deadline:executor.spin_once(timeout_sec=.03)
            except Exception:pass
        result_doc.update(success=False,message='cancelled by Workbench operator',finished_at=datetime.now(timezone.utc).isoformat())
        write_json(args.result,result_doc)
        return 130
    except Exception as exc:
        result_doc.update(success=False,message=str(exc),finished_at=datetime.now(timezone.utc).isoformat())
        write_json(args.result,result_doc)
        print(json.dumps(result_doc,ensure_ascii=False),file=sys.stderr)
        return 2
    finally:
        executor.remove_node(node);node.destroy_node();rclpy.shutdown()


if __name__=='__main__':
    sys.exit(main())
