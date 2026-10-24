#!/usr/bin/env python3
"""One-shot client for the existing C1 AcquireView capability; never commands base motion."""
import argparse
import hashlib
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
    if value.sec < 0 or not 0 <= value.nanosec < 1_000_000_000:
        raise ValueError('invalid camera timestamp')
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


def validate_capture(document, root):
    """Fail closed on bad action status, angle feedback, timestamps or disk output."""
    if not document['success']:
        return
    if document.get('action_status') != 4 or document.get('error_code') != 0:
        raise RuntimeError('AcquireView action did not succeed')
    for axis, actual in document['actual_angles'].items():
        if not math.isfinite(actual) or abs(actual-document['requested_angles'][axis]) > 1.5:
            raise RuntimeError('gimbal actual angle outside arrival tolerance: '+axis)
    if not 0 < document['reached_stamp']['nanoseconds'] < document['image_stamp']['nanoseconds']:
        raise RuntimeError('capability returned image not newer than stable gimbal arrival')
    if document['save_image']:
        image=Path(document['image_path']).resolve()
        if not image.is_relative_to(root) or not image.is_file() or image.stat().st_size<=0:
            raise RuntimeError('AcquireView reported success but image is missing/outside capture session')
        content=image.read_bytes()
        if not (content.startswith(b'\x89PNG\r\n\x1a\n') or content.startswith(b'\xff\xd8\xff')):
            raise RuntimeError('saved image is not PNG/JPEG')
        document.update(image_path=str(image),image_sha256=hashlib.sha256(content).hexdigest(),size_bytes=len(content))


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
    from tf2_ros import Buffer, TransformListener
    tf_buffer=Buffer();listener=TransformListener(tf_buffer,node)
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
        goal.capture_name=goal.tag
        def feedback(event):
            value=event.feedback
            write_json(Path(args.result).with_name('feedback.json'),dict(stage=value.stage,state=int(value.state),
                actual_angles=dict(heading=float(value.current_heading),roll=float(value.current_roll),pitch=float(value.current_pitch)),
                stable_count=int(value.stable_count),max_error=float(value.max_error),observed_at=datetime.now(timezone.utc).isoformat()))
        sent=client.send_goal_async(goal,feedback_callback=feedback)
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
        result_doc.update(success=bool(result.success),action_status=int(wrapper.status),error_code=int(result.error_code),
            message=str(result.message),image_path=str(result.image_path),
            reached_stamp=stamp(result.reached_stamp),image_stamp=stamp(result.image_stamp),
            actual_angles={'heading':float(result.actual_heading),'roll':float(result.actual_roll),'pitch':float(result.actual_pitch)},
            finished_at=datetime.now(timezone.utc).isoformat())
        if result_doc['success']:
            validate_capture(result_doc,root)
            try:
                from rclpy.time import Time
                transform=tf_buffer.lookup_transform('map','base_link',Time.from_msg(result.image_stamp)).transform
                q=transform.rotation
                if not all(math.isfinite(v) for v in (transform.translation.x,transform.translation.y,transform.translation.z,q.x,q.y,q.z,q.w)) or abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1)>1e-3:
                    raise ValueError('invalid capture-time robot TF')
                result_doc['pose_at_capture']=dict(frame_id='map',x=transform.translation.x,y=transform.translation.y,z=transform.translation.z,
                    yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
                result_doc['pose_valid']=True
            except Exception as exc:
                result_doc.update(pose_at_capture=None,pose_valid=False,pose_error=str(exc))
            if result_doc['save_image']:
                image=Path(result.image_path).resolve()
                result_doc['image_path']=str(image)
                result_doc['manifest_path']=str(image.parent/'manifest.jsonl')
                append_manifest(image.parent/'manifest.jsonl',result_doc)
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
        append_manifest(root/'actions.jsonl',result_doc)
        write_json(args.result,result_doc)
        return 130
    except Exception as exc:
        result_doc.update(success=False,message=str(exc),finished_at=datetime.now(timezone.utc).isoformat())
        append_manifest(root/'actions.jsonl',result_doc)
        write_json(args.result,result_doc)
        print(json.dumps(result_doc,ensure_ascii=False),file=sys.stderr)
        return 2
    finally:
        executor.remove_node(node);node.destroy_node();rclpy.shutdown()


if __name__=='__main__':
    try:
        sys.exit(main())
    except Exception as exc:
        # Import/configuration failures must also leave a reviewable result.
        arguments=sys.argv[1:]
        result=dict(success=False,message=str(exc),finished_at=datetime.now(timezone.utc).isoformat())
        if '--result' in arguments:write_json(arguments[arguments.index('--result')+1],result)
        if '--capture-root' in arguments:append_manifest(Path(arguments[arguments.index('--capture-root')+1])/'actions.jsonl',result)
        print(str(exc),file=sys.stderr)
        sys.exit(2)
