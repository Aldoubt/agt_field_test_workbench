#!/usr/bin/env python3
"""Replay captured base-frame clouds through native A/B preprocessors, privately."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import numpy as np
import yaml


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--capture', type=Path, required=True)
    parser.add_argument('--params', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    os.environ.update(ROS_DOMAIN_ID='181', ROS_LOCALHOST_ONLY='1',
                      ROS_LOG_DIR=str(args.output/'ros_logs'))
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import PointCloud2, PointField
    from std_msgs.msg import Header

    root = Path(__file__).resolve().parents[4]
    binary = root/'install/agt_pointcloud_preprocessor/lib/agt_pointcloud_preprocessor/agt_pointcloud_preprocessor'
    parameters = yaml.safe_load(args.params.read_text())['agt_pointcloud_preprocessor']['ros__parameters']
    boxes = parameters['self_filter']['extra_boxes']
    assert boxes
    rclpy.init()
    node = rclpy.create_node('navigation_self_mask_regression')
    received = {}
    children, logs, subscriptions = [], [], []

    def callback(name, message):
        fields = {field.name: field for field in message.fields}
        dtype = np.dtype(dict(names=['x','y','z'], formats=['<f4']*3,
                              offsets=[fields[key].offset for key in ['x','y','z']],
                              itemsize=message.point_step))
        raw = np.frombuffer(message.data, dtype=dtype)
        received[name] = np.column_stack([raw[key] for key in ['x','y','z']])

    try:
        for name in ('baseline', 'masked'):
            selected = yaml.safe_load(yaml.safe_dump(parameters))
            selected.update(input_topic='/self_mask_test/input',
                            output_topic='/self_mask_test/'+name,
                            base_frame='base_link', expected_input_frame='', statistics_output='')
            if name == 'baseline':
                selected['self_filter'].pop('extra_boxes', None)
            else:
                selected['self_filter']['extra_boxes'] = boxes
            config = args.output/(name+'.yaml')
            config.write_text(yaml.safe_dump({'/**': {'ros__parameters': selected}}))
            log = (args.output/(name+'.log')).open('w');logs.append(log)
            children.append(subprocess.Popen([str(binary), '--ros-args', '-r',
                '__node:=self_mask_'+name, '--params-file', str(config)],
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True))
            subscriptions.append(node.create_subscription(PointCloud2, '/self_mask_test/'+name,
                lambda message, name=name: callback(name, message), qos_profile_sensor_data))
        publisher = node.create_publisher(PointCloud2, '/self_mask_test/input', 1)
        points = np.load(args.capture)['raw'][::10].astype('<f4')
        message = PointCloud2(header=Header(frame_id='base_link'), height=1, width=len(points),
            fields=[PointField(name=key, offset=index*4, datatype=PointField.FLOAT32, count=1)
                    for index, key in enumerate(['x','y','z'])],
            is_bigendian=False, point_step=12, row_step=12*len(points),
            data=points.tobytes(), is_dense=True)
        started = time.monotonic();last_publish = 0.
        while time.monotonic()-started < 8:
            rclpy.spin_once(node, timeout_sec=.02)
            if time.monotonic()-last_publish > .5:
                message.header.stamp = node.get_clock().now().to_msg()
                publisher.publish(message);last_publish = time.monotonic()
        assert set(received) == {'baseline','masked'}, 'both native outputs required'
        def inside(values):
            result = np.zeros(len(values), dtype=bool)
            for start in range(0,len(boxes),6):
                bounds = boxes[start:start+6]
                result |= np.all((values >= np.array(bounds[::2])) &
                                 (values <= np.array(bounds[1::2])), axis=1)
            return result
        before, after = received['baseline'], received['masked']
        outside = before[~inside(before)]
        key = lambda array: {tuple(row) for row in array}
        checks = dict(self_returns_present_before=int(inside(before).sum())>0,
                      self_returns_removed=int(inside(after).sum())==0,
                      all_external_returns_preserved=key(outside)==key(after),
                      original_input_unchanged=bytes(message.data)==points.tobytes())
        result = dict(input_points=len(points), baseline_points=len(before), masked_points=len(after),
                      self_points_before=int(inside(before).sum()), self_points_after=int(inside(after).sum()),
                      external_points_preserved=len(key(outside)&key(after)),
                      bounds_base_link=boxes, checks=checks, passed=all(checks.values()))
        np.savez_compressed(args.output/'outputs.npz', baseline=before, masked=after)
        (args.output/'result.json').write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps(result,indent=2))
        return 0 if result['passed'] else 1
    finally:
        for child in children:
            if child.poll() is None:
                os.killpg(child.pid,signal.SIGINT)
                child.wait(timeout=5)
        for log in logs:log.close()
        node.destroy_node();rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
