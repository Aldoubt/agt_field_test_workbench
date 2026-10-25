#!/usr/bin/env python3
"""Exercise native rolling costmap rates with synthetic TF in a private domain."""
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
    parser.add_argument('--params', type=Path, required=True)
    parser.add_argument('--clouds', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    os.environ.update(ROS_DOMAIN_ID='182', ROS_LOCALHOST_ONLY='1',
                      ROS_LOG_DIR=str(args.output/'ros_logs'))
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import PolygonStamped, TransformStamped
    from lifecycle_msgs.srv import ChangeState
    from nav_msgs.msg import OccupancyGrid
    from sensor_msgs.msg import PointCloud2, PointField
    from tf2_ros import TransformBroadcaster

    rclpy.init();node = rclpy.create_node('rolling_costmap_rate_probe')
    tf = TransformBroadcaster(node)
    raw_pub = node.create_publisher(PointCloud2, '/agt/livox/points', 1)
    mark_pub = node.create_publisher(PointCloud2, '/agt/navigation/points_obstacles', 1)
    points = np.load(args.clouds)['masked'].astype('<f4')
    cloud = PointCloud2(height=1, width=len(points), point_step=12, row_step=12*len(points),
        fields=[PointField(name=name, offset=index*4, datatype=PointField.FLOAT32, count=1)
                for index,name in enumerate(['x','y','z'])], data=points.tobytes(), is_dense=True)
    cloud.header.frame_id = 'base_link'
    started = time.monotonic();observations = {'footprint': [], 'costmap': []};origins = []
    def transform():
        message = TransformStamped();message.header.stamp = node.get_clock().now().to_msg()
        message.header.frame_id = 'odom';message.child_frame_id = 'base_link'
        message.transform.translation.x = .03*(time.monotonic()-started)
        message.transform.rotation.w = 1.;tf.sendTransform(message)
    def publish():
        cloud.header.stamp = node.get_clock().now().to_msg();raw_pub.publish(cloud);mark_pub.publish(cloud)
    def footprint(_message):observations['footprint'].append(time.monotonic())
    def costmap(message):
        observations['costmap'].append(time.monotonic());origins.append(message.info.origin.position.x)
    timers = [node.create_timer(.05,transform), node.create_timer(.1,publish)]
    subscriptions = [node.create_subscription(PolygonStamped, '/local_costmap/published_footprint', footprint,qos_profile_sensor_data),
                     node.create_subscription(OccupancyGrid, '/local_costmap/costmap', costmap,qos_profile_sensor_data)]
    params = yaml.safe_load(args.params.read_text())
    results = {}
    try:
        for name,update,publish_hz in [('baseline',10.,5.),('target',20.,10.)]:
            selected = yaml.safe_load(yaml.safe_dump(params))
            selected['local_costmap']['local_costmap']['ros__parameters'].update(
                update_frequency=update, publish_frequency=publish_hz)
            config = args.output/(name+'.yaml');config.write_text(yaml.safe_dump(selected))
            stream = (args.output/(name+'.log')).open('w')
            # Use the costmap's production owner, which spins its child node.
            child = subprocess.Popen(['/opt/ros/humble/lib/nav2_controller/controller_server',
                '--ros-args','--params-file',str(config)],
                stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
            client = node.create_client(ChangeState,'/controller_server/change_state')
            try:
                deadline = time.monotonic()+10
                while not client.service_is_ready() and time.monotonic()<deadline:
                    rclpy.spin_once(node,timeout_sec=.05)
                assert client.service_is_ready(), 'native costmap lifecycle service missing'
                for transition in (1,3):
                    request = ChangeState.Request();request.transition.id = transition
                    future = client.call_async(request)
                    rclpy.spin_until_future_complete(node,future,timeout_sec=8)
                    assert future.done() and future.result() and future.result().success
                until = time.monotonic()+2
                while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.02)
                observations = {'footprint': [], 'costmap': []};origins = []
                until = time.monotonic()+8
                while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.02)
                rates = {key:(len(values)-1)/(values[-1]-values[0]) if len(values)>1 else 0.
                         for key,values in observations.items()}
                results[name] = dict(configured_update_hz=update,configured_publish_hz=publish_hz,
                    observed_hz=rates,rolling_origin_span_m=max(origins)-min(origins) if origins else 0.)
            finally:
                if child.poll() is None:os.killpg(child.pid,signal.SIGINT);child.wait(timeout=5)
                stream.close();node.destroy_client(client)
        results['checks'] = dict(update_20hz=18<=results['target']['observed_hz']['footprint']<=22,
            publication_faster=results['target']['observed_hz']['costmap']>1.5*results['baseline']['observed_hz']['costmap'],
            rolling_origin_follows_tf=results['target']['rolling_origin_span_m']>.15)
        results['passed'] = all(results['checks'].values())
        (args.output/'result.json').write_text(json.dumps(results,indent=2)+'\n');print(json.dumps(results,indent=2))
        return 0 if results['passed'] else 1
    finally:
        node.destroy_node();rclpy.shutdown()


if __name__ == '__main__':raise SystemExit(main())
