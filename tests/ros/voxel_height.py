"""Native VoxelLayer A/B at the observed odom height; no navigation goals."""
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
    parser = argparse.ArgumentParser()
    parser.add_argument('--baseline', type=Path, required=True)
    parser.add_argument('--target', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    os.environ.update(ROS_DOMAIN_ID='185', ROS_LOCALHOST_ONLY='1', ROS_LOG_DIR=str(args.output/'logs'))
    import rclpy
    from geometry_msgs.msg import TransformStamped
    from lifecycle_msgs.srv import ChangeState
    from nav_msgs.msg import OccupancyGrid
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import PointCloud2, PointField
    from tf2_ros import TransformBroadcaster

    rclpy.init()
    node = rclpy.create_node('voxel_height_test')
    broadcaster = TransformBroadcaster(node)
    latest = {}
    def cloud(points):
        points = np.asarray(points, dtype='<f4')
        m = PointCloud2(height=1, width=len(points), point_step=12, row_step=12*len(points),
            fields=[PointField(name=k, offset=i*4, datatype=PointField.FLOAT32, count=1)
                    for i,k in enumerate(('x','y','z'))], data=points.tobytes(), is_dense=True)
        m.header.frame_id = 'test_sensor'
        return m
    # Sensor odom z=-0.15; base z=-0.85, similar to the stationary live snapshot.
    # A 0.4 m-high obstacle relative to the base is odom z=-0.45.
    obstacle = cloud([[1.5, 0., -.30]])
    ground = cloud([[3., 1., -.85], [3., -1., -.85]])
    mark = node.create_publisher(PointCloud2, '/height_test/mark', 10)
    clear = node.create_publisher(PointCloud2, '/height_test/clear', 10)
    def tick():
        stamp = node.get_clock().now().to_msg()
        for child,z in [('base_link',-.85),('test_sensor',-.15)]:
            t = TransformStamped()
            t.header.frame_id, t.child_frame_id = 'odom', child
            t.header.stamp = stamp
            t.transform.translation.z = z
            t.transform.rotation.w = 1.
            broadcaster.sendTransform(t)
        for msg,pub in [(obstacle,mark),(ground,clear)]:
            msg.header.stamp = stamp
            pub.publish(msg)
    timer = node.create_timer(.05, tick)
    def endpoints(m):
        if m.width:
            a=np.frombuffer(m.data,dtype='<f4').reshape(-1,3)
            latest['endpoints']=a[np.isfinite(a).all(axis=1)]
    subscriptions = [node.create_subscription(OccupancyGrid, '/local_costmap/costmap',
                          lambda m:latest.update(grid=m), qos_profile_sensor_data),
                     node.create_subscription(PointCloud2, '/local_costmap/clearing_endpoints',
                          endpoints, qos_profile_sensor_data)]
    def spin(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:rclpy.spin_once(node, timeout_sec=.01)
    result={}
    try:
        for name, path in [('baseline',args.baseline),('target',args.target)]:
            tree=yaml.safe_load(path.read_text())
            layer=tree['local_costmap']['local_costmap']['ros__parameters']['voxel_layer']
            layer['lidar3d_mark']['topic']='/height_test/mark'
            layer['lidar3d_clear']['topic']='/height_test/clear'
            params=args.output/(name+'.yaml');params.write_text(yaml.safe_dump(tree))
            stream=(args.output/(name+'.log')).open('w')
            process=subprocess.Popen(['/opt/ros/humble/lib/nav2_controller/controller_server',
                '--ros-args','--params-file',str(params)], stdout=stream,stderr=subprocess.STDOUT,
                start_new_session=True)
            client=node.create_client(ChangeState,'/controller_server/change_state')
            try:
                end=time.monotonic()+10
                while not client.service_is_ready() and time.monotonic()<end:spin(.05)
                assert client.service_is_ready()
                for transition in (1,3):
                    req=ChangeState.Request();req.transition.id=transition
                    future=client.call_async(req)
                    rclpy.spin_until_future_complete(node,future,timeout_sec=8.)
                    assert future.done() and future.result().success
                latest.clear();spin(3.)
                grid=latest['grid'];x=int((1.5-grid.info.origin.position.x)/grid.info.resolution)
                y=int((0.-grid.info.origin.position.y)/grid.info.resolution)
                endpoints=latest['endpoints']
                cells=np.asarray(grid.data).reshape(grid.info.height,grid.info.width)
                # OccupancyGrid carries float32 resolution; allow the adjacent
                # cell at a metric boundary rather than truncating 109.999998.
                result[name]={'low_obstacle_cost':int(cells[y-1:y+2,x-1:x+2].max()),
                    'center_cell_cost':int(cells[y,x]),
                    'min_clear_endpoint_odom_z':float(endpoints[:,2].min())}
            finally:
                os.killpg(process.pid,signal.SIGINT)
                process.wait(timeout=5.)
                node.destroy_client(client);stream.close();spin(.3)
        result['checks']={'low_obstacle_was_discarded':result['baseline']['low_obstacle_cost']==0,
            'low_obstacle_is_now_lethal':result['target']['low_obstacle_cost']==100,
            'ground_rays_were_clipped':abs(result['baseline']['min_clear_endpoint_odom_z']+.3)<.02,
            'ground_rays_reach_endpoint':result['target']['min_clear_endpoint_odom_z']<-.9}
        result['passed']=all(result['checks'].values())
        (args.output/'result.json').write_text(json.dumps(result,indent=2))
        print(json.dumps(result,indent=2))
        return 0 if result['passed'] else 1
    finally:
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':raise SystemExit(main())
