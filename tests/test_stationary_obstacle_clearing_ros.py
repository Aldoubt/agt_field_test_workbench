"""Native costmaps in isolated DDS: clearing must use evidence, never a TTL."""
import os
import signal
import subprocess
import time

import numpy as np
import pytest
import yaml

pytestmark=pytest.mark.skipif(
    os.environ.get('ROS_DOMAIN_ID')!='183' or os.environ.get('ROS_LOCALHOST_ONLY')!='1',
    reason='requires isolated domain 183 on localhost')


@pytest.mark.parametrize('mode',['old_voxel','field_2d'])
def test_stationary_person_leaves_and_returns_without_erasing_real_obstacles(tmp_path,mode):
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import OccupancyGrid
    from sensor_msgs.msg import PointCloud2,PointField
    from tf2_ros import TransformBroadcaster
    from lifecycle_msgs.srv import ChangeState
    from agt_field_test_bringup import profiles
    from workbench.runtime import ROOT,REPO
    path=tmp_path/'params.yaml'
    profiles.navigation(profiles.load(ROOT/'config/robots/bunker.yaml','bunker'),
                        REPO.parent/'agt_navigation_v3/config',path)
    tree=yaml.safe_load(path.read_text());local=tree['local_costmap']['local_costmap']['ros__parameters']
    layer=local['obstacle_layer']
    layer['lidar3d_mark']['topic']='/clearing_test/mark'
    layer['lidar3d_clear']['topic']='/clearing_test/clear'
    if mode=='old_voxel':
        # Same observation/height policy as the previous field configuration.
        layer.update(plugin='nav2_costmap_2d::VoxelLayer',origin_z=-3.,z_resolution=.4,
                     z_voxels=16,mark_threshold=0,publish_voxel_map=False)
        local['inflation_layer'].update(inflation_radius=.85,cost_scaling_factor=2.)
    path.write_text(yaml.safe_dump(tree))
    rclpy.init(args=[]);node=rclpy.create_node('isolated_stationary_obstacle_clearing')
    tf=TransformBroadcaster(node);mark=node.create_publisher(PointCloud2,'/clearing_test/mark',10)
    clear=node.create_publisher(PointCloud2,'/clearing_test/clear',10)
    state=dict(person=True,input=True);latest={};observed=[]
    def cloud(points):
        a=np.asarray(points,dtype='<f4').reshape(-1,3)
        m=PointCloud2(height=1,width=len(a),point_step=12,row_step=12*len(a),is_dense=True,
            fields=[PointField(name=k,offset=i*4,datatype=PointField.FLOAT32,count=1)
                    for i,k in enumerate(['x','y','z'])],data=a.tobytes())
        m.header.frame_id='clearing_test_sensor';m.header.stamp=node.get_clock().now().to_msg();return m
    def tick():
        for child,z in [('base_link',-.85),('clearing_test_sensor',-.15)]:
            t=TransformStamped();t.header.frame_id='odom';t.child_frame_id=child
            t.header.stamp=node.get_clock().now().to_msg();t.transform.translation.z=z
            t.transform.rotation.w=1.;tf.sendTransform(t)
        if state['input']:
            # Sensor z=-0.15 in odom: low object is 0.15 m above base ground.
            low=[2.,1.,-.55];person=[1.5,0.,.75]
            mark.publish(cloud(([person] if state['person'] else [])+[low]))
            clear.publish(cloud(([person] if state['person'] else [[3.,0.,-.70]])+[low]))
    node.create_timer(.05,tick)
    def receive(msg):latest['grid']=msg;observed.append(time.monotonic())
    node.create_subscription(OccupancyGrid,'/local_costmap/costmap',receive,qos_profile_sensor_data)
    def spin(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.02)
    def cost(x,y):
        m=latest['grid'];grid=np.asarray(m.data).reshape(m.info.height,m.info.width)
        i=int((x-m.info.origin.position.x)/m.info.resolution)
        j=int((y-m.info.origin.position.y)/m.info.resolution)
        return int(grid[j-1:j+2,i-1:i+2].max())
    log_path=tmp_path/'controller.log';log=log_path.open('w')
    process=subprocess.Popen(['/opt/ros/humble/lib/nav2_controller/controller_server',
        '--ros-args','--params-file',str(path)],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    client=node.create_client(ChangeState,'/controller_server/change_state')
    try:
        deadline=time.monotonic()+6.
        while not client.service_is_ready() and time.monotonic()<deadline:spin(.05)
        assert client.service_is_ready(),log_path.read_text()
        for transition in (1,3):
            request=ChangeState.Request();request.transition.id=transition
            future=client.call_async(request);rclpy.spin_until_future_complete(node,future,timeout_sec=5.)
            assert future.done() and future.result().success,log_path.read_text()
        spin(3.)
        assert cost(1.5,0.)==cost(2.,1.)==100
        # Keep the full sampled 3x3 region beyond the new halo, accounting
        # for grid-cell rounding of the requested inflation radius.
        assert (cost(2.32,0.)>0) if mode=='old_voxel' else (cost(2.32,0.)==0)
        state['person']=False;spin(3.)
        assert cost(1.5,0.)==(100 if mode=='old_voxel' else 0)
        assert cost(2.,1.)==100  # Low real obstacle must remain.
        state['person']=True;spin(1.)
        assert cost(1.5,0.)==100  # Reappearing person is marked again.
        state['input']=False;spin(1.)
        assert cost(1.5,0.)==100  # No sensor evidence: never expire obstacles.
        state.update(person=False,input=True);spin(2.)
        assert cost(1.5,0.)==(100 if mode=='old_voxel' else 0)
        assert cost(2.,1.)==100
        assert len(observed)>40
    finally:
        if process.poll() is None:os.killpg(process.pid,signal.SIGINT)
        try:process.wait(timeout=5.)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=3.)
        node.destroy_node();rclpy.shutdown();log.close()
