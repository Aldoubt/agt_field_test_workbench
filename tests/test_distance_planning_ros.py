"""Native Smac planner only, synthetic maps in private DDS; never a controller."""
import math
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


@pytest.fixture
def planner(tmp_path):
    import rclpy
    from rclpy.qos import QoSProfile,DurabilityPolicy
    from rclpy.action import ActionClient
    from nav_msgs.msg import OccupancyGrid
    from geometry_msgs.msg import TransformStamped,PoseStamped
    from tf2_ros import StaticTransformBroadcaster
    from lifecycle_msgs.srv import ChangeState
    from nav2_msgs.action import ComputePathToPose
    from agt_field_test_bringup import profiles
    from workbench.runtime import REPO,ROOT
    profile=profiles.load(ROOT/'config/robots/bunker.yaml','bunker')
    source=REPO.parent/'agt_navigation_v3/config'
    baseline=tmp_path/'baseline.yaml';params=tmp_path/'distance.yaml'
    profiles.navigation(profile,source,baseline)
    profiles.navigation(profile,source,params,'distance')
    tree=yaml.safe_load(params.read_text());p=tree['planner_server']['ros__parameters']
    p['planner_plugins']=['GridBased','Baseline']
    p['Baseline']=yaml.safe_load(baseline.read_text())['planner_server']['ros__parameters']['GridBased']
    params.write_text(yaml.safe_dump(tree))
    rclpy.init(args=[]);node=rclpy.create_node('isolated_distance_path_regression')
    qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    publisher=node.create_publisher(OccupancyGrid,'/map',qos)
    msg=OccupancyGrid();msg.header.frame_id='map';msg.info.resolution=.05
    msg.info.width=500;msg.info.height=240;msg.info.origin.position.x=-5.;msg.info.origin.position.y=-5.
    msg.info.origin.orientation.w=1.
    grid=np.zeros((240,500),dtype=np.int8)
    start=np.array([.329844069,-.197154558]);yaw=-.0958670173
    def set_map(case):
        grid[:]=0
        if case in ('obstacle','unknown'):
            grid[80:105,145:155]=100 if case=='obstacle' else -1
        if case=='no_route':grid[:,145:155]=-1
        msg.data=grid.ravel().tolist()
    set_map('free')
    def publish():
        msg.header.stamp=node.get_clock().now().to_msg();publisher.publish(msg)
    node.create_timer(.05,publish)
    tf=StaticTransformBroadcaster(node);t=TransformStamped();t.header.frame_id='map';t.child_frame_id='base_link'
    t.transform.translation.x=float(start[0]);t.transform.translation.y=float(start[1]);t.transform.rotation.w=1.
    t.header.stamp=node.get_clock().now().to_msg();tf.sendTransform(t)
    def spin(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.02)
    def wait(future,seconds=6.):
        end=time.monotonic()+seconds
        while not future.done() and time.monotonic()<end:
            assert process.poll() is None,log_path.read_text()
            rclpy.spin_once(node,timeout_sec=.02)
        assert future.done(),'isolated planner timed out'
        return future.result()
    log_path=tmp_path/'planner.log';log=log_path.open('w')
    process=subprocess.Popen(['/opt/ros/humble/lib/nav2_planner/planner_server',
        '--ros-args','--params-file',str(params)],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    action=None
    try:
        client=node.create_client(ChangeState,'/planner_server/change_state')
        end=time.monotonic()+6.
        while not client.service_is_ready() and time.monotonic()<end:spin(.05)
        assert client.service_is_ready(),log_path.read_text()
        for transition in (1,3):
            req=ChangeState.Request();req.transition.id=transition
            assert wait(client.call_async(req)).success,log_path.read_text()
        action=ActionClient(node,ComputePathToPose,'/compute_path_to_pose')
        assert action.wait_for_server(timeout_sec=3.)
        def plan(distance,case='free',plugin='GridBased'):
            set_map(case);spin(.65)  # static layer consumes the replacement map
            req=ComputePathToPose.Goal();req.use_start=True;req.planner_id=plugin
            end=start+distance*np.array([math.cos(yaw),math.sin(yaw)])
            for field,xy in [('start',start),('goal',end)]:
                m=PoseStamped();m.header.frame_id='map'
                m.pose.position.x=float(xy[0]);m.pose.position.y=float(xy[1])
                m.pose.orientation.z=math.sin(yaw/2);m.pose.orientation.w=math.cos(yaw/2);setattr(req,field,m)
            begin=time.monotonic();handle=wait(action.send_goal_async(req));assert handle.accepted
            result=wait(handle.get_result_async());elapsed=time.monotonic()-begin
            xy=np.array([[p.pose.position.x,p.pose.position.y] for p in result.result.path.poses])
            return result.status,xy,elapsed,grid.copy(),start,end
        yield plan
    finally:
        if process.poll() is None:os.killpg(process.pid,signal.SIGINT)
        try:process.wait(timeout=4.)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=3.)
        if action is not None:action.destroy()
        node.destroy_node();rclpy.shutdown();log.close()


@pytest.mark.parametrize('distance',[5.,8.,12.])
def test_free_distance_route_removes_grid_bends(planner,distance):
    status,xy,elapsed,_,start,end=planner(distance)
    assert status==4 and len(xy)>2
    normal=np.array([-(end-start)[1],(end-start)[0]])/distance
    deviation=float(np.max(np.abs((xy-start)@normal)))
    assert deviation<.04
    assert elapsed<2.2
    if distance==5.:
        status,old,_,_,_,_=planner(distance,plugin='Baseline')
        assert status==4 and deviation<.5*np.max(np.abs((old-start)@normal))


@pytest.mark.parametrize('case',['obstacle','unknown','no_route'])
def test_distance_smoothing_preserves_obstacle_and_unknown_avoidance(planner,case):
    status,xy,_,grid,start,end=planner(5.,case)
    if case=='no_route':
        assert status==6 and not len(xy)
        return
    assert status==4 and len(xy)>2
    # Check every segment at 1 cm spacing, including between returned poses.
    samples=np.concatenate([a+np.linspace(0,1,max(2,int(np.linalg.norm(b-a)/.01)+1))[:,None]*(b-a)
                            for a,b in zip(xy,xy[1:])])
    ij=np.floor((samples+5.)/.05).astype(int)
    assert np.all((ij>=0)&(ij<np.array([500,240])))
    values=grid[ij[:,1],ij[:,0]]
    assert np.all(values==0),'path cuts through an obstacle or unknown cells'
    normal=np.array([-(end-start)[1],(end-start)[0]])/5.
    assert np.max(np.abs((xy-start)@normal))>.5,'obstructed route must detour'
