"""Actual Humble runtime/services/TF; synthetic scans, no matcher mocks."""
import json
import os
import threading
import time
import numpy as np
import pytest
import yaml
pytestmark=pytest.mark.skipif(os.environ.get('AGT_RUN_SC_ROS')!='1',reason='requires built Humble experiment overlay')


def wait_for(predicate,timeout=45):
    end=time.monotonic()+timeout
    while time.monotonic()<end:
        if predicate():return
        time.sleep(.025)
    raise AssertionError('ROS condition timeout')


@pytest.mark.parametrize('mode',['sc_kiss','bbs'])
def test_runtime_manager(mode,tmp_path):
    from scipy.spatial.transform import Rotation
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import TransformStamped,PoseWithCovarianceStamped
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import PointCloud2,PointField
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from tf2_ros import TransformBroadcaster,StaticTransformBroadcaster
    from agt_robot_interfaces.msg import LocalizationStatus
    from agt_global_relocalization.global_relocalization import GlobalRelocalization
    from agt_localization_manager.localization_manager import LocalizationManager
    from synthetic import fixture
    data=fixture(tmp_path/'fixture')
    if mode=='bbs':
        # Preserve the baseline's existing level-query operating contract.
        data['rotation']=Rotation.identity()
        data['points']=data['map_points']-data['translation']
    r_bl=Rotation.from_matrix(data['spec']['body_from_lidar_rotation']);t_bl=np.array(data['spec']['body_from_lidar_translation'])
    r_vl=Rotation.from_euler('xyz',[0.,.23,0.]);t_vl=np.array([.45,0.,.7])
    r_bv=r_bl*r_vl.inv();t_bv=t_bl-r_bv.apply(t_vl)
    roll,pitch,_=data['rotation'].as_euler('xyz')
    r_ob=Rotation.from_euler('xyz',[roll,pitch,0.]);r_ov=r_ob*r_bv;t_ov=r_ob.apply(t_bv)
    sensor=(data['points']-t_bl)@r_bl.as_matrix()
    cfg=dict(global_backend=mode,sc_database=str(data['database']),global_map=str(data['map']),
        follow_map_manager=False,body_to_base_calibration_file=str(data['config']),
        stationary_odom_topic='/wheel/odom',work_dir=str(tmp_path/'work'),sdk_timeout_sec=30.,
        mapping_body_livox_translation=t_bl.tolist(),mapping_body_livox_quaternion_xyzw=r_bl.as_quat().tolist(),
        sdk_command='ros2 run agt_global_relocalization_native bbs_gicp_localizer --map {global_map} --scan {scan_pcd} --timeout {timeout_sec}')
    if mode=='bbs':
        # Exercise the existing production Polar Context -> BBS -> GICP path.
        import subprocess
        assets=tmp_path/'bbs_assets'
        subprocess.run(['ros2','run','agt_global_relocalization_native','build_relocalization_assets',
            '--map',str(data['map']),'--output',str(assets)],check=True)
        subprocess.run(['ros2','run','agt_global_relocalization_native','build_relocalization_candidates',
            '--map-dir',str(data['keyframes']),'--output',str(assets)],check=True)
        cfg['relocalization_assets']=str(assets)
        cfg['candidate_sdk_command']=('ros2 run agt_global_relocalization_native candidate_bbs_gicp_localizer '
            '--map {global_map} --scan {scan_pcd} {assets_arg} --timeout {timeout_sec} '
            '--candidate-top-k 4 --descriptor-prefilter 80 --candidate-xy-radius 5.0 --candidate-z-radius 2.5 '
            '--candidate-yaw-range-deg 0.0 --roll-pitch-range-deg 0.0 '
            '--bbs-query-frame-mode {bbs_query_frame_mode} --per-candidate-timeout 6.0 --bbs-score-threshold 0.03 --gicp-max-corr 2.5')
    param=tmp_path/'params.yaml';param.write_text(yaml.safe_dump({'agt_global_relocalization':{'ros__parameters':cfg}}))
    rclpy.init(args=['--ros-args','--params-file',str(param)])
    runtime=GlobalRelocalization();manager=LocalizationManager();probe=Node('sc_experiment_probe')
    executor=MultiThreadedExecutor(num_threads=4);nodes=[runtime,manager,probe]
    for node in nodes:executor.add_node(node)
    poses=[];statuses=[];public=[];ticks=[];moving=[False]
    probe.create_subscription(PoseWithCovarianceStamped,'/agt/relocalization/pose',poses.append,10)
    probe.create_subscription(String,'/agt/global_relocalization/status',lambda m:statuses.append(json.loads(m.data)),30)
    probe.create_subscription(LocalizationStatus,'/agt/localization/status',public.append,30)
    odom_pub=probe.create_publisher(Odometry,'/agt/odometry/local',20);wheel_pub=probe.create_publisher(Odometry,'/wheel/odom',20)
    cloud_pub=probe.create_publisher(PointCloud2,'/agt/livox/points',qos_profile_sensor_data)
    broadcaster=TransformBroadcaster(probe);static=StaticTransformBroadcaster(probe)
    def fill(transform,t,q):
        transform.translation.x,transform.translation.y,transform.translation.z=map(float,t)
        transform.rotation.x,transform.rotation.y,transform.rotation.z,transform.rotation.w=map(float,q)
    mount=TransformStamped();mount.header.frame_id='base_link';mount.child_frame_id='livox_frame'
    fill(mount.transform,t_vl,r_vl.as_quat());static.sendTransform(mount)
    def publish():
        stamp=probe.get_clock().now().to_msg();ticks.append(time.monotonic())
        odom=Odometry();odom.header.frame_id='odom';odom.child_frame_id='base_link';odom.header.stamp=stamp
        odom.pose.pose.position.x,odom.pose.pose.position.y,odom.pose.pose.position.z=map(float,t_ov)
        q=r_ov.as_quat();odom.pose.pose.orientation.x,odom.pose.pose.orientation.y,odom.pose.pose.orientation.z,odom.pose.pose.orientation.w=map(float,q)
        odom_pub.publish(odom)
        wheel=Odometry();wheel.header.stamp=stamp;wheel.header.frame_id='odom';wheel.child_frame_id='base_link';wheel.pose.pose.orientation.w=1.;wheel.twist.twist.linear.x=.5 if moving[0] else 0.;wheel_pub.publish(wheel)
        tf=TransformStamped();tf.header=odom.header;tf.child_frame_id='base_link';fill(tf.transform,t_ov,q);broadcaster.sendTransform(tf)
        cloud=PointCloud2();cloud.header.stamp=stamp;cloud.header.frame_id='livox_frame';cloud.height=1;cloud.width=len(sensor)
        cloud.fields=[PointField(name=n,offset=i*4,datatype=PointField.FLOAT32,count=1) for i,n in enumerate('xyz')]
        cloud.point_step=12;cloud.row_step=12*len(sensor);cloud.is_dense=True;cloud.data=np.asarray(sensor,dtype='<f4').tobytes();cloud_pub.publish(cloud)
    probe.create_timer(.05,publish);client=probe.create_client(Trigger,'/agt/localization/relocalize')
    spinner=threading.Thread(target=executor.spin,daemon=True);spinner.start()
    try:
        wait_for(lambda:len(runtime.clouds)>=5 and len(manager._odom)>5)
        assert client.wait_for_service(timeout_sec=3.)
        if mode=='sc_kiss':
            runtime.set_parameters([Parameter('sc_database',value=str(tmp_path/'missing_database'))])
            call=client.call_async(Trigger.Request());wait_for(call.done);assert call.result().success
            wait_for(lambda:any(s['state']=='REJECTED' for s in statuses))
            wait_for(lambda:public and not public[-1].global_correction_valid)
            assert not poses and manager._correction_current is None
            runtime.set_parameters([Parameter('sc_database',value=str(data['database']))])
        begin=len(ticks);call=client.call_async(Trigger.Request());wait_for(call.done);assert call.result().success
        wait_for(lambda:any(s['state']=='SUCCEEDED' for s in statuses))
        wait_for(lambda:public and public[-1].global_correction_valid)
        assert poses and manager._correction_current is not None and len(ticks)>begin+3
        msg=poses[-1].pose.pose;p=np.array([msg.position.x,msg.position.y,msg.position.z])
        assert np.linalg.norm(p-(data['translation']+data['rotation'].apply(t_bv)))<.25
        q=Rotation.from_quat([msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w])
        assert (q*(data['rotation']*r_bv).inv()).magnitude()<np.radians(5)
        success=[s for s in statuses if s['state']=='SUCCEEDED'][-1]
        assert success['global_backend']==mode and success['fitness']<=1 and success['overlap']>=.2
        if mode=='sc_kiss':assert success['kiss_final_inliers']>=20 and success['sc_candidates']
        correction=manager._correction_current
        import copy
        bad=copy.deepcopy(poses[-1]);bad.pose.covariance[0]=100.;manager._on_global_pose(bad)
        assert manager._correction_current==correction
        assert not any('/cmd_vel' in name for name,_ in probe.get_publisher_names_and_types_by_node(runtime.get_name(),runtime.get_namespace()))
        if mode=='sc_kiss':
            previous=len(poses);old_status=len(statuses)
            call=client.call_async(Trigger.Request());wait_for(call.done);assert call.result().success
            wait_for(lambda:any(s['state']=='SC_KISS_SEARCHING' for s in statuses[old_status:]))
            moving[0]=True
            wait_for(lambda:any(s['state']=='REJECTED' for s in statuses[old_status:]))
            assert len(poses)==previous
            assert manager._correction_current is None

    finally:
        executor.shutdown(timeout_sec=3);spinner.join(timeout=3)
        for node in nodes:node.destroy_node()
        rclpy.shutdown()


def test_backend_timeout_reaps_real_child_processes(tmp_path):
    import rclpy
    import shlex
    import sys
    from agt_global_relocalization.global_relocalization import GlobalRelocalization
    child=tmp_path/'child.pid'
    script=tmp_path/'spawn.py'
    script.write_text("import subprocess,sys,time\nfrom pathlib import Path\np=subprocess.Popen([sys.executable,'-c','import time;time.sleep(60)'])\nPath(sys.argv[1]).write_text(str(p.pid))\ntime.sleep(60)\n")
    rclpy.init(args=[]);node=GlobalRelocalization()
    try:
        with pytest.raises(TimeoutError,match='backend timeout'):
            node._run_backend(shlex.join([sys.executable,str(script),str(child)]),.5)
        pid=int(child.read_text());status=__import__('pathlib').Path(f'/proc/{pid}/stat')
        assert not status.exists() or status.read_text().split()[2]=='Z'
        assert node._backend_process is None
    finally:node.destroy_node();rclpy.shutdown()


@pytest.mark.parametrize('mode',['bbs','sc_kiss'])
def test_launch_argument_reaches_runtime_parameter(mode,tmp_path):
    import rclpy
    import subprocess
    import signal
    from rclpy.node import Node
    from rcl_interfaces.srv import GetParameters
    log=(tmp_path/'launch.log').open('w')
    proc=subprocess.Popen(['ros2','launch','agt_global_relocalization','global_relocalization.launch.py',
        'global_backend:='+mode,'sc_database:='+str(tmp_path/'database'),'auto_request:=false'],
        stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    rclpy.init(args=[]);probe=Node('experiment_launch_parameter_probe')
    client=probe.create_client(GetParameters,'/agt_global_relocalization/get_parameters')
    try:
        assert client.wait_for_service(timeout_sec=10), (tmp_path/'launch.log').read_text()
        request=GetParameters.Request();request.names=['global_backend','sc_database']
        future=client.call_async(request);rclpy.spin_until_future_complete(probe,future,timeout_sec=5)
        assert future.done() and future.result().values[0].string_value==mode
        assert future.result().values[1].string_value==str(tmp_path/'database')
    finally:
        probe.destroy_node();rclpy.shutdown()
        try:os.killpg(proc.pid,signal.SIGINT)
        except ProcessLookupError:pass
        try:proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid,signal.SIGKILL);proc.wait(timeout=3)
        log.close()
