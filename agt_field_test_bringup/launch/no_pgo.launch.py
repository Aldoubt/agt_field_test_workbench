"""FAST-LIO2 plus an independent append-only artifact exporter. No PGO nodes."""
from pathlib import Path
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,OpaqueFunction,RegisterEventHandler,EmitEvent,LogInfo
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from agt_field_test_bringup.livox import validate_mid360s_config
import json

def configure(context):
    v=lambda n:LaunchConfiguration(n).perform(context)
    config=Path(v('lio_config'));data=yaml.safe_load(config.read_text());sim=v('use_sim_time')=='true'
    filtered = v('self_filter_enabled') == 'true'
    nodes = [Node(package='agt_field_test_bringup',executable='time_align.py',parameters=[{
        'imu_input':v('imu_topic'),'lidar_input':v('lidar_topic'),'imu_offset_sec':float(v('imu_offset_sec')),
        'lidar_offset_sec':float(v('lidar_offset_sec')),'use_sim_time':sim}]),
        Node(package='agt_mid360_adapter',executable='mid360_adapter_node',parameters=[{'input_topic':'/field/self_filtered/lidar' if filtered else '/field/aligned/lidar'}]),
        Node(package='fastlio2',namespace='fastlio2',executable='lio_node',name='fastlio_mapping',parameters=[{'config_path':str(config),'use_sim_time':sim}],
             remappings=[(data['imu_topic'],'/field/aligned/imu'),(data['lidar_topic'],'/mapping/sensor/livox')]),
        Node(package='agt_field_test_bringup',executable='lio_artifact',parameters=[{
             'output_dir':v('output'),'world_frame':data['world_frame'],'use_sim_time':sim}])]
    if filtered:
        nodes.append(Node(package='agt_field_test_bringup',executable='livox_self_filter',
            name='field_livox_self_filter',parameters=[str(Path(__file__).resolve().parents[1]/'config'/'livox_self_filter.yaml'),
            {'use_sim_time':sim,
             'lio_filter_num':int(data['lidar_filter_num']),
             'lio_min_range_m':float(data['lidar_min_range']),
             'lio_max_range_m':float(data['lidar_max_range'])}]))
    if v('base_adapter')=='true':
        nodes.append(Node(package='agt_fastlio_adapter',executable='fastlio_adapter',name='mapping_base_adapter',parameters=[{
            'input_topic':'/fastlio2/lio_odom','output_topic':'/mapping/base_odometry',
            'expected_odom_frame':data['world_frame'],'expected_base_frame':data['body_frame'],
            'output_odom_frame':data['world_frame'],'output_base_frame':'base_link','tf_child_frame':'base_footprint',
            'convert_body_to_base':True,'body_to_base_calibration_file':str(config),'mount_lidar_frame':'livox_frame',
            'mount_base_frame':'base_link','use_sim_time':sim}]))
    if v('livox_config'):
        identity=validate_mid360s_config(v('livox_config'))
        nodes.extend([LogInfo(msg='Livox startup identity: '+json.dumps(identity,sort_keys=True)),
            Node(package='livox_ros_driver2',executable='livox_ros_driver2_node',parameters=[{
            'xfer_format':1,'multi_topic':0,'data_src':0,'publish_freq':10.,'output_data_type':0,
            'frame_id':'livox_frame','lvx_file_path':'','user_config_path':identity['config_path']}])])
    # Only ROS Node processes have OnProcessExit; LogInfo is a one-shot action.
    # Treating it as a process target can invalidate a live MID360S launch.
    handlers=[RegisterEventHandler(OnProcessExit(target_action=n,on_exit=[
        EmitEvent(event=Shutdown(reason='no-PGO critical node stopped'))]))
        for n in nodes if isinstance(n,Node)]
    return handlers+nodes

def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument(n,default_value=d) for n,d in (
        ('base_adapter','false'),('lio_config',''),('output',''),('use_sim_time','true'),('lidar_topic','/livox/lidar'),
        ('imu_topic','/livox/imu'),('livox_config',''),('imu_offset_sec','0.0'),('lidar_offset_sec','0.0'),('self_filter_enabled','true'))]+[OpaqueFunction(function=configure)])
