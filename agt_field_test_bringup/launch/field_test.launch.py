"""Single selected robot. Motion-capable chassis processes require explicit opt-in."""
from pathlib import Path
import os
import json
import hashlib
import tempfile
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
                            RegisterEventHandler, EmitEvent, LogInfo)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from agt_field_test_bringup import profiles, calibration
from agt_field_test_bringup.fwmini import load_profile, require_mode, robot_description, lio_configs
from agt_field_test_bringup.livox import validate_mid360s_config


def _include(package, filename, args=None):
    path = Path(get_package_share_directory(package)) / 'launch' / filename
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(str(path)),
                                    launch_arguments=(args or {}).items())


def _sha256(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def _mid360s_nodes(config_path):
    identity = validate_mid360s_config(config_path)
    # This is configuration identity only. No serial number or online state is
    # claimed before the driver has communicated with the physical sensor.
    announcement = LogInfo(msg='Livox startup identity: ' + json.dumps(identity, sort_keys=True))
    driver = Node(package='livox_ros_driver2', executable='livox_ros_driver2_node', parameters=[{
        'user_config_path': identity['config_path'], 'xfer_format': 1, 'multi_topic': 0,
        'data_src': 0, 'publish_freq': 10., 'output_data_type': 0,
        'frame_id': 'livox_frame', 'lvx_file_path': ''}])
    return [announcement, driver]


def _configure(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    robot = value('robot')
    if robot not in ('bunker', 'fwmini'):
        raise RuntimeError('robot must be bunker or fwmini')
    workspace = Path(os.environ.get('AGT_FIELD_TEST_WS', str(Path.home()/'agt_field_test_ws')))
    robot_profile = profiles.load(value('robot_profile_file') or profiles.selected_path(workspace,robot),robot)
    global_backend = value('global_backend')
    if global_backend not in ('bbs', 'kiss'):
        raise RuntimeError('global_backend must be bbs or kiss')
    chassis = value('enable_chassis').lower() == 'true'
    localization = value('enable_localization').lower() == 'true'
    navigation = value('enable_navigation').lower() == 'true'
    description_enabled = value('enable_description').lower() == 'true'
    guard_enabled = value('enable_guard').lower() == 'true'
    external_chassis = value('external_chassis').lower() == 'true'
    external_localization = value('external_localization').lower() == 'true'
    bundle=calibration.load(value('calibration_bundle'),robot) if value('calibration_bundle') else None
    profile = None
    if robot == 'fwmini':
        profile = load_profile(value('fwmini_mount_file'))
        mode = value('fwmini_mode')
        if localization and mode != 'supervised_localization' and not navigation:
            raise RuntimeError('FW-mini localization requires fwmini_mode:=supervised_localization')
        require_mode(profile, 'autonomous_navigation' if navigation else mode,
                     supervised=value('supervised').lower() == 'true')
    if navigation and not localization and not external_localization:
        raise RuntimeError('navigation requires the localization stack')
    if localization and external_localization:
        raise RuntimeError('do not start a second localization/TF owner')
    if navigation and not chassis and not external_chassis:
        raise RuntimeError('navigation requires a chassis in this launch or an independently supervised chassis component')
    if navigation and chassis and external_chassis:
        raise RuntimeError('do not start a second chassis when external_chassis:=true')
    if localization and not Path(value('global_map')).is_file():
        raise RuntimeError('enable_localization requires an existing global_map PCD')
    if navigation:
        from pathlib import Path as _P
        import sys
        sys.path.insert(0,str(workspace/'src/agt_field_test_workbench'))
        from workbench.safety import report as safety_report
        nav_map=json.loads((Path(value('map_yaml')).parent/'manifest.json').read_text())
        safety_report(value('safety_report'),robot,robot_profile['profile_sha256'],nav_map['sha256'],
                      calibration.digest(bundle) if bundle else __import__('agt_field_test_bringup.fwmini',fromlist=['digest']).digest(profile))
        if (bundle or profile)['status']!='VERIFIED':raise RuntimeError('VERIFIED mounting calibration required for navigation')
    chassis_nodes=[]
    description_nodes=[]
    if robot == 'bunker':
        if description_enabled:
            description_nodes = ([Node(package='robot_state_publisher',executable='robot_state_publisher',parameters=[{'robot_description':calibration.description(bundle,value('lio_backend'))}])]
                 if bundle else [_include('tracked_chassis_description','display.launch.py',{'start_rviz':'false'})])
        nodes=list(description_nodes)
        if value('enable_mid360').lower() == 'true':
            nodes.extend(_mid360s_nodes(value('livox_config')))
        if chassis:
            driver=Node(package='agt_bunker_base', executable='agt_bunker_base_node', parameters=[
                str(Path(get_package_share_directory('agt_bunker_base'))/'config/bunker.yaml'),
                {'port_name':robot_profile['can_interface'],'cmd_vel_topic':robot_profile['topics']['driver'],
                 'odom_topic_name':robot_profile['wheel_odom_topic'],'publish_odom_tf':False,
                 'control_rate':int(robot_profile['control_hz'])}])
            chassis_nodes.append(driver);nodes.append(driver)
            if not navigation and guard_enabled:
                nodes.append(Node(package='agt_base_control', executable='cmd_vel_guard',
                                  name='agt_cmd_vel_guard', output='screen', parameters=[profiles.guard(robot_profile)]))
    else:
        if chassis and not Path(value('fwmini_geometry_file')).is_file():
            raise RuntimeError('FW-mini chassis requires a measured fwmini_geometry_file')
        if chassis:
            geometry = yaml.safe_load(Path(value('fwmini_geometry_file')).read_text())
            if geometry['body'].get('measured') is not True or any(
                    abs(geometry['body'][key]-profile['body'][key]) > 1e-6
                    for key in ('length_m', 'width_m', 'height_m')):
                raise RuntimeError('measured FW-mini geometry must match selected mount profile')
        # One RSP owns all physical mounting edges. Do not include the old
        # description launch or a second base_link -> livox static publisher.
        description_nodes = [Node(package='robot_state_publisher', executable='robot_state_publisher',
                      name='fwmini_robot_state_publisher', output='screen', parameters=[{
                          'robot_description': robot_description(profile, value('fwmini_internal_config') or None),
                          'use_sim_time': value('use_sim_time').lower() == 'true'}])] if description_enabled else []
        nodes=list(description_nodes)
        if value('enable_mid360').lower() == 'true':
            nodes.extend(_mid360s_nodes(value('livox_config')))
        if chassis:
            # Adapter starts disabled and needs explicit /fwmini/enable_motion.
            if not navigation and guard_enabled:
                nodes.append(Node(package='agt_base_control',executable='cmd_vel_guard',name='agt_cmd_vel_guard',parameters=[profiles.guard(robot_profile)]))
            chassis_nodes=[
                Node(package='yhs_can_control',executable='yhs_can_control_node',name='yhs_can_control_node',parameters=[str(Path(get_package_share_directory('yhs_can_control'))/'params/cfg.yaml'),{'can_name':robot_profile['can_interface']}]),
                Node(package='fwmini_chassis_adapter', executable='fwmini_chassis_adapter_node',
                     parameters=[{'max_linear_x':robot_profile['forward_mps'], 'max_linear_y':robot_profile['lateral_mps'],
                                  'max_angular_z':robot_profile['angular_radps'], 'cmd_timeout':robot_profile['command_timeout_sec']}]),
                Node(package='fwmini_odom_adapter', executable='fwmini_odom_adapter_node',
                     name='fwmini_odom_adapter', output='screen',
                     parameters=[{'publish_tf': False}]),
            ]
            nodes.extend(chassis_nodes)
    localization_nodes = []
    if localization:
        localization_args = {
            'global_backend': global_backend,
            'global_map': value('global_map'), 'lio_backend': value('lio_backend'),
            'relocalization_assets': value('relocalization_assets'),
            'enable_map_tracking': value('enable_map_tracking'),
            'auto_relocalize': 'false', 'use_sim_time': value('use_sim_time')}
        if profile:
            if value('fwmini_internal_config'):
                raise RuntimeError('localization uses selected profile LiDAR/IMU values; internal override is mapping-only')
            fast, batch = lio_configs(profile, get_package_share_directory('agt_navigation_runtime'),
                                     tempfile.mkdtemp(prefix='fwmini_lio_'))
            localization_args.update(fastlio_config=fast, batch_config=batch)
        if bundle:
            fast,batch=calibration.configs(bundle,get_package_share_directory('agt_navigation_runtime'),
                workspace/'src/agt-lio-pgo-mapping/bringup/agt_mapping_bringup/config/fastlio2_mid360.yaml', tempfile.mkdtemp(prefix='field_cal_'))
            imu_offset = float(bundle['time_sync']['imu_offset_sec'])
            lidar_offset = float(bundle['time_sync']['lidar_offset_sec'])
            align_imu = int(imu_offset * 1e9) != 0
            align_lidar = int(lidar_offset * 1e9) != 0
            aligned_imu_topic = '/field/aligned/imu' if align_imu else '/livox/imu'
            aligned_lidar_topic = '/field/aligned/lidar' if align_lidar else '/livox/lidar'
            localization_args.update(fastlio_config=fast,batch_config=batch,
                                     lidar_topic=aligned_lidar_topic,imu_topic=aligned_imu_topic)
            # A zero-offset candidate must not route full LiDAR scans through
            # the timestamp relay. Only relay streams that require correction.
            if align_imu or align_lidar:
                localization_nodes.append(Node(
                    package='agt_field_test_bringup', executable='time_align.py',
                    parameters=[bundle['time_sync'], {
                        'align_imu': align_imu, 'align_lidar': align_lidar}]))
        if value('gps_alignment_file'):
            from agt_field_test_bringup.gps import load_alignment
            map_sha=json.loads((Path(value('map_yaml')).parent/'manifest.json').read_text())['sha256']
            cal_sha=calibration.digest(bundle) if bundle else __import__('agt_field_test_bringup.fwmini',fromlist=['digest']).digest(profile)
            load_alignment(value('gps_alignment_file'),map_sha,cal_sha)
            localization_args['global_pose_output']='/field/relocalization/raw_pose'
            localization_nodes.append(Node(package='agt_field_test_bringup',executable='gps_candidate_gate.py',parameters=[{
                'alignment_file':value('gps_alignment_file'),'map_sha256':map_sha,'calibration_sha256':cal_sha,
                'use_sim_time':value('use_sim_time')=='true'}]))
        localization_nodes.append(_include('agt_system_bringup', 'localization.launch.py', localization_args))
        localization_nodes.append(Node(package='agt_field_test_bringup', executable='localization_audit.py',
                          name='agt_field_test_localization_audit', output='screen',
                          parameters=[{'global_map': value('global_map'),
                                       'directory': str(Path(os.environ.get(
                                           'AGT_FIELD_TEST_WS', str(Path.home() / 'agt_field_test_ws'))) / 'runs')}]))
    if profile or bundle:
        if profile and value('fwmini_mode') in ('sensor_mapping', 'offline_mapping'):
            lio_config = value('fwmini_internal_config') or str(
                Path(get_package_share_directory('agt_mapping_bringup'))/'config/fastlio2_mid360.yaml')
            lio_data = yaml.safe_load(Path(lio_config).read_text())
            localization_nodes.append(Node(package='agt_fastlio_adapter', executable='fastlio_adapter',
                name='fwmini_mapping_base_adapter', output='screen', parameters=[{
                    'input_topic': '/fastlio2/lio_odom', 'output_topic': '/mapping/fwmini/base_odometry',
                    'expected_odom_frame': lio_data['world_frame'], 'expected_base_frame': lio_data['body_frame'],
                    'output_odom_frame': lio_data['world_frame'], 'output_base_frame': 'base_link',
                    'tf_child_frame': 'base_footprint', 'convert_body_to_base': True,
                    'body_to_base_calibration_file': lio_config,
                    'mount_lidar_frame': 'livox_frame', 'mount_base_frame': 'base_link',
                    'use_sim_time': value('use_sim_time').lower() == 'true'}]))
        arguments = (['--bundle',value('calibration_bundle'),'--backend',value('lio_backend')] if bundle else
                     ['--profile',str(Path(value('fwmini_mount_file')).expanduser().resolve())])
        if value('fwmini_internal_config'):
            arguments += ['--internal-config', value('fwmini_internal_config')]
        preflight_args = arguments + ['--preflight']
        if value('fwmini_tf_report'):
            preflight_args += ['--report', value('fwmini_tf_report')]
        preflight = Node(package='agt_field_test_bringup', executable='fwmini_tf_contract.py',
                         name='fwmini_tf_preflight', output='screen', arguments=preflight_args)
        watcher = Node(package='agt_field_test_bringup', executable='fwmini_tf_contract.py',
                       name='fwmini_tf_watch', output='screen', arguments=arguments)

        def after_tf(event, _context):
            if event.returncode != 0:
                return [EmitEvent(event=Shutdown(reason='FW-mini TF preflight failed'))]
            return [LogInfo(msg=f"{robot} {(profile or bundle)['status']}: selected TF checked"),
                    *([watcher] if description_enabled else []), *localization_nodes]

        def guard_exit(event, _context):
            return [EmitEvent(event=Shutdown(reason='FW-mini TF ownership violation'))] if event.returncode else []

        handlers=[RegisterEventHandler(OnProcessExit(target_action=preflight, on_exit=after_tf))]
        if description_enabled:
            handlers.append(RegisterEventHandler(OnProcessExit(target_action=watcher, on_exit=guard_exit)))
        if description_nodes:
            handlers.append(RegisterEventHandler(OnProcessExit(target_action=description_nodes[0], on_exit=[
                EmitEvent(event=Shutdown(reason='robot description stopped'))])))
        nodes.extend([*handlers, preflight])
    else:
        nodes.extend(localization_nodes)
    if chassis_nodes:
        def chassis_exit(event, _context):
            return [EmitEvent(event=Shutdown(reason='chassis component process exited'))]
        for chassis_node in chassis_nodes:
            nodes.append(RegisterEventHandler(OnProcessExit(target_action=chassis_node,on_exit=chassis_exit)))
    if navigation:
        map_yaml = Path(value('map_yaml')).expanduser().resolve()
        manifest_path = map_yaml.parent / 'manifest.json'
        if map_yaml.name != 'map.yaml' or not manifest_path.is_file():
            raise RuntimeError('navigation map must be a versioned Map Studio map.yaml')
        manifest = json.loads(manifest_path.read_text())
        if manifest.get('status') != 'ACCEPTED':
            raise RuntimeError('navigation map is not ACCEPTED')
        global_map = Path(value('global_map')).expanduser().resolve()
        if not manifest.get('source_pcd') or Path(manifest['source_pcd']).resolve() != global_map:
            raise RuntimeError('Nav2 map and localization PCD identities differ')
        if _sha256(global_map) != manifest.get('source_pcd_sha256'):
            raise RuntimeError('localization PCD SHA-256 differs from Map Studio provenance')
        runtime_params = profiles.navigation(robot_profile, Path(get_package_share_directory('agt_system_bringup'))/'config',
            Path(tempfile.mkdtemp(prefix='field_profile_'))/'navigation.yaml')
        if value('enable_camera_gimbal').lower() == 'true':
            nodes.append(_include('autolabor_c1_bringup', 'autolabor_c1.launch.py', {
                'device_path': value('camera_device_path'),
                'port_name': value('gimbal_port_name'), 'gui': 'false'}))
        nodes.append(_include('agt_system_bringup', 'navigation.launch.py', {
            'runtime_params_file':runtime_params,
            'map': str(map_yaml), 'map_id': manifest['id'], 'enable_inspection': 'false'}))
        if value('enable_inspection').lower() == 'true':
            # Reuse only the headless V3 mission executor, not its RViz patrol UI.
            runtime_share=Path(get_package_share_directory('agt_navigation_runtime'))
            nodes.append(Node(package='agt_navigation_runtime',executable='mission_runtime',
                name='mission_runtime',output='screen',parameters=[str(runtime_share/'config/runtime.yaml'),
                    {'record_root':str(workspace/'data/captures/missions')}]))
    return nodes


def generate_launch_description():
    workspace = Path(os.environ.get('AGT_FIELD_TEST_WS', str(Path.home() / 'agt_field_test_ws')))
    mount = workspace / 'calibration/fwmini/fwmini_mid360_candidate.yaml'
    if not mount.is_file():
        mount = Path(get_package_share_directory('agt_field_test_bringup')) / 'config/fwmini_mid360_candidate.yaml'
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='bunker'),
        DeclareLaunchArgument('global_backend', default_value='bbs'),
        DeclareLaunchArgument('relocalization_assets', default_value=''),
        DeclareLaunchArgument('enable_inspection', default_value='true'),
        DeclareLaunchArgument('enable_camera_gimbal', default_value='false'),
        DeclareLaunchArgument('camera_device_path', default_value='/dev/video0'),
        DeclareLaunchArgument('gimbal_port_name', default_value='/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0'),
        DeclareLaunchArgument('gps_alignment_file', default_value=''),
        DeclareLaunchArgument('robot_profile_file', default_value=''),
        DeclareLaunchArgument('calibration_bundle', default_value=''),
        DeclareLaunchArgument('safety_report', default_value=''),
        DeclareLaunchArgument('livox_config', default_value=str(
            Path(get_package_share_directory('agt_field_test_bringup')) / 'config/MID360s_config.json')),
        DeclareLaunchArgument('enable_chassis', default_value='false'),
        DeclareLaunchArgument('enable_description', default_value='true'),
        DeclareLaunchArgument('enable_guard', default_value='true'),
        DeclareLaunchArgument('external_chassis', default_value='false'),
        DeclareLaunchArgument('external_localization', default_value='false'),
        DeclareLaunchArgument('enable_mid360', default_value='true'),
        DeclareLaunchArgument('enable_localization', default_value='false'),
        DeclareLaunchArgument('enable_navigation', default_value='false'),
        DeclareLaunchArgument('global_map', default_value=''),
        DeclareLaunchArgument('map_yaml', default_value=''),
        DeclareLaunchArgument('lio_backend', default_value='batch_lio'),
        DeclareLaunchArgument('enable_map_tracking', default_value='true'),
        DeclareLaunchArgument('fwmini_geometry_file', default_value=''),
        DeclareLaunchArgument('fwmini_mount_file', default_value=str(mount)),
        DeclareLaunchArgument('fwmini_mode', default_value='sensor_test'),
        DeclareLaunchArgument('supervised', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('fwmini_internal_config', default_value=''),
        DeclareLaunchArgument('fwmini_tf_report', default_value=''),
        OpaqueFunction(function=_configure),
    ])
