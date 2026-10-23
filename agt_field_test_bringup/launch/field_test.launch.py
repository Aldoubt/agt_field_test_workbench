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
from agt_field_test_bringup.fwmini import load_profile, require_mode, robot_description, lio_configs


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


def _configure(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    robot = value('robot')
    if robot not in ('bunker', 'fwmini'):
        raise RuntimeError('robot must be bunker or fwmini')
    chassis = value('enable_chassis').lower() == 'true'
    localization = value('enable_localization').lower() == 'true'
    navigation = value('enable_navigation').lower() == 'true'
    profile = None
    if robot == 'fwmini':
        profile = load_profile(value('fwmini_mount_file'))
        mode = value('fwmini_mode')
        if localization and mode != 'supervised_localization' and not navigation:
            raise RuntimeError('FW-mini localization requires fwmini_mode:=supervised_localization')
        require_mode(profile, 'autonomous_navigation' if navigation else mode,
                     supervised=value('supervised').lower() == 'true')
    if navigation and (robot != 'bunker' or not chassis or not localization):
        raise RuntimeError('Formal Nav2 currently requires the accepted Bunker profile; FW-mini needs independent Nav2 geometry/localization/motion acceptance')
    if localization and not Path(value('global_map')).is_file():
        raise RuntimeError('enable_localization requires an existing global_map PCD')
    if robot == 'bunker':
        nodes = [_include('tracked_chassis_description', 'display.launch.py',
                          {'start_rviz': 'false'})]
        if value('enable_mid360').lower() == 'true':
            nodes.append(_include('livox_ros_driver2', 'msg_MID360_launch.py'))
        if chassis:
            nodes.append(_include('agt_bunker_base', 'bunker_base.launch.py'))
            if not navigation:
                # Manual low-speed trials remain possible without global
                # localization. The guard still enforces timeout and limits.
                nodes.append(Node(package='agt_base_control', executable='cmd_vel_guard',
                                  name='agt_cmd_vel_guard', output='screen', parameters=[{
                                      'default_control_mode': 'manual',
                                      'require_localization_status': False,
                                      'max_linear_x': 0.05, 'max_reverse_x': 0.03,
                                      'max_angular_z': 0.15,
                                      'command_timeout_sec': 0.20,
                                  }]))
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
        nodes = [Node(package='robot_state_publisher', executable='robot_state_publisher',
                      name='fwmini_robot_state_publisher', output='screen', parameters=[{
                          'robot_description': robot_description(profile, value('fwmini_internal_config') or None),
                          'use_sim_time': value('use_sim_time').lower() == 'true'}])]
        if value('enable_mid360').lower() == 'true':
            nodes.append(_include('livox_ros_driver2', 'msg_MID360_launch.py'))
        if chassis:
            # Adapter starts disabled and needs explicit /fwmini/enable_motion.
            nodes.extend([
                _include('yhs_can_control', 'yhs_can_control.launch.py'),
                Node(package='agt_base_control', executable='cmd_vel_guard',
                     name='agt_cmd_vel_guard', output='screen', parameters=[{
                         'default_control_mode': 'manual',
                         'require_localization_status': False,
                         'output_topic': '/cmd_vel',
                         'max_linear_x': 0.05, 'max_reverse_x': 0.03,
                         'max_angular_z': 0.15,
                         'command_timeout_sec': 0.20,
                     }]),
                _include('fwmini_chassis_adapter', 'adapter.launch.py'),
                Node(package='fwmini_odom_adapter', executable='fwmini_odom_adapter_node',
                     name='fwmini_odom_adapter', output='screen',
                     parameters=[{'publish_tf': False}]),
            ])
    localization_nodes = []
    if localization:
        localization_args = {
            'global_map': value('global_map'), 'lio_backend': value('lio_backend'),
            'enable_map_tracking': value('enable_map_tracking'),
            'auto_relocalize': 'false', 'use_sim_time': value('use_sim_time')}
        if profile:
            if value('fwmini_internal_config'):
                raise RuntimeError('localization uses selected profile LiDAR/IMU values; internal override is mapping-only')
            fast, batch = lio_configs(profile, get_package_share_directory('agt_navigation_runtime'),
                                     tempfile.mkdtemp(prefix='fwmini_lio_'))
            localization_args.update(fastlio_config=fast, batch_config=batch)
        localization_nodes.append(_include('agt_system_bringup', 'localization.launch.py', localization_args))
        localization_nodes.append(Node(package='agt_field_test_bringup', executable='localization_audit.py',
                          name='agt_field_test_localization_audit', output='screen',
                          parameters=[{'global_map': value('global_map'),
                                       'directory': str(Path(os.environ.get(
                                           'AGT_FIELD_TEST_WS', str(Path.home() / 'agt_field_test_ws'))) / 'runs')}]))
    if profile:
        if value('fwmini_mode') in ('sensor_mapping', 'offline_mapping'):
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
        arguments = ['--profile', str(Path(value('fwmini_mount_file')).expanduser().resolve())]
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
            return [LogInfo(msg=f"FW-mini {profile['status']}: TF checked; observation mode only"),
                    watcher, *localization_nodes]

        def guard_exit(event, _context):
            return [EmitEvent(event=Shutdown(reason='FW-mini TF ownership violation'))] if event.returncode else []

        nodes.extend([RegisterEventHandler(OnProcessExit(target_action=preflight, on_exit=after_tf)),
                      RegisterEventHandler(OnProcessExit(target_action=nodes[0], on_exit=[
                          EmitEvent(event=Shutdown(reason='FW-mini robot description stopped'))])),
                      RegisterEventHandler(OnProcessExit(target_action=watcher, on_exit=guard_exit)), preflight])
    else:
        nodes.extend(localization_nodes)
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
        nodes.append(_include('agt_system_bringup', 'navigation.launch.py', {
            'map': str(map_yaml), 'map_id': manifest['id'], 'enable_inspection': 'false'}))
    return nodes


def generate_launch_description():
    workspace = Path(os.environ.get('AGT_FIELD_TEST_WS', str(Path.home() / 'agt_field_test_ws')))
    mount = workspace / 'calibration/fwmini/fwmini_mid360_candidate.yaml'
    if not mount.is_file():
        mount = Path(get_package_share_directory('agt_field_test_bringup')) / 'config/fwmini_mid360_candidate.yaml'
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='bunker'),
        DeclareLaunchArgument('enable_chassis', default_value='false'),
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
