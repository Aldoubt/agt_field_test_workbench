"""Single selected robot. Motion-capable chassis processes require explicit opt-in."""
from pathlib import Path
import os
import json
import hashlib

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


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
    if navigation and (robot != 'bunker' or not chassis or not localization):
        raise RuntimeError('Nav2 field profile requires Bunker chassis and localization')
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
        if localization:
            raise RuntimeError('FW-mini MID360 mount TF is not measured; localization blocked')
        if chassis and not Path(value('fwmini_geometry_file')).is_file():
            raise RuntimeError('FW-mini chassis requires a measured fwmini_geometry_file')
        nodes = [_include('fwmini_bringup', 'description.launch.py', {
            'preview': str(not chassis).lower(),
            **({'geometry_file': value('fwmini_geometry_file')} if chassis else {})})]
        if value('enable_mid360').lower() == 'true':
            nodes.append(_include('livox_ros_driver2', 'msg_MID360_launch.py'))
        if chassis:
            # Adapter starts disabled and needs explicit /fwmini/enable_motion.
            nodes.extend([
                _include('yhs_can_control', 'yhs_can_control.launch.py'),
                _include('fwmini_chassis_adapter', 'adapter.launch.py'),
                Node(package='fwmini_odom_adapter', executable='fwmini_odom_adapter_node',
                     name='fwmini_odom_adapter', output='screen'),
            ])
    if localization:
        nodes.append(_include('agt_system_bringup', 'localization.launch.py', {
            'global_map': value('global_map'), 'lio_backend': value('lio_backend'),
            'enable_map_tracking': value('enable_map_tracking'),
            'auto_relocalize': 'false'}))
        nodes.append(Node(package='agt_field_test_bringup', executable='localization_audit.py',
                          name='agt_field_test_localization_audit', output='screen',
                          parameters=[{'global_map': value('global_map'),
                                       'directory': str(Path(os.environ.get(
                                           'AGT_FIELD_TEST_WS', str(Path.home() / 'agt_field_test_ws'))) / 'runs')}]))
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
        OpaqueFunction(function=_configure),
    ])
