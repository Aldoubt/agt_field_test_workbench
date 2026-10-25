"""One robot profile feeds UI, guard, driver, controller and smoother."""
import copy
import hashlib
import json
import math
from pathlib import Path
import yaml


def load(path, robot=None):
    path = Path(path).expanduser().resolve()
    p = yaml.safe_load(path.read_text())
    if p['robot'] not in ('bunker', 'fwmini') or (robot and p['robot'] != robot):
        raise ValueError('robot/profile mismatch')
    for key in ('forward_mps', 'reverse_mps', 'angular_radps', 'linear_accel_mps2', 'linear_decel_mps2'):
        if isinstance(p[key], bool) or not math.isfinite(p[key]) or p[key] <= 0:
            raise ValueError('invalid motion limit: '+key)
    p.setdefault('angular_accel_radps2', .3)
    p.setdefault('angular_decel_radps2', .4)
    p.setdefault('lateral_mps', 0.0)
    p.setdefault('command_timeout_sec', .2)
    p.setdefault('local_costmap_update_hz', 10.0)
    p.setdefault('local_costmap_publish_hz', 5.0)
    p.setdefault('navigation_self_mask_boxes', [])
    boxes = p['navigation_self_mask_boxes']
    if (not isinstance(boxes, list) or len(boxes) % 6 or
            any(isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value) for value in boxes) or
            any(boxes[i]>=boxes[i+1] for i in range(0,len(boxes),2))):
        raise ValueError('invalid navigation self mask bounds')
    for key in ('angular_accel_radps2','angular_decel_radps2','command_timeout_sec','control_hz'):
        if isinstance(p[key],bool) or not math.isfinite(p[key]) or p[key]<=0:raise ValueError('invalid motion limit: '+key)
    for key in ('local_costmap_update_hz', 'local_costmap_publish_hz'):
        if isinstance(p[key],bool) or not math.isfinite(p[key]) or p[key]<=0:
            raise ValueError('invalid costmap frequency: '+key)
    if not math.isfinite(p['lateral_mps']) or p['lateral_mps']<0:raise ValueError('invalid lateral limit')
    if p['robot'] == 'bunker' and p['lateral_mps'] != 0:
        raise ValueError('Bunker must remain nonholonomic')
    if p['robot']=='fwmini' and (p['control_hz']!=50 or p.get('can_bitrate')!=500000):raise ValueError('FW-mini requires 50 Hz CtrlCmd and CAN 500 kbit/s')
    p['topics'] = dict(nav_input='/cmd_vel_nav', smoothed='/cmd_vel', manual='/agt/hmi/cmd_vel',
                      guarded='/mux/cmd_vel' if p['robot']=='bunker' else '/fwmini/cmd_vel_guarded',
                      driver='/mux/cmd_vel' if p['robot']=='bunker' else '/ctrl_cmd',
                      feedback='/wheel/odom' if p['robot']=='bunker' else '/chassis_info_fb')
    if p.get('driver_tf_enabled',False) is not False:raise ValueError('wheel driver TF must be disabled; selected LIO owns local TF')
    if p.get('body_frame','base_link')!='base_link' or p.get('odom_frame','odom')!='odom':raise ValueError('field frame contract requires odom/base_link')
    if int(p['control_hz'])!=p['control_hz']:raise ValueError('integer control frequency required')
    if p.get('wheel_odom_topic','/wheel/odom')!='/wheel/odom':raise ValueError('wheel feedback contract requires /wheel/odom')
    for key,topic in (('driver_command_topic',p['topics']['driver']),('guarded_command_topic',p['topics']['guarded'])):
        if key in p and p[key]!=topic:raise ValueError('profile control topic contract mismatch: '+key)
    overrides=p.get('topic_overrides', {})
    if any(k not in p['topics'] or v!=p['topics'][k] for k,v in overrides.items()):
        raise ValueError('control topic contract is fixed; unsupported override')
    p['profile_path'] = str(path)
    p['profile_sha256'] = hashlib.sha256(path.read_bytes()).hexdigest()
    return p


def guard(p, navigation=False):
    return dict(require_chassis_feedback=p['robot']=='bunker', require_deadman=not navigation, allow_navigation=navigation, kinematic_profile=p['robot'], default_control_mode='navigation' if navigation else 'manual',
        require_localization_status=navigation, input_topic=p['topics']['smoothed'],
        manual_input_topic=p['topics']['manual'], output_topic=p['topics']['guarded'],
        max_linear_x=p['forward_mps'], max_reverse_x=p['reverse_mps'], max_linear_y=p['lateral_mps'],
        max_angular_z=p['angular_radps'], max_linear_accel=p['linear_accel_mps2'],
        max_linear_decel=p['linear_decel_mps2'], max_angular_accel=p['angular_accel_radps2'],
        command_timeout_sec=p['command_timeout_sec'], publish_rate_hz=float(p['control_hz']))


def navigation(p, config_dir, output):
    tree = {}
    def merge(a,b):
        for k,v in b.items():
            if isinstance(v,dict) and isinstance(a.get(k),dict): merge(a[k],v)
            else: a[k]=copy.deepcopy(v)
    for name in ('robot','navigation','controller','costmap','perception','safety'):
        merge(tree,yaml.safe_load((Path(config_dir)/(name+'.yaml')).read_text()) or {})
    params=lambda name:tree[name]['ros__parameters']
    params('agt_motion_limits').update(forward_mps=p['forward_mps'], reverse_mps=p['reverse_mps'],
        angular_radps=p['angular_radps'], controller_cruise_mps=p['forward_mps'],
        controller_approach_mps=min(.03,p['forward_mps']), controller_regulated_min_mps=min(.03,p['forward_mps']),
        rotate_to_heading_radps=p['angular_radps'], controller_bootstrap_angular_accel=p['angular_accel_radps2'],
        linear_accel_mps2=p['linear_accel_mps2'],linear_decel_mps2=p['linear_decel_mps2'],
        angular_accel_radps2=p['angular_accel_radps2'],angular_decel_radps2=p['angular_decel_radps2'])
    # The field map can contain unknown cells after local-ground projection.
    # Never plan through unobserved space just to get a shorter Smac path.
    params('planner_server')['GridBased']['allow_unknown'] = False
    # Selected explicitly by the photo BT. Ordinary/accuracy navigation keeps
    # the original checker as the first (default) entry.
    controller = params('controller_server')
    if 'photo_goal_checker' not in controller['goal_checker_plugins']:
        controller['goal_checker_plugins'].append('photo_goal_checker')
    controller['photo_goal_checker'] = {
        'plugin': 'nav2_controller::SimpleGoalChecker',
        'stateful': True, 'xy_goal_tolerance': 0.35,
        'yaw_goal_tolerance': math.pi,
    }
    # Humble refuses an empty checker ID when multiple checkers are loaded.
    # Preserve ordinary navigation's checker explicitly in both default BTs.
    from ament_index_python.packages import get_package_share_directory
    import xml.etree.ElementTree as ET
    bt_dir = Path(get_package_share_directory('nav2_bt_navigator'))/'behavior_trees'
    for key, filename in (
        ('default_nav_to_pose_bt_xml', 'navigate_w_recovery_and_replanning_only_if_path_becomes_invalid.xml'),
        ('default_nav_through_poses_bt_xml', 'navigate_through_poses_w_replanning_and_recovery.xml'),
    ):
        bt = ET.parse(bt_dir/filename)
        for follow in bt.iter('FollowPath'):
            follow.set('goal_checker_id', controller['goal_checker_plugins'][0])
        destination = Path(output).with_name(Path(output).stem+'_'+key+'.xml')
        bt.write(destination, encoding='unicode')
        params('bt_navigator')[key] = str(destination.resolve())
    params('controller_server')['FollowPath'].update(desired_linear_vel=p['forward_mps'],
        min_approach_linear_velocity=min(.03,p['forward_mps']),regulated_linear_scaling_min_speed=min(.03,p['forward_mps']),
        rotate_to_heading_angular_vel=p['angular_radps'],max_angular_accel=p['angular_accel_radps2'])
    params('behavior_server').update(max_rotational_vel=p['angular_radps'], min_rotational_vel=min(.03,p['angular_radps']),
        rotational_acc_lim=p['angular_accel_radps2'])
    params('velocity_smoother').update(max_velocity=[p['forward_mps'],0.,p['angular_radps']],
        min_velocity=[-p['reverse_mps'],0.,-p['angular_radps']],
        max_accel=[p['linear_accel_mps2'],0.,p['angular_accel_radps2']],
        max_decel=[-p['linear_decel_mps2'],0.,-p['angular_decel_radps2']],smoothing_frequency=float(p['control_hz']))
    params('agt_cmd_vel_guard').update(guard(p,True))
    for key in ('local_costmap','global_costmap'):
        tree[key][key]['ros__parameters'].update(footprint=json.dumps(p['footprint_m']),robot_base_frame=p['body_frame'])
    tree['local_costmap']['local_costmap']['ros__parameters'].update(
        update_frequency=p['local_costmap_update_hz'], publish_frequency=p['local_costmap_publish_hz'])
    # These limits are in odom, whereas the upstream ground/self filtering
    # already operates relative to the vehicle/terrain. The observed base z
    # is about -0.85 m: the old [-0.3, 2.1] voxel band clipped ground clearing
    # rays and the old +0.1 m marking threshold discarded low obstacles.
    if p['robot'] == 'bunker':
        voxel = tree['local_costmap']['local_costmap']['ros__parameters']['voxel_layer']
        voxel.update(origin_z=-3.0, z_resolution=0.4, max_obstacle_height=3.4)
        for source in ('lidar3d_mark', 'lidar3d_clear'):
            voxel[source].update(min_obstacle_height=-3.0, max_obstacle_height=3.4)
    if p['navigation_self_mask_boxes']:
        params('agt_pointcloud_preprocessor')['self_filter']['extra_boxes'] = p['navigation_self_mask_boxes']
    Path(output).write_text(yaml.safe_dump(tree,sort_keys=False))
    return str(output)


def selected_path(workspace,robot):
    directory=Path(workspace)/'config/robots';pointer=directory/('active_'+robot+'.json')
    if not pointer.is_file():return directory/(robot+'.yaml')
    item=json.loads(pointer.read_text());path=Path(item['path']).resolve()
    if not path.is_relative_to(directory.resolve()) or hashlib.sha256(path.read_bytes()).hexdigest()!=item['sha256']:raise ValueError('active robot profile version changed')
    return path
