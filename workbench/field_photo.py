"""RViz waypoint collection for the existing inspection runtime and C1 camera."""
import argparse
import os
from pathlib import Path
import shlex
import math

import yaml

REPO = Path(__file__).resolve().parents[1]
ROOT = Path(os.environ.get('AGT_FIELD_TEST_WS', str(REPO.parent.parent)))


def validate_photo_parameters(runtime, controller):
    expected = str(REPO/'config/photo_navigate.xml')
    if runtime.get('nav_behavior_tree') != expected or runtime.get('recenter_after_capture') is not True:
        raise ValueError('拍照执行器未加载新配置；在原 ./field_photo runtime 终端 Ctrl+C 后重新运行 ./field_photo runtime。')
    if ('photo_goal_checker' not in (controller.get('goal_checker_plugins') or []) or
            controller.get('photo_goal_checker.xy_goal_tolerance') != .35 or
            not math.isclose(controller.get('photo_goal_checker.yaw_goal_tolerance') or 0., math.pi)):
        raise ValueError('现场导航未加载 photo_goal_checker；需重启 field_trial 并重新完成定位、nav。')


def check_photo_runtime():
    """Read actual running parameters before submitting any motion-capable task."""
    import rclpy
    from .ros_parameters import read_parameters
    from .mode_state import require_mode_ready

    require_mode_ready(ROOT)

    rclpy.init(args=[])
    node = rclpy.create_node('field_photo_check_runtime')
    def read(name, keys):
        return read_parameters(node,name,keys)
    try:
        runtime = read('mission_runtime', ['nav_behavior_tree', 'recenter_after_capture'])
        controller = read('controller_server', ['goal_checker_plugins',
                          'photo_goal_checker.xy_goal_tolerance', 'photo_goal_checker.yaw_goal_tolerance'])
        validate_photo_parameters(runtime, controller)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def command_plan(args):
    if args.command == 'camera':
        # Humble usb_cam mishandles relative V4L by-id symlink targets. Resolve
        # the stable selection before passing the concrete node to the driver.
        device = str(Path(args.camera_device).expanduser().resolve())
        return ['ros2', 'launch', 'autolabor_c1_bringup', 'autolabor_c1.launch.py',
                'device_path:='+device, 'port_name:='+args.gimbal_port,
                'image_width:=1920', 'image_height:=1080', 'fps:=30.0',
                'pixel_format:=mjpeg2rgb', 'gui:=false',
                'capture_output_root:='+str(ROOT/'data/captures/camera')]
    if args.command == 'runtime':
        from ament_index_python.packages import get_package_share_directory
        params = Path(get_package_share_directory('agt_navigation_runtime'))/'config/runtime.yaml'
        return ['ros2', 'run', 'agt_navigation_runtime', 'mission_runtime', '--ros-args',
                '--params-file', str(params), '-p', 'record_root:='+str(ROOT/'data/captures/missions'),
                '-p', 'nav_behavior_tree:='+str(REPO/'config/photo_navigate.xml'),
                '-p', 'recenter_after_capture:=true', '-p', 'stationary_timeout_sec:=15.0',
                '-p', 'wait_for_bunker_control:=true', '-p', 'wheel_stop_gate:=true']
    if args.command == 'queue':
        map_id = args.map_id or yaml.safe_load((ROOT/'config/field_trial.yaml').read_text())['map_id']
        return ['ros2', 'run', 'agt_rviz_patrol', 'rviz_patrol', '--ros-args',
                '-p', 'goal_topic:=/agt/rviz_patrol/goal', '-p', 'map_id:='+map_id,
                '-p', 'preset_file:='+str(REPO/'config/photo_three_views.yaml'),
                '-p', 'mission_dir:='+str(ROOT/'missions/rviz_photo')]
    if args.command == 'rviz':
        return ['ros2', 'run', 'rviz2', 'rviz2', '-d', str(REPO/'config/field_photo.rviz')]
    return ['ros2', 'service', 'call', '/agt/rviz_patrol/'+args.command, 'std_srvs/srv/Trigger', '{}']


def main(argv=None):
    parser = argparse.ArgumentParser(description='RViz 多点排队 → 到点停稳 → 云台三视图拍照 → 返回起点')
    parser.add_argument('command', choices=('camera', 'runtime', 'queue', 'rviz', 'start', 'clear', 'cancel'))
    parser.add_argument('--camera-device', default='/dev/v4l/by-id/usb-Wasintek_Wasintek_camera_00.00.01-video-index0')
    parser.add_argument('--gimbal-port', default='/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0')
    parser.add_argument('--map-id', help='queue 所用地图；默认与 config/field_trial.yaml 一致')
    parser.add_argument('--dry-run', action='store_true')
    args = parser.parse_args(argv)
    command = command_plan(args)
    if args.dry_run:
        print(shlex.join(command))
        return 0
    if args.command == 'start':
        try:
            check_photo_runtime()
        except ValueError as exc:
            print(str(exc), flush=True)
            return 2
        print('提交队列任务：依次导航、停稳、每点拍三张，然后返回启动任务时的位置。', flush=True)
    os.execvp(command[0], command)


if __name__ == '__main__':
    raise SystemExit(main())
