#!/usr/bin/env python3
"""Reuse the mapping runner, then its existing PCD-to-grid exporter."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import signal
import time
import shutil

REPO = Path(__file__).resolve().parents[1]
ROOT = REPO.parent.parent
sys.path.insert(0, str(REPO))
from workbench.maps import Maps  # noqa: E402
from workbench.runtime import check_bag, write_json  # noqa: E402
from workbench.fwmini import resolve_profile, load_profile, require_mode, digest, sha256  # noqa: E402


def stop_child(process):
    if process is not None and process.poll() is None:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            process.terminate()
            process.wait(timeout=5)


ACTIVE_MAPPING = None

def finish_mapping(*_):
    if ACTIVE_MAPPING and ACTIVE_MAPPING.poll() is None: ACTIVE_MAPPING.send_signal(signal.SIGUSR1)

signal.signal(signal.SIGUSR1, finish_mapping)

def run_mapping(command, args):
    global ACTIVE_MAPPING
    """Own the selected mount in the mapping domain for this job's lifetime."""
    if os.environ.get('FIELD_REUSE_RUNTIME')=='1':
        ACTIVE_MAPPING=subprocess.Popen(command,env=os.environ.copy())
        try:return ACTIVE_MAPPING.wait(),{'reuse_runtime':True,'calibration':os.environ.get('FIELD_CALIBRATION_BUNDLE')}
        finally:stop_child(ACTIVE_MAPPING)
    if args.robot != 'fwmini':
        bundle=os.environ.get('FIELD_CALIBRATION_BUNDLE')
        tf_process=None
        env=dict(os.environ,ROS_DOMAIN_ID=os.environ.get('FIELD_MAPPING_DOMAIN','89'))
        try:
            if bundle:
                command_tf=['bash',str(REPO/'scripts/ros_exec.sh'),'ros2','launch','agt_field_test_bringup','field_test.launch.py',
                    'robot:=bunker','enable_chassis:=false','enable_mid360:=false','calibration_bundle:='+bundle]
                tf_process=subprocess.Popen(command_tf,env=env,start_new_session=True)
                time.sleep(.5)
                subprocess.run(['bash',str(REPO/'scripts/ros_exec.sh'),'ros2','run','agt_field_test_bringup','fwmini_tf_contract.py',
                    '--bundle',bundle,'--preflight'],env=env,check=True,timeout=15)
            ACTIVE_MAPPING=subprocess.Popen(command,env=env)
            while ACTIVE_MAPPING.poll() is None:
                if tf_process and tf_process.poll() is not None:raise RuntimeError('Bunker TF guard stopped')
                time.sleep(.1)
            return ACTIVE_MAPPING.returncode,{'bundle':bundle}
        finally:
            stop_child(ACTIVE_MAPPING);stop_child(tf_process)
    mount = resolve_profile(ROOT, args.mount_profile)
    profile = load_profile(mount)
    require_mode(profile, 'sensor_mapping' if args.live else 'offline_mapping')
    directory = ROOT/'runs'/(Path(args.output).name+'_fwmini_mount')
    directory.mkdir(parents=True, exist_ok=False)
    snapshot = directory/'mount.yaml'
    shutil.copyfile(mount, snapshot)
    ready = directory/'tf_ready.json'
    launch = ['bash', str(REPO/'scripts/ros_exec.sh'), 'ros2', 'launch', 'agt_field_test_bringup',
              'field_test.launch.py', 'robot:=fwmini', 'enable_chassis:=false', 'enable_mid360:=false',
              f'fwmini_mount_file:={snapshot}', f'fwmini_tf_report:={ready}',
              'fwmini_mode:='+('sensor_mapping' if args.live else 'offline_mapping'),
              'use_sim_time:='+('false' if args.live else 'true')]
    internal_config = Path(os.environ.get('AGT_FIELD_TEST_FASTLIO_CONFIG', str(
        ROOT/'src/agt-lio-pgo-mapping/bringup/agt_mapping_bringup/config/fastlio2_mid360.yaml')))
    internal_snapshot = directory/'fastlio.yaml'
    shutil.copyfile(internal_config, internal_snapshot)
    launch += [f'fwmini_internal_config:={internal_snapshot}']
    env = dict(os.environ, ROS_DOMAIN_ID=os.environ.get('FIELD_MAPPING_DOMAIN','89'), AGT_FIELD_TEST_FASTLIO_CONFIG=str(internal_snapshot))
    tf_process = mapping_process = None
    try:
        with (directory/'tf.log').open('w') as log:
            tf_process = subprocess.Popen(launch, env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            deadline = time.monotonic()+15
            while not ready.is_file():
                if tf_process.poll() is not None or time.monotonic() > deadline:
                    raise RuntimeError(f'FW-mini mount TF preflight failed; see {directory}/tf.log')
                time.sleep(0.1)
            mapping_process = subprocess.Popen(command, env=env)
            ACTIVE_MAPPING = mapping_process
            while mapping_process.poll() is None:
                if tf_process.poll() is not None:
                    raise RuntimeError('FW-mini mount TF guard stopped during mapping')
                time.sleep(0.1)
            metadata = {'path': str(snapshot), 'calibration_sha256': digest(profile),
                        'status': profile['status'], 'tf_report': str(ready),
                        'internal_config': str(internal_snapshot), 'internal_sha256': sha256(internal_snapshot)}
            return mapping_process.returncode, metadata
    finally:
        stop_child(mapping_process)
        stop_child(tf_process)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--robot', required=True, choices=['bunker', 'fwmini'])
    parser.add_argument('--output', required=True)
    parser.add_argument('--bag')
    parser.add_argument('--live', action='store_true')
    parser.add_argument('--livox-config')
    parser.add_argument('--calibration')
    parser.add_argument('--mount-profile')
    parser.add_argument('--backend',choices=['fastlio','pgo_experiment'],default='fastlio')
    parser.add_argument('--map-name',default='')
    args = parser.parse_args()
    output = Path(args.output).resolve()
    mapping = ROOT / 'src/agt-lio-pgo-mapping/scripts/run_mid360_mapping.sh'
    command = ['bash', str(mapping)]
    if args.live:
        command += ['--live', '--robot', 'yhs_v1', '--livox-config', args.livox_config, str(output)]
    else:
        command += [args.bag, str(output)]
    command += ['--no-rviz', '--domain-id', '89', '--setup', str(ROOT / 'install/setup.bash')]
    if args.backend == 'fastlio':
        config = os.environ.get('AGT_FIELD_TEST_FASTLIO_CONFIG', str(ROOT/'src/agt-lio-pgo-mapping/bringup/agt_mapping_bringup/config/fastlio2_mid360.yaml'))
        command = ['bash',str(REPO/'scripts/ros_exec.sh'),'python3',str(REPO/'scripts/no_pgo_mapping.py'),
                   '--output',str(output),'--config',config,'--imu-offset',os.environ.get('FIELD_IMU_OFFSET','0'),
                   '--lidar-offset',os.environ.get('FIELD_LIDAR_OFFSET','0')]
        if args.robot=='bunker' or os.environ.get('FIELD_REUSE_RUNTIME')=='1':command += ['--base-adapter']
        if args.live:
            command += ['--live','--livox-config',args.livox_config or '']
            if os.environ.get('FIELD_REUSE_RUNTIME')=='1':command += ['--reuse-sensors']
        else:command += ['--bag',args.bag]
    print('Running selected mapping backend:', command, flush=True)
    code, mount_metadata = run_mapping(command, args)
    if code:
        print(f'Mapping exited {code}; no conversion or acceptance claimed', flush=True)
        return code
    raw_integrity = check_bag(output / 'raw_bag') if args.live else None
    if args.live and raw_integrity['state'] != 'PASS':
        print(f'Live raw bag incomplete: {raw_integrity}; output preserved', file=sys.stderr)
        return 5
    def save_stage(stage, **details):
        write_json(output/'save_status.json',dict(stage=stage,**details))
    save_stage('PCD_IMPORT')
    package = output / 'map_package'
    if not package.is_dir():
        print(f'Mapping package missing: {package}', file=sys.stderr)
        return 3
    from workbench.artifacts import Clouds
    clouds=Clouds(ROOT)
    default_lio_config=ROOT/'src/agt-lio-pgo-mapping/bringup/agt_mapping_bringup/config/fastlio2_mid360.yaml'
    internal_config=mount_metadata.get('internal_config') or os.environ.get('AGT_FIELD_TEST_FASTLIO_CONFIG') or str(default_lio_config)
    calibration_meta=dict(robot=args.robot,legacy_id=args.calibration,mount=mount_metadata,
                          internal_config=internal_config,source_artifact=str(output))
    if Path(internal_config).is_file():calibration_meta['internal_sha256']=sha256(Path(internal_config))
    if cleanup:calibration_meta['sc_keyframes']=str(output/'temporal_cleanup'/'keyframes')
    if os.environ.get('FIELD_CALIBRATION_BUNDLE'):
        bundle_path=Path(os.environ['FIELD_CALIBRATION_BUNDLE']);calibration_meta.update(bundle=str(bundle_path),bundle_sha256=sha256(bundle_path))
    cleanup = None
    selected_pcd = package/'map.pcd'
    if args.backend == 'fastlio' and args.robot == 'bunker' and os.environ.get('FIELD_CALIBRATION_BUNDLE'):
        save_stage('TEMPORAL_CLEANUP')
        from workbench.map_cleanup import clean_mapping
        cleanup = clean_mapping(output, ROOT, os.environ['FIELD_CALIBRATION_BUNDLE'])
        selected_pcd = Path(cleanup['cleaned_pcd'])
    # Deliberately candidate-only: first-stage temporal cleanup remains the
    # default map source. Optional independent clearance evidence must be
    # reviewed before a later explicit map-version promotion.
    start_clearance = {'state': 'NOT_REQUESTED'}
    if cleanup and os.environ.get('AGT_START_CLEARANCE_CANDIDATES','0') == '1':
        save_stage('START_CLEARANCE')
        candidate_dir = output/'temporal_cleanup'/'start_clearance_candidate'
        try:
            from agt_dynamic_map_cleaner.cli import run as clean_start
            candidate = clean_start(
                selected_pcd, output/'temporal_cleanup'/'keyframes', candidate_dir,
                REPO/'agt_dynamic_map_cleaner'/'config'/'start_clearance.yaml',
                output/'temporal_cleanup'/'manifest.json')
            start_clearance = dict(candidate, output=str(candidate_dir))
        except Exception as error:
            start_clearance = {'state':'FAILED','error':str(error)[:800],
                               'original_static_pcd':str(selected_pcd)}
            # The operator should still be able to inspect the independently
            # generated first-stage map; do not call this optional run READY.
            print('Start clearance candidate FAILED: '+str(error),flush=True)
    cloud=clouds.import_pcd(selected_pcd,calibration=calibration_meta,bag=args.bag or str(output/'raw_bag'),name=args.map_name,
                            cleanup=cleanup)
    save_stage('GRID_EXPORT', cloud_id=cloud['id'])
    version=clouds.grid(cloud['id'], {},build_assets=False)
    save_stage('RELOCALIZATION_ASSETS', cloud_id=cloud['id'], map_id=version['id'])
    assets=clouds.build_assets(cloud['id'],map_id=version['id'])
    result_stage=('COMPLETE' if assets.get('bbs')=='READY' and assets.get('polar_context')=='READY'
                  and assets.get('scan_context')=='READY' else 'PARTIAL_ASSETS')
    save_stage(result_stage, cloud_id=cloud['id'], map_id=version['id'],
               asset_state=assets['state'], asset_error=assets.get('error',''))
    write_json(output / 'field_test_result.json', {'robot': args.robot, 'calibration': args.calibration,
               'source_bag': args.bag or str(output / 'raw_bag'), 'map_version': version['id'],
               'raw_bag_integrity': raw_integrity, 'mount_profile': mount_metadata,
               'relocalization_assets': assets, 'sc_database':assets.get('sc_database'),
               'status': 'DRAFT',
               'temporal_cleanup': cleanup or {'state':'NOT_RUN', 'reason':'requires indexed FAST-LIO export and selected Bunker bundle'},
               'start_clearance_candidate': start_clearance,
               'save_state': result_stage})
    print('Map Studio version:', version['id'], flush=True)
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        sys.exit(130)
