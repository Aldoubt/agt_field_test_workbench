#!/usr/bin/env python3
"""Reuse the mapping runner, then its existing PCD-to-grid exporter."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys

REPO = Path(__file__).resolve().parents[1]
ROOT = REPO.parent.parent
sys.path.insert(0, str(REPO))
from workbench.maps import Maps  # noqa: E402
from workbench.runtime import write_json  # noqa: E402


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--robot', required=True, choices=['bunker', 'fwmini'])
    parser.add_argument('--output', required=True)
    parser.add_argument('--bag')
    parser.add_argument('--live', action='store_true')
    parser.add_argument('--livox-config')
    parser.add_argument('--calibration')
    args = parser.parse_args()
    output = Path(args.output).resolve()
    mapping = ROOT / 'src/agt-lio-pgo-mapping/scripts/run_mid360_mapping.sh'
    command = ['bash', str(mapping)]
    if args.live:
        command += ['--live', '--robot', 'yhs_v1', '--livox-config', args.livox_config, str(output)]
    else:
        command += [args.bag, str(output)]
    command += ['--no-rviz', '--domain-id', '89', '--setup', str(ROOT / 'install/setup.bash')]
    print('Running existing mapping entry point:', command, flush=True)
    code = subprocess.call(command)
    if code:
        print(f'Mapping exited {code}; no conversion or acceptance claimed', flush=True)
        return code
    package = output / 'map_package'
    if not package.is_dir():
        print(f'Mapping package missing: {package}', file=sys.stderr)
        return 3
    grid = output / 'map_grid'
    projection = ROOT / 'src/agt-lio-pgo-mapping/exporters/agt_pcd2grid_exporter/config/projection.yaml'
    converter = ['bash', str(REPO / 'scripts/ros_exec.sh'), 'ros2', 'run',
                 'agt_pcd2grid_exporter', 'pcd2grid_exporter', '--package', str(package),
                 '--config', str(projection), '--output', str(grid)]
    print('Converting verified PCD package:', converter, flush=True)
    code = subprocess.call(converter)
    if code:
        print(f'PCD-to-grid conversion exited {code}; 3D package preserved', file=sys.stderr)
        return code
    try:
        version = Maps(ROOT).import_yaml(grid / 'map.yaml',
                                         calibration=args.calibration, source_bag=args.bag)
    except (ValueError, KeyError, OSError) as exc:
        print(f'2D map import failed: {exc}; grid preserved', file=sys.stderr)
        return 4
    write_json(output / 'field_test_result.json', {'robot': args.robot, 'calibration': args.calibration,
               'source_bag': args.bag or str(output / 'raw_bag'), 'map_version': version['id'],
               'status': 'DRAFT'})
    print('Map Studio version:', version['id'], flush=True)
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        sys.exit(130)
