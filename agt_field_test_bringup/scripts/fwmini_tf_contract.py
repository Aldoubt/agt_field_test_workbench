#!/usr/bin/env python3
"""Validate evidence in Python, then observe TF GIDs in Humble rclcpp."""
import argparse
import json
import os
from pathlib import Path
from ament_index_python.packages import get_package_prefix
from agt_field_test_bringup.fwmini import edges, load_profile, quaternion, digest


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--profile', required=True)
    parser.add_argument('--internal-config')
    parser.add_argument('--preflight', action='store_true')
    parser.add_argument('--report', default='')
    parser.add_argument('--timeout', type=float, default=10.0)
    args, ros_args = parser.parse_known_args()
    profile = load_profile(args.profile)
    expected = edges(profile, args.internal_config)
    parameters = {
        'children': list(expected),
        'parents': [value[0] for value in expected.values()],
        'expected_xyz': [float(v) for value in expected.values() for v in value[1]],
        'expected_quaternions': [float(v) for value in expected.values() for v in quaternion(value[2])],
        'calibration_sha256': digest(profile), 'report_path': args.report,
        'preflight': args.preflight, 'timeout_sec': args.timeout,
    }
    binary = Path(get_package_prefix('agt_field_test_bringup'))/'lib/agt_field_test_bringup/fwmini_tf_guard'
    command = [str(binary), *ros_args, '--ros-args']
    for key, value in parameters.items():
        command += ['-p', key+':='+json.dumps(value)]
    os.execv(binary, command)


if __name__ == '__main__':
    main()
