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
    parser.add_argument('--profile')
    parser.add_argument('--bundle')
    parser.add_argument('--backend',default='fastlio2')
    parser.add_argument('--internal-config')
    parser.add_argument('--preflight', action='store_true')
    parser.add_argument('--report', default='')
    parser.add_argument('--timeout', type=float, default=10.0)
    args, ros_args = parser.parse_known_args()
    if args.bundle:
        from agt_field_test_bringup import calibration
        import xml.etree.ElementTree as ET
        profile=calibration.load(args.bundle);expected={}
        root=ET.fromstring(calibration.description(profile,args.backend))
        for joint in root.findall('joint'):
            child=joint.find('child').attrib['link']
            if child not in ('base_link','lidar_link','livox_frame','imu_link'):continue
            origin=joint.find('origin').attrib
            expected[child]=(joint.find('parent').attrib['link'],list(map(float,origin['xyz'].split())),list(map(float,origin['rpy'].split())))
        calibration_digest=calibration.digest(profile)
    else:
        profile = load_profile(args.profile)
        expected = edges(profile, args.internal_config)
        calibration_digest=digest(profile)
    parameters = {
        'children': list(expected),
        'parents': [value[0] for value in expected.values()],
        'expected_xyz': [float(v) for value in expected.values() for v in value[1]],
        'expected_quaternions': [float(v) for value in expected.values() for v in quaternion(value[2])],
        'calibration_sha256': calibration_digest, 'report_path': args.report,
        'preflight': args.preflight, 'timeout_sec': args.timeout,
    }
    binary = Path(get_package_prefix('agt_field_test_bringup'))/'lib/agt_field_test_bringup/fwmini_tf_guard'
    command = [str(binary), *ros_args, '--ros-args']
    for key, value in parameters.items():
        command += ['-p', key+':='+json.dumps(value)]
    os.execv(binary, command)


if __name__ == '__main__':
    main()
