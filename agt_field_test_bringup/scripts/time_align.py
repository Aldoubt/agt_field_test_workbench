#!/usr/bin/env python3
"""Compatibility entrypoint for native, timing-preserving LiDAR/IMU alignment."""
import os
from pathlib import Path
import sys
from ament_index_python.packages import get_package_prefix


def main():
    binary = Path(get_package_prefix('agt_field_test_bringup')) / 'lib/agt_field_test_bringup/time_align'
    os.execv(binary, [str(binary), *sys.argv[1:]])


if __name__ == '__main__':
    main()
