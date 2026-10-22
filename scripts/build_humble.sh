#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo 'BLOCKED: Ubuntu 22.04 / ROS 2 Humble is required for this build.' >&2
  exit 2
fi
python3 "$ROOT/src/agt_field_test_workbench/scripts/fetch_locked.py"
python3 "$ROOT/src/agt_field_test_workbench/scripts/audit.py" > "$ROOT/reports/dependency_audit.json"
python3 "$ROOT/src/agt_field_test_workbench/scripts/apply_mapping_calibration.py"
LIVOX="$ROOT/src/external/livox_ros_driver2"
if [[ ! -e "$LIVOX/package.xml" ]]; then cp "$LIVOX/package_ROS2.xml" "$LIVOX/package.xml"; fi
if [[ ! -e "$LIVOX/launch" ]]; then cp -a "$LIVOX/launch_ROS2" "$LIVOX/launch"; fi
if ! command -v rosdep >/dev/null || ! command -v colcon >/dev/null; then
  echo 'BLOCKED: install python3-rosdep, python3-colcon-common-extensions, ros-humble-gtsam and ros-humble-sophus.' >&2
  exit 2
fi
# Only selected packages enter the build graph. The selected FAST-LIO2 and
# Livox revisions are from the mapping baseline; V3's alternative revisions
# are deliberately absent from .repos.
source /opt/ros/humble/setup.bash
cd "$ROOT"
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y \
  --skip-keys 'GTSAM Sophus livox_ros_driver2'
colcon build --base-paths src --packages-select livox_ros_driver2 \
  --cmake-args -DROS_EDITION=ROS2 -DDISTRO_ROS=humble
colcon build --base-paths src --packages-up-to agt_mapping_bringup agt_pcd2grid_exporter agt_field_test_bringup
colcon test --base-paths src --packages-select agt_field_test_bringup
colcon test-result --verbose > "$ROOT/reports/colcon_test_result.txt"
