#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo 'BLOCKED: Ubuntu 22.04 / ROS 2 Humble is required for this build.' >&2
  exit 2
fi
python3 "$ROOT/src/agt_field_test_workbench/scripts/fetch_locked.py"
python3 "$ROOT/src/agt_field_test_workbench/scripts/apply_control_integration.py" \
  --manifest "$ROOT/src/agt_field_test_workbench/patches/frontend_manifest.json"
python3 "$ROOT/src/agt_field_test_workbench/scripts/apply_control_integration.py" \
  --manifest "$ROOT/src/agt_field_test_workbench/patches/map_cleanup_manifest.json"
python3 "$ROOT/src/agt_field_test_workbench/scripts/apply_control_integration.py"
python3 "$ROOT/src/agt_field_test_workbench/scripts/audit.py" > "$ROOT/reports/dependency_audit.json"
python3 "$ROOT/src/agt_field_test_workbench/scripts/apply_mapping_calibration.py"
LIVOX="$ROOT/src/external/livox_ros_driver2"
if [[ ! -e "$LIVOX/package.xml" ]]; then cp "$LIVOX/package_ROS2.xml" "$LIVOX/package.xml"; fi
if [[ ! -e "$LIVOX/launch" ]]; then cp -a "$LIVOX/launch_ROS2" "$LIVOX/launch"; fi
# This upstream CMakeLists installs both launch/ and config/, while the latter
# has no tracked files in the pinned revision.
mkdir -p "$ROOT/src/agt_navigation_v3/navigation/nav2/agt_nav2_bringup/config"
if ! command -v rosdep >/dev/null || ! command -v colcon >/dev/null; then
  echo 'BLOCKED: install python3-rosdep, python3-colcon-common-extensions, ros-humble-gtsam and ros-humble-sophus.' >&2
  exit 2
fi
# Only selected packages enter the build graph. The selected FAST-LIO2 and
# Livox revisions are from the mapping baseline; V3's alternative revisions
# are deliberately absent from .repos.
set +u
source /opt/ros/humble/setup.bash
set -u
cd "$ROOT"
# Upstream manifests contain raw Debian package names and optional C1/camera
# runtime keys. The field entry disables those capabilities; native libraries
# below remain required and are checked explicitly, rather than silently skipped.
for required in libyaml-cpp-dev libeigen3-dev python3-setuptools; do
  dpkg-query -W -f='${Status}' "$required" | grep -q 'install ok installed' || { echo "Missing $required" >&2; exit 2; }
done
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y \
  --skip-keys 'GTSAM Sophus livox_ros_driver2 catkin ament_python libyaml-cpp-dev libeigen3-dev camera_gimbal_interfaces agt_navigation_route_runtime autolabor_c1_bringup'
SDK_PREFIX="$ROOT/work/livox_sdk2_install"
cmake -S "$ROOT/src/external/Livox-SDK2" -B "$ROOT/work/livox_sdk2_build" \
  -DCMAKE_INSTALL_PREFIX="$SDK_PREFIX"
cmake --build "$ROOT/work/livox_sdk2_build" --parallel 2
cmake --install "$ROOT/work/livox_sdk2_build"
BBS_PREFIX="$ROOT/work/bbs3d_install"
cmake -S "$ROOT/src/external/3d_bbs" -B "$ROOT/work/bbs3d_build" \
  -DBUILD_CUDA=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$BBS_PREFIX"
cmake --build "$ROOT/work/bbs3d_build" --parallel 2
cmake --install "$ROOT/work/bbs3d_build"
UGV_PREFIX="$ROOT/work/ugv_sdk_install"
cmake -S "$ROOT/src/external/ugv_sdk" -B "$ROOT/work/ugv_sdk_build" \
  -DBUILD_TESTING=OFF -DCMAKE_INSTALL_PREFIX="$UGV_PREFIX"
cmake --build "$ROOT/work/ugv_sdk_build" --parallel 2
cmake --install "$ROOT/work/ugv_sdk_build"
export CMAKE_PREFIX_PATH="$UGV_PREFIX:$BBS_PREFIX:${CMAKE_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="$UGV_PREFIX/lib:$BBS_PREFIX/lib:$SDK_PREFIX/lib:${LD_LIBRARY_PATH:-}"
colcon build --base-paths src --packages-select livox_ros_driver2 \
  --cmake-args -DROS_EDITION=ROS2 -DDISTRO_ROS=humble \
  "-DLIVOX_LIDAR_SDK_LIBRARY=$SDK_PREFIX/lib/liblivox_lidar_sdk_shared.so" \
  "-DLIVOX_LIDAR_SDK_INCLUDE_DIR=$SDK_PREFIX/include"
# The navigation native relocalizer calls find_package(small_gicp), but its
# upstream package.xml does not declare that build edge.
colcon build --base-paths src --packages-select small_gicp
colcon build --base-paths src --packages-select sophus \
  --cmake-args -DBUILD_SOPHUS_TESTS=OFF -DBUILD_SOPHUS_EXAMPLES=OFF
set +u
source "$ROOT/install/setup.bash"
set -u
colcon build --base-paths src --packages-select agt_global_relocalization_native \
  --cmake-args "-DBBS3D_INCLUDE_DIR=$BBS_PREFIX/include" \
  "-DBBS3D_CPU_LIBRARY=$BBS_PREFIX/lib/libcpu_bbs3d.so"
colcon build --base-paths src --packages-up-to agt_asensing_driver agt_mapping_bringup agt_pcd2grid_exporter agt_field_test_bringup agt_dynamic_map_cleaner \
  --packages-ignore ugv_sdk bunker_base
colcon test --base-paths src --packages-select agt_asensing_driver fwmini_chassis_adapter agt_field_test_bringup agt_pcd2grid_exporter agt_mapping_bringup \
  --packages-ignore ugv_sdk bunker_base
colcon test-result --verbose > "$ROOT/reports/colcon_test_result.txt"
