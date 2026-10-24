#!/usr/bin/env bash
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROOT="${AGT_FIELD_TEST_WS:-$(cd "$REPO/../.." && pwd)}"
mkdir -p "$ROOT/work"
if ! command -v flock >/dev/null; then echo 'flock is required' >&2; exit 2; fi
# Lock resources, not the whole session: a sensor launch must coexist with a
# chassis launch. Defaults match field_test.launch.py. FDs survive exec.
sensor=true; chassis=false; description=true; localization=false; navigation=false; guard=true
for argument in "$@"; do
  case "$argument" in
    enable_mid360:=*) sensor="${argument#*:=}";;
    enable_chassis:=*) chassis="${argument#*:=}";;
    enable_description:=*) description="${argument#*:=}";;
    enable_localization:=*) localization="${argument#*:=}";;
    enable_navigation:=*) navigation="${argument#*:=}";;
    enable_guard:=*) guard="${argument#*:=}";;
  esac
done
acquire() { flock -n "$1" || { echo "resource already owned: $2" >&2; exit 3; }; }
if [[ "$sensor" == true ]]; then exec 20>"$ROOT/work/sensor.lock"; acquire 20 sensor; fi
if [[ "$chassis" == true ]]; then exec 21>"$ROOT/work/chassis.lock"; acquire 21 chassis; fi
if [[ "$description" == true ]]; then exec 22>"$ROOT/work/description.lock"; acquire 22 description; fi
if [[ "$localization" == true ]]; then exec 23>"$ROOT/work/localization.lock"; acquire 23 localization; fi
if [[ "$navigation" == true || ( "$chassis" == true && "$guard" == true ) ]]; then
  exec 24>"$ROOT/work/control.lock"; acquire 24 control
fi
exec bash "$REPO/scripts/ros_exec.sh" ros2 launch agt_field_test_bringup field_test.launch.py "$@"
