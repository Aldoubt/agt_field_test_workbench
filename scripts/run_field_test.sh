#!/usr/bin/env bash
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROOT="$(cd "$REPO/../.." && pwd)"
mkdir -p "$ROOT/work"
if ! command -v flock >/dev/null; then echo 'flock is required' >&2; exit 2; fi
# The lock covers both robot profiles and prevents a second script-run chassis
# instance. Direct ros2 launches still require operator coordination.
exec flock -n "$ROOT/work/field_test.lock" bash "$REPO/scripts/ros_exec.sh" \
  ros2 launch agt_field_test_bringup field_test.launch.py "$@"
