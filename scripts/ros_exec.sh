#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
set +u
source /opt/ros/humble/setup.bash
source "$ROOT/install/setup.bash"
set -u
export LD_LIBRARY_PATH="$ROOT/work/ugv_sdk_install/lib:$ROOT/work/bbs3d_install/lib:$ROOT/work/livox_sdk2_install/lib:${LD_LIBRARY_PATH:-}"
exec "$@"
