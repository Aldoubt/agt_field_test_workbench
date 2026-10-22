#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
source /opt/ros/humble/setup.bash
source "$ROOT/install/setup.bash"
exec "$@"
