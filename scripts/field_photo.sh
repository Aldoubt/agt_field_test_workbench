#!/usr/bin/env bash
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export PYTHONPATH="$REPO:${PYTHONPATH:-}"
exec bash "$REPO/scripts/ros_exec.sh" python3 -m workbench.field_photo "$@"
