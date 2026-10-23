#!/usr/bin/env bash
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROOT="$(cd "$REPO/../.." && pwd)"
export LD_LIBRARY_PATH="$ROOT/work/ugv_sdk_install/lib:$ROOT/work/bbs3d_install/lib:$ROOT/work/livox_sdk2_install/lib:${LD_LIBRARY_PATH:-}"
cd "$REPO"
exec python3 -m workbench.server --host 127.0.0.1 --port "${AGT_WORKBENCH_PORT:-8765}"
