#!/usr/bin/env bash
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO"
exec python3 -m workbench.server --host 127.0.0.1 --port "${AGT_WORKBENCH_PORT:-8765}"
