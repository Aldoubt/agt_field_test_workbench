#!/usr/bin/env bash
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROOT="$(cd "$REPO/../.." && pwd)"
if [[ "$REPO" != "$ROOT/src/agt_field_test_workbench" ]]; then
  echo 'Clone this repository at ~/agt_field_test_ws/src/agt_field_test_workbench' >&2
  exit 2
fi
mkdir -p "$ROOT"/{calibration,maps,bags,runs,reports,work,config}
if [[ -e "$ROOT/.repos" ]]; then
  cmp -s "$ROOT/.repos" "$REPO/dependencies/field_test.repos" || {
    echo 'Existing .repos differs; preserved. Resolve before proceeding.' >&2; exit 2;
  }
else
  cp "$REPO/dependencies/field_test.repos" "$ROOT/.repos"
fi
cp -an "$REPO/templates/config/." "$ROOT/config/"
mkdir -p "$ROOT/calibration/bunker" "$ROOT/calibration/fwmini"
python3 "$REPO/scripts/fetch_locked.py"
python3 "$REPO/scripts/apply_mapping_calibration.py"
python3 "$REPO/scripts/audit.py" > "$ROOT/reports/dependency_audit.json"
echo "Initialized $ROOT; next run scripts/build_humble.sh on Ubuntu 22.04/Humble."
