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
  if ! cmp -s "$ROOT/.repos" "$REPO/dependencies/field_test.repos"; then
    python3 - "$ROOT/.repos" "$REPO/dependencies/field_test.repos" <<'PY'
import shutil
import sys
import yaml

current_path, new_path = sys.argv[1:]
current = yaml.safe_load(open(current_path, encoding='utf-8'))['repositories']
new = yaml.safe_load(open(new_path, encoding='utf-8'))['repositories']
if not all(new.get(name) == spec for name, spec in current.items()):
    raise SystemExit('Existing .repos has changed pins; preserved. Resolve before proceeding.')
shutil.copyfile(new_path, current_path)
print(f'Updated .repos with {len(new) - len(current)} new locked repositories')
PY
  fi
else
  cp "$REPO/dependencies/field_test.repos" "$ROOT/.repos"
fi
cp -an "$REPO/templates/config/." "$ROOT/config/"
# Preserve operator-owned entrypoints; only install missing wrappers.
cp -an "$REPO/templates/bin/." "$ROOT/"
mkdir -p "$ROOT/calibration/bunker" "$ROOT/calibration/fwmini"
cp -n "$REPO/agt_field_test_bringup/config/fwmini_mid360_candidate.yaml" \
  "$ROOT/calibration/fwmini/fwmini_mid360_candidate.yaml"
cp -n "$REPO/agt_field_test_bringup/config/bunker_calibration_candidate.yaml" "$ROOT/calibration/bunker/bunker_calibration_candidate.yaml"
python3 "$REPO/scripts/fetch_locked.py"
python3 "$REPO/scripts/apply_mapping_calibration.py"
python3 "$REPO/scripts/apply_control_integration.py"
python3 "$REPO/scripts/apply_sc_kiss_integration.py"
python3 "$REPO/scripts/apply_control_integration.py" --manifest "$REPO/patches/navigation_perception_manifest.json"
python3 "$REPO/scripts/apply_control_integration.py" --manifest "$REPO/patches/photo_mission_manifest.json"
python3 "$REPO/scripts/apply_control_integration.py" --manifest "$REPO/patches/map_tracking_support_manifest.json"
python3 "$REPO/scripts/apply_control_integration.py" --manifest "$REPO/patches/ugv_protocol_lifetime_manifest.json"
python3 "$REPO/scripts/audit.py" > "$ROOT/reports/dependency_audit.json"
echo "Initialized $ROOT; next run scripts/build_humble.sh on Ubuntu 22.04/Humble."
