#!/usr/bin/env bash
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROOT="${AGT_FIELD_TEST_WS:-$(cd "$REPO/../.." && pwd)}"
export AGT_FIELD_TEST_WS="$ROOT"
set +u
source /opt/ros/humble/setup.bash
if [[ -f "$ROOT/install/setup.bash" ]]; then source "$ROOT/install/setup.bash"; fi
set -u
python3 "$REPO/scripts/apply_control_integration.py" --workspace "$ROOT"
python3 "$REPO/scripts/apply_sc_kiss_integration.py" --workspace "$ROOT"
python3 "$REPO/scripts/apply_control_integration.py" --workspace "$ROOT" \
  --manifest "$REPO/patches/relocalization_self_filter_manifest.json"
# Reuse existing KISS binaries; compile the accepted locked recipe only if absent.
if ! PYTHONPATH="$REPO/agt_sc_kiss:${PYTHONPATH:-}" python3 -c 'from agt_sc_kiss.localizer import require_kiss; require_kiss()'; then
  python3 "$REPO/scripts/build_kiss_optional.py"
fi
cd "$ROOT"
colcon build --base-paths "$ROOT/src" --packages-select agt_sc_kiss agt_global_relocalization agt_system_bringup agt_field_test_bringup agt_robot_studio \
  --cmake-args "-DSCANCONTEXT_SOURCE_DIR=$ROOT/src/external/scancontext" -DCMAKE_BUILD_TYPE=Release
