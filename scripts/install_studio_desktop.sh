#!/usr/bin/env bash
# Install user-scoped shortcuts only. Never start devices or replace robot data.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
STUDIO_WS="${AGT_FIELD_TEST_WS:-$(cd "$REPO/../.." && pwd)}"
STUDIO_BIN_DIR="$HOME/.local/bin"
usage() {
    cat <<'EOF'
Usage: install_studio_desktop.sh [--workspace DIRECTORY] [--bin-dir DIRECTORY]

Install AGT Robot Studio in the current user's application menu (no sudo).
The workspace must already contain install/setup.bash. ROS/driver/model installation
and colcon builds are separate. Existing calibration, maps and user config are untouched.
XDG_DATA_HOME selects the application-menu location; --bin-dir defaults to ~/.local/bin.
EOF
}
while (($#)); do
    case "$1" in
        -h|--help) usage; exit 0 ;;
        --workspace|--bin-dir)
            (($# >= 2)) || { printf '%s needs a directory\n' "$1" >&2; exit 2; }
            if [[ "$1" == --workspace ]]; then STUDIO_WS="$2"; else STUDIO_BIN_DIR="$2"; fi
            shift 2 ;;
        *) printf 'Unknown option: %s\n' "$1" >&2; exit 2 ;;
    esac
done
[[ -d "$STUDIO_WS" ]] || { printf 'Workspace missing: %s\n' "$STUDIO_WS" >&2; exit 2; }
STUDIO_WS="$(cd "$STUDIO_WS" && pwd)"
[[ -r "$STUDIO_WS/install/setup.bash" ]] || { printf 'Build the workspace first: %s/install/setup.bash is missing\n' "$STUDIO_WS" >&2; exit 2; }

python3 - "$REPO/scripts/launch_studio.sh" "$STUDIO_WS" "$STUDIO_BIN_DIR" "${XDG_DATA_HOME:-$HOME/.local/share}/applications" <<'PY'
import os
from pathlib import Path
import shlex
import sys
import tempfile

script, workspace, bin_dir, apps_dir = (Path(value).expanduser().resolve() for value in sys.argv[1:])
marker = '# AGT Robot Studio managed shortcut'
launcher = bin_dir / 'agt-robot-studio'
desktop = apps_dir / 'agt-robot-studio.desktop'

def desktop_arg(value):
    # Exec is parsed by the desktop launcher, not by a shell. Escape field codes
    # and reserved characters according to Desktop Entry string/Exec rules.
    value = str(value).replace('%', '%%')
    for old, new in (('\\', '\\\\\\\\'), ('"', '\\\\"'), ('`', '\\\\`'), ('$', '\\\\$')):
        value = value.replace(old, new)
    return '"' + value + '"'

wrapper = '#!/usr/bin/env bash\n' + marker + '\nexec /bin/bash ' + shlex.quote(str(script)) + ' --workspace ' + shlex.quote(str(workspace)) + ' "$@"\n'
entry = '\n'.join([
    marker, '[Desktop Entry]', 'Type=Application', 'Version=1.0',
    'Name=AGT Robot Studio', 'Name[zh_CN]=AGT 机器人工作台',
    'Comment=Robot status, navigation and camera gimbal workbench',
    'Exec=' + desktop_arg(launcher) + ' --desktop',
    'Icon=applications-engineering', 'Terminal=false', 'Categories=Science;Robotics;',
    'StartupNotify=true', 'Actions=Demo;', '', '[Desktop Action Demo]',
    'Name=Offline UI demo', 'Name[zh_CN]=离线界面演示',
    'Exec=' + desktop_arg(launcher) + ' --desktop --demo', '',
])

for target in (launcher, desktop):
    if target.exists() and marker not in target.read_text(encoding='utf8').splitlines()[:2]:
        raise SystemExit('Refusing to replace an unrelated file: ' + str(target))
for target, content in ((launcher, wrapper), (desktop, entry)):
    target.parent.mkdir(parents=True, exist_ok=True)
    fd, temp = tempfile.mkstemp(prefix='.' + target.name, dir=target.parent)
    try:
        with os.fdopen(fd, 'w', encoding='utf8') as stream:
            stream.write(content)
        os.chmod(temp, 0o755)
        os.replace(temp, target)
    finally:
        if os.path.exists(temp):
            os.unlink(temp)
print('Installed application menu entry: ' + str(desktop))
print('Launcher: ' + str(launcher))
print('Check environment: ' + shlex.quote(str(launcher)) + ' --check')
print('Offline demo: ' + shlex.quote(str(launcher)) + ' --demo')
print('No GUI or device process was started; no robot configuration was changed.')
PY
