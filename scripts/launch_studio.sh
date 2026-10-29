#!/usr/bin/env bash
# Launch the installed Qt application in one explicitly selected ROS workspace.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
STUDIO_WS="${AGT_FIELD_TEST_WS:-$(cd "$REPO/../.." && pwd)}"
STUDIO_ROS_SETUP="${AGT_STUDIO_ROS_SETUP:-/opt/ros/humble/setup.bash}"
STUDIO_CHECK=false
STUDIO_DESKTOP=false
STUDIO_ARGS=()

usage() {
    cat <<'EOF'
Usage: launch_studio.sh [--workspace DIRECTORY] [--check] [--desktop] [--demo] [--theme NAME] [--config FILE]

Launch the installed AGT Robot Studio using ROS 2 Humble and this workspace.
  --workspace DIRECTORY  Colcon workspace containing install/setup.bash.
  --check                Check environment and package imports without starting the GUI.
  --desktop              Save launcher output and show graphical startup errors if available.
  --demo                 Open the application's offline demo; no robot connection.
  --theme NAME           field, industrial, or daylight; appearance only.
  --config FILE          Pass an existing configuration file to the application.

AGT_FIELD_TEST_WS selects the default workspace. AGT_STUDIO_ROS_SETUP can select
an explicit Humble setup file. This launcher does not install ROS or build packages.
EOF
}

fail() {
    local message="$1"
    printf 'AGT Robot Studio: %s\n' "$message" >&2
    if "$STUDIO_DESKTOP" && { [[ -n "${DISPLAY:-}" ]] || [[ -n "${WAYLAND_DISPLAY:-}" ]]; }; then
        if command -v zenity >/dev/null 2>&1; then
            zenity --error --title='AGT Robot Studio 启动失败' --text="$message" 2>/dev/null || true
        elif command -v notify-send >/dev/null 2>&1; then
            notify-send 'AGT Robot Studio 启动失败' "$message" 2>/dev/null || true
        fi
    fi
    exit 2
}

while (($#)); do
    case "$1" in
        -h|--help) usage; exit 0 ;;
        --workspace)
            (($# >= 2)) || fail '--workspace needs a directory'
            STUDIO_WS="$2"; shift 2 ;;
        --check) STUDIO_CHECK=true; shift ;;
        --desktop) STUDIO_DESKTOP=true; shift ;;
        --demo) STUDIO_ARGS+=("$1"); shift ;;
        --theme)
            (($# >= 2)) || fail '--theme needs field, industrial, or daylight'
            case "$2" in field|industrial|daylight) STUDIO_ARGS+=("$1" "$2"); shift 2 ;;
                *) fail '主题必须为 field、industrial 或 daylight' ;; esac ;;
        --config)
            (($# >= 2)) || fail '--config needs a file'
            [[ -f "$2" ]] || fail "配置文件不存在：$2"
            STUDIO_ARGS+=("$1" "$(realpath "$2")"); shift 2 ;;
        --) shift; STUDIO_ARGS+=("$@"); break ;;
        *) fail "未知参数：$1（使用 --help 查看用法）" ;;
    esac
done

if "$STUDIO_DESKTOP"; then
    STUDIO_LOG_DIR="${XDG_STATE_HOME:-$HOME/.local/state}/agt-robot-studio"
    mkdir -p "$STUDIO_LOG_DIR"
    exec >>"$STUDIO_LOG_DIR/launcher.log" 2>&1
    printf '\n[%s] AGT Robot Studio launcher\n' "$(date -Is)"
fi

[[ -d "$STUDIO_WS" ]] || fail "工作区不存在：$STUDIO_WS"
STUDIO_WS="$(cd "$STUDIO_WS" && pwd)"
[[ -r "$STUDIO_ROS_SETUP" ]] || fail "未找到 ROS 2 Humble：$STUDIO_ROS_SETUP。请先完成目标机环境部署。"
[[ -r "$STUDIO_WS/install/setup.bash" ]] || fail "工作区未构建：$STUDIO_WS/install/setup.bash。请先 colcon build --packages-select agt_robot_studio --symlink-install。"
export AGT_FIELD_TEST_WS="$STUDIO_WS"
# ROS setup scripts may read unset variables; restore strict mode afterwards.
set +u
source "$STUDIO_ROS_SETUP"
source "$STUDIO_WS/install/setup.bash"
set -u
command -v ros2 >/dev/null 2>&1 || fail '环境中没有 ros2 命令，请检查 ROS 和工作区 setup.bash。'
if ! STUDIO_PACKAGE_PREFIX="$(ros2 pkg prefix agt_robot_studio 2>/dev/null)"; then
    fail '未找到已构建的 agt_robot_studio；请在选定工作区构建后重新启动。'
fi
if ! python3 -c 'from PyQt5.QtWidgets import QApplication; import numpy, yaml; import agt_robot_studio.ui' >/dev/null 2>&1; then
    fail 'Python/Qt 或工作台依赖不可用。请核对 python3-pyqt5、python3-numpy、python3-yaml 和工作区构建结果。'
fi
export LD_LIBRARY_PATH="$STUDIO_WS/work/ugv_sdk_install/lib:$STUDIO_WS/work/bbs3d_install/lib:$STUDIO_WS/work/livox_sdk2_install/lib:${LD_LIBRARY_PATH:-}"
printf 'Workspace: %s\nInstalled Studio: %s\n' "$STUDIO_WS" "$STUDIO_PACKAGE_PREFIX"
if "$STUDIO_CHECK"; then
    printf 'Environment check passed; no GUI or robot process started.\n'
    exit 0
fi
cd "$STUDIO_WS"
exec ros2 run agt_robot_studio agt_robot_studio "${STUDIO_ARGS[@]}"
