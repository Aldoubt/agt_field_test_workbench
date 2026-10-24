"""Architecture checks: Qt is an operator UI, never a second motion backend."""
import ast
from pathlib import Path

PKG = Path(__file__).resolve().parents[1]


def test_ros_adapter_is_read_only_and_has_no_motion_action_client():
    code = (PKG/'agt_robot_studio'/'ros_worker.py').read_text()
    tree = ast.parse(code)
    names = [n.func.attr for n in ast.walk(tree) if isinstance(n, ast.Call)
             and isinstance(n.func, ast.Attribute)]
    assert 'create_publisher' not in names
    assert 'ActionClient' not in code
    assert 'DriveOnHeading' not in code
    assert 'ClosedLoopMotion' not in code


def test_ui_reuses_workbench_for_all_motion_and_relocalization():
    code = (PKG/'agt_robot_studio'/'ui.py').read_text()
    assert "'/api/control/goal'" in code
    assert "'/api/control/distance'" in code
    assert "'/api/control/relocalize'" in code
    assert "'/api/control/stop'" in code
    assert 'create_publisher' not in code


def test_map_requires_yaml_and_keeps_unknown():
    ui = (PKG/'agt_robot_studio'/'ui.py').read_text()
    assert '没有配套 YAML 时拒绝猜测分辨率和原点' in ui
    assert "'未知（灰）',205" in ui
