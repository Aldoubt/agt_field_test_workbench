"""Static regression checks; run without ROS, Qt, vehicle or internet."""
import ast
from pathlib import Path


PKG = Path(__file__).resolve().parents[1]


def test_gui_never_publishes_twist_or_runs_closed_loop_servo():
    code = (PKG/'agt_robot_studio'/'ros_worker.py').read_text()
    tree = ast.parse(code)
    calls = [n for n in ast.walk(tree) if isinstance(n, ast.Call)]
    names = [n.func.attr for n in calls if isinstance(n.func, ast.Attribute)]
    assert 'create_publisher' not in names
    assert 'publish' not in names
    assert 'ClosedLoopMotion' not in code
    assert 'DriveOnHeading' in code and 'Spin' in code


def test_odom_is_display_only_not_a_hard_kill_gate():
    code = (PKG/'agt_robot_studio'/'ros_worker.py').read_text()
    assert 'odom_max_age_s' not in code
    assert 'obstacle_clear_topic' not in code
    assert 'spin_once(node, timeout_sec=.04)' in code
    assert '_decode_queue' in code and '_decode_loop' in code


def test_motion_defaults_fail_closed_and_reverse_blocked():
    text = (PKG/'config'/'studio.yaml').read_text()
    assert 'live_motion_enabled: false' in text
    assert 'allow_reverse: false' in text
    assert 'guard_max_age_s: 1.5' in text
