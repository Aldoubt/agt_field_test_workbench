"""Shared stop-and-shoot runtime configuration for CLI and Qt sessions."""
from pathlib import Path


def photo_runtime_parameters(behavior_tree, record_root, robot='bunker'):
    if robot not in ('bunker', 'fwmini'):
        raise ValueError('unknown photo runtime robot')
    tree = Path(behavior_tree).expanduser().resolve()
    if not tree.is_file():
        raise ValueError('photo behavior tree is missing: ' + str(tree))
    return {
        'record_root': str(record_root),
        'nav_behavior_tree': str(tree),
        'recenter_after_capture': True,
        'stationary_timeout_sec': 15.0,
        'wait_for_bunker_control': robot == 'bunker',
        'wheel_stop_gate': robot == 'bunker',
    }
