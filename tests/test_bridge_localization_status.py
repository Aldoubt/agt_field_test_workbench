"""Read-only snapshot serialization; no ROS node or publisher is created."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock


def test_localization_status_serializes_manager_and_receipt_freshness():
    spec = importlib.util.spec_from_file_location(
        'studio_status_bridge', Path(__file__).resolve().parents[1]/'scripts/ros_bridge.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    fake = SimpleNamespace(samples={}, fresh=Mock(return_value=True))
    assert module.Bridge.localization_status(fake) is None
    fake.fresh.assert_not_called()
    values = dict(state=2, reason='waiting_global_pose', local_odom_fresh=True,
                  local_odom_age_sec=.02, global_correction_valid=False)
    fake.samples['localization'] = SimpleNamespace(**values)
    assert module.Bridge.localization_status(fake) == dict(values, fresh=True)
    fake.fresh.assert_called_once_with('localization', .5)
    fake.fresh.return_value = False
    assert module.Bridge.localization_status(fake) == dict(values, fresh=False)
