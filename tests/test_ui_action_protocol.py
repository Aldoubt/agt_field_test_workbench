"""Production backend ledger JSON consumed by the production UI tracker."""
import json
from pathlib import Path
import sys

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'agt_robot_studio'))
from agt_robot_studio.operator_actions import ActionTracker
from workbench.action_status import ActionLedger


def wire(value):
    return json.loads(json.dumps(value))


def test_backend_receipt_and_result_join_the_ui_request_without_idle_gap():
    ledger=ActionLedger();ui=ActionTracker();payload=ui.begin('route')
    record,new=ledger.begin('route',dict(payload,map_id='map-1',route_id='route-1'))
    assert new
    ui.acknowledge(wire({'action':record}));ui.observe(wire(ledger.snapshot()))
    assert ui.busy
    ledger.update(record['action_id'],'ACCEPTED');ui.observe(wire(ledger.snapshot()))
    assert ui.busy
    ledger.update(record['action_id'],'SUCCEEDED');ui.observe(wire(ledger.snapshot()))
    assert not ui.busy and '已完成' in ui.summary()


def test_real_stop_receipt_needs_live_terminal_and_does_not_claim_standstill():
    ledger=ActionLedger();ui=ActionTracker();payload=ui.begin('goal')
    goal,_=ledger.begin('goal',payload);ui.acknowledge({'action':goal})
    stop_payload=ui.begin('stop');assert stop_payload['target_request_id']==payload['request_id']
    stop,_=ledger.begin('stop',stop_payload)
    ui.acknowledge({'action':ledger.update(stop['action_id'],'STOP_REQUESTED')})
    assert ui.busy
    ledger.update(goal['action_id'],'CANCELED')
    terminal=ledger.update(stop['action_id'],'DISPATCHED')
    ui.acknowledge(wire({'action':terminal}));assert ui.busy
    ui.observe(wire(ledger.snapshot()))
    assert not ui.busy and '需核对实际停稳' in ui.summary()


def test_rejected_command_is_correlated_and_duplicate_does_not_dispatch_again():
    ledger=ActionLedger();ui=ActionTracker();payload=ui.begin('goal')
    record,_=ledger.begin('goal',payload)
    rejected=ledger.update(record['action_id'],'REJECTED','localization not fresh')
    ui.acknowledge(wire({'action':rejected}));ui.failed_transport('goal','HTTP400')
    ui.observe(wire(ledger.snapshot()))
    assert not ui.busy and 'localization not fresh' in ui.summary()
    duplicate,new=ledger.begin('goal',payload)
    assert not new and duplicate['action_id']==record['action_id']
