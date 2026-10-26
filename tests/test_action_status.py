from concurrent.futures import ThreadPoolExecutor

import pytest

from workbench.action_status import ActionLedger, REQUEST_LIMIT


def test_movement_sequence_and_terminal_results():
    ledger = ActionLedger()
    action, is_new = ledger.begin('goal', {'context': {'map_id': 'orchard', 'robot': 'bunker'}})
    assert is_new and action['state'] == 'REQUESTED'
    assert action['map_id'] == 'orchard' and action['robot'] == 'bunker'
    seq = action['seq']
    for state in ('ARMING', 'ACCEPTED', 'RUNNING', 'SUCCEEDED'):
        action = ledger.update(action['action_id'], state)
        assert action['seq'] > seq
        seq = action['seq']
    assert action['terminal'] is True
    assert not ledger.busy
    snapshot = ledger.snapshot()
    assert snapshot['sequence'] == seq
    assert snapshot['current'] is None
    assert snapshot['last'] == action


def test_request_deduplication_and_conflicts():
    ledger = ActionLedger()
    payload = {'request_id': 'goal:1', 'x': 1, 'y': 2}
    first, _ = ledger.begin('goal', payload)
    duplicate, is_new = ledger.begin('goal', dict(reversed(list(payload.items()))))
    assert not is_new and duplicate == first
    assert ledger.snapshot()['sequence'] == first['seq']
    with pytest.raises(ValueError, match='different'):
        ledger.begin('goal', dict(payload, x=3))
    with pytest.raises(ValueError, match='different'):
        ledger.begin('route', payload)
    finished = ledger.update(first['action_id'], 'SUCCEEDED')
    assert ledger.begin('goal', payload) == (finished, False)


@pytest.mark.parametrize('request_id', ['', 'x' * 129, 'has space', '../x', '\n', 3, None])
def test_request_id_validation(request_id):
    with pytest.raises(ValueError, match='request_id'):
        ActionLedger().begin('goal', {'request_id': request_id})


def test_all_generated_request_ids_are_distinct():
    ledger = ActionLedger()
    first, _ = ledger.begin('goal', {})
    second, _ = ledger.begin('goal', {})
    assert first['request_id'] != second['request_id']
    assert first['action_id'] != second['action_id']


def test_cancel_race_ignores_late_acceptance_and_feedback():
    ledger = ActionLedger()
    action, _ = ledger.begin('route', {'route_id': 'row-1'})
    canceled = ledger.update(action['action_id'], 'CANCEL_REQUESTED', reason='operator stop')
    for late_state in ('REQUESTED', 'ARMING', 'ACCEPTED', 'RUNNING'):
        assert ledger.update(action['action_id'], late_state, progress=1) == canceled
    assert ledger.busy
    terminal = ledger.update(action['action_id'], 'CANCELED')
    for late_state in ('ACCEPTED', 'RUNNING', 'SUCCEEDED', 'FAILED'):
        assert ledger.update(action['action_id'], late_state) == terminal
    assert not ledger.busy
    assert ledger.snapshot()['sequence'] == terminal['seq']


def test_terminal_result_can_arrive_after_cancel_request():
    ledger = ActionLedger()
    action, _ = ledger.begin('goal')
    ledger.update(action['action_id'], 'CANCEL_REQUESTED')
    result = ledger.update(action['action_id'], 'SUCCEEDED', reason='result won cancellation race')
    assert result['state'] == 'SUCCEEDED'
    assert result['terminal']


def test_old_motion_stays_busy_when_new_request_is_rejected():
    ledger = ActionLedger()
    old, _ = ledger.begin('distance')
    ledger.update(old['action_id'], 'RUNNING')
    new, _ = ledger.begin('goal')
    assert ledger.busy_except(new['action_id'])
    ledger.update(new['action_id'], 'REJECTED', reason='another action is active')
    snapshot = ledger.snapshot()
    assert snapshot['busy'] and ledger.busy
    assert snapshot['current']['action_id'] == old['action_id']
    assert snapshot['last']['action_id'] == new['action_id']
    assert not ledger.busy_except(old['action_id'])


def test_nonmovement_failure_or_dispatch_is_not_motion():
    ledger = ActionLedger()
    action, _ = ledger.begin('gimbal')
    assert not ledger.busy
    assert ledger.snapshot()['current'] is None
    dispatched = ledger.update(action['action_id'], 'DISPATCHED')
    assert dispatched['terminal']
    stop, _ = ledger.begin('stop')
    assert ledger.update(stop['action_id'], 'DISPATCHED')['state'] == 'DISPATCHED'
    assert not ledger.busy


def test_stop_waits_for_target_result_without_implying_physical_stop():
    ledger = ActionLedger()
    goal, _ = ledger.begin('goal')
    ledger.update(goal['action_id'], 'RUNNING')
    stop, _ = ledger.begin('stop')
    waiting = ledger.update(stop['action_id'], 'STOP_REQUESTED', targets=[goal['action_id']])
    assert waiting['terminal'] is False
    assert ledger.busy
    assert ledger.snapshot()['current']['action_id'] == goal['action_id']
    assert ledger.update(stop['action_id'], 'ACCEPTED') == waiting
    ledger.update(goal['action_id'], 'CANCELED')
    done = ledger.update(stop['action_id'], 'DISPATCHED', reason='target cancellation acknowledged')
    assert done['terminal'] and done['state'] == 'DISPATCHED'
    assert not ledger.busy


def test_all_public_records_are_deep_copies():
    ledger = ActionLedger()
    payload = {'request_id': 'copy-1', 'context': {'map_id': 'orchard'}, 'points': [{'x': 1}]}
    action, _ = ledger.begin('route', payload)
    payload['points'][0]['x'] = 9
    action['payload']['points'][0]['x'] = 8
    action['context']['map_id'] = 'wrong'
    stored = ledger.get(action['action_id'])
    assert stored['payload']['points'][0]['x'] == 1
    assert stored['context']['map_id'] == 'orchard'
    snapshot = ledger.snapshot()
    snapshot['recent'][0]['payload']['points'][0]['x'] = 7
    snapshot['current']['state'] = 'SUCCEEDED'
    assert ledger.get(action['action_id'])['state'] == 'REQUESTED'
    assert ledger.get(action['action_id'])['payload']['points'][0]['x'] == 1
    result = {'distance': [1, 2]}
    updated = ledger.update(action['action_id'], 'RUNNING', result=result)
    result['distance'][0] = 8
    updated['result']['distance'][0] = 9
    assert ledger.get(action['action_id'])['result']['distance'] == [1, 2]


def test_retention_keeps_pending_motion_and_terminal_tombstones():
    ledger = ActionLedger()
    pending, _ = ledger.begin('goal', {'request_id': 'pending'})
    first, _ = ledger.begin('gimbal', {'request_id': 'terminal-first'})
    ledger.update(first['action_id'], 'DISPATCHED')
    for index in range(160):
        action, _ = ledger.begin('gimbal', {'request_id': 'extra-' + str(index)})
        ledger.update(action['action_id'], 'DISPATCHED')
    assert ledger.busy
    assert ledger.busy_except(None)
    assert ledger.snapshot()['current']['action_id'] == pending['action_id']
    assert len(ledger.snapshot()['recent']) == 32
    assert len(ledger._records) == 128
    summary, is_new = ledger.begin('gimbal', {'request_id': 'terminal-first'})
    assert not is_new and summary['action_id'] == first['action_id']
    assert summary['terminal'] and summary['state'] == 'DISPATCHED'
    assert 'payload' not in summary
    assert ledger.update(first['action_id'], 'RUNNING') == summary
    with pytest.raises(ValueError, match='different'):
        ledger.begin('gimbal', {'request_id': 'terminal-first', 'angle': 3})


def test_capacity_cannot_allow_a_repeated_motion_and_does_not_block_stop():
    ledger = ActionLedger()
    first_payload = {'request_id': 'request-0'}
    first = None
    for index in range(REQUEST_LIMIT):
        action, _ = ledger.begin('goal', {'request_id': 'request-' + str(index)})
        if index == 0:
            first = action
        ledger.update(action['action_id'], 'SUCCEEDED')
    with pytest.raises(ValueError, match='restart the bridge'):
        ledger.begin('goal', {'request_id': 'one-too-many'})
    duplicate, is_new = ledger.begin('goal', first_payload)
    assert not is_new and duplicate['action_id'] == first['action_id']
    stop, is_new = ledger.begin('stop', {'request_id': 'safety-stop'})
    assert is_new
    assert ledger.update(stop['action_id'], 'DISPATCHED')['terminal']


def test_bridge_restart_changes_identity_and_does_not_share_state():
    first, second = ActionLedger(), ActionLedger()
    a, _ = first.begin('goal', {'request_id': 'caller-1'})
    b, is_new = second.begin('goal', {'request_id': 'caller-1'})
    assert is_new
    assert a['bridge_id'] != b['bridge_id']
    assert a['action_id'] != b['action_id']
    assert second.get(a['action_id']) is None


def test_concurrent_duplicate_requests_have_one_dispatch_owner():
    ledger = ActionLedger()
    with ThreadPoolExecutor(max_workers=8) as executor:
        results = list(executor.map(
            lambda _: ledger.begin('goal', {'request_id': 'concurrent-1'}), range(24)))
    assert sum(is_new for _, is_new in results) == 1
    assert len({record['action_id'] for record, _ in results}) == 1
    assert ledger.snapshot()['sequence'] == 1


def test_invalid_update_cannot_replace_action_identity():
    ledger = ActionLedger()
    action, _ = ledger.begin('goal')
    with pytest.raises(ValueError, match='lifecycle'):
        ledger.update(action['action_id'], 'RUNNING', request_id='replacement')
    with pytest.raises(ValueError, match='finite JSON'):
        ledger.update(action['action_id'], 'RUNNING', distance=float('nan'))
    with pytest.raises(ValueError, match='unknown action state'):
        ledger.update(action['action_id'], 'UNKNOWN')
    with pytest.raises(KeyError):
        ledger.update('missing', 'FAILED')
    assert ledger.get(action['action_id']) == action
