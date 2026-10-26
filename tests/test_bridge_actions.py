"""Execute production bridge callbacks with deterministic ROS action futures.

Only ROS messages/clients and hardware checks are injected. The bridge's actual
request dispatch, state transitions, revalidation, cancellation and receipts run.
No ROS graph, hardware driver, or motion command leaves this process.
"""
import ast
from concurrent.futures import Future
import json
import math
import re
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock

import pytest

from workbench.action_status import ActionLedger
from workbench.control_error import ControlError

REPO = Path(__file__).resolve().parents[1]


class Publisher:
    def __init__(self): self.messages = []
    def publish(self, message): self.messages.append(message)


class Handle:
    def __init__(self, accepted=True):
        self.accepted = accepted
        self.result = Future()
        self.cancels = []
    def get_result_async(self): return self.result
    def cancel_goal_async(self):
        future = Future()
        self.cancels.append(future)
        return future


class Client:
    def __init__(self): self.sent = []
    def server_is_ready(self): return True
    def send_goal_async(self, goal, **kwargs):
        future = Future()
        self.sent.append((goal, kwargs, future))
        return future


def stamped():
    return NS(header=NS(frame_id='', stamp=None),
              pose=NS(position=NS(x=0., y=0.), orientation=NS(z=0., w=1.)))


@pytest.fixture
def bridge():
    tree = ast.parse((REPO / 'scripts/ros_bridge.py').read_text())
    production = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == 'Bridge')
    names = {'action_receipt', '_finish_action', '_finish_stops', '_cancel_handle',
             '_submit_action', 'goal', 'start_mission', 'completed', 'stop', 'dispatch',
             '_dispatch', 'reject_queued_action'}
    methods = [node for node in production.body if isinstance(node, ast.FunctionDef) and node.name in names]
    assert len(methods) == len(names)
    isolated = ast.Module(body=[ast.ClassDef(name='Bridge', bases=[], keywords=[], body=methods,
                                            decorator_list=[])], type_ignores=[])
    validate = Mock()
    namespace = dict(ActionLedger=ActionLedger, ControlError=ControlError, math=math, json=json, re=re,
                     Path=Path, ROOT=Path('/unused'), String=NS, Bool=NS, Twist=NS,
                     SetBool=NS(Request=NS), PoseStamped=stamped,
                     NavigateToPose=NS(Goal=NS), FollowWaypoints=NS(Goal=NS),
                     ExecuteInspectionMission=NS(Goal=NS),
                     Maps=lambda root: NS(get=lambda identity: {'id': identity}),
                     validate_endpoints=validate, pose=lambda p: dict(p))
    exec(compile(ast.fix_missing_locations(isolated), 'production_bridge_callbacks', 'exec'), namespace)
    node = namespace['Bridge']()
    node.actions = ActionLedger()
    node.goals = []
    node.goal_actions = {}
    node.action_handles = {}
    node.goal_result = None
    node.sent_action_ids = set()
    node.stop_waiters = {}
    node.cancelled_request_ids = set()
    node.goal_epoch = 0
    node.pending_goal = False
    node.pending_action_id = None
    node.enable_future = None
    node.lease = None
    node.closed_leases = set()
    node.jog_until = 0
    node.mode, node.cmd, node.deadman = Publisher(), Publisher(), Publisher()
    node.session = dict(robot='bunker', map_id='map', profile={}, task='navigation')
    node.current = lambda: node.session
    node.check_navigation = Mock(side_effect=lambda: node.session)
    node.get_clock = lambda: NS(now=lambda: NS(to_msg=lambda: 'stamp'))
    node.get_logger = lambda: NS(warning=lambda message: None)
    node.navigate, node.waypoints, node.mission = Client(), Client(), Client()
    node.samples = {}
    node.record = Mock()
    node.service_calls = []
    def service(*args):
        future = Future()
        node.service_calls.append((args, future))
        return future
    node.call_service = service
    node.validate = validate
    node.namespace = namespace
    return node


def payload(request='request-1', **changes):
    return dict(request_id=request, robot='bunker', map_id='map', pose=dict(x=1., y=2., yaw=.2), **changes)


def start(bridge, request='request-1'):
    receipt = bridge.dispatch('goal', payload(request))
    return receipt, bridge.navigate.sent[-1][2]


def finish(handle, status=4, **result):
    handle.result.set_result(NS(status=status, result=NS(**result)))


def test_requested_accepted_and_result_are_distinct_and_correlated(bridge):
    receipt, acceptance = start(bridge)
    assert receipt['state'] == 'REQUESTED' and not receipt['terminal']
    assert bridge.actions.busy and bridge.goal_result is None
    handle = Handle()
    acceptance.set_result(handle)
    assert bridge.actions.get(receipt['action_id'])['state'] == 'ACCEPTED'
    assert bridge.goal_result is None
    finish(handle)
    result = bridge.goal_result
    assert result['state'] == 'SUCCEEDED'
    for name in ('action_id', 'request_id', 'bridge_id'):
        assert result[name] == receipt[name]
    assert result['seq'] > receipt['seq']
    assert not bridge.actions.busy
    bridge.validate.assert_called_once()


def test_repeat_request_is_not_sent_twice_and_conflict_is_rejected(bridge):
    first, acceptance = start(bridge)
    repeat = bridge.dispatch('goal', payload())
    assert repeat['duplicate'] and repeat['action_id'] == first['action_id']
    assert len(bridge.navigate.sent) == 1
    changed = payload(); changed['pose']['x'] = 9.
    with pytest.raises(ValueError, match='different action payload'):
        bridge.dispatch('goal', changed)
    assert len(bridge.navigate.sent) == 1
    acceptance.set_result(Handle(False))
    with pytest.raises(ControlError) as error:
        bridge.dispatch('goal', payload())
    assert error.value.action['action_id'] == first['action_id']
    assert error.value.action['state'] == 'REJECTED'


def test_second_motion_is_rejected_without_unlocking_first(bridge):
    first, _ = start(bridge)
    with pytest.raises(ControlError) as error:
        bridge.dispatch('goal', payload('request-2'))
    assert error.value.action['state'] == 'REJECTED'
    assert error.value.action['action_id'] != first['action_id']
    assert bridge.actions.snapshot()['current']['action_id'] == first['action_id']
    assert bridge.pending_goal and len(bridge.navigate.sent) == 1


def test_precheck_revalidation_failure_is_terminal_and_visible(bridge):
    bridge.check_navigation.side_effect = [bridge.session, ValueError('localization became stale')]
    with pytest.raises(ControlError, match='localization became stale') as error:
        bridge.dispatch('goal', payload())
    assert error.value.action['state'] == 'REJECTED'
    assert error.value.action['terminal']
    assert bridge.actions.snapshot()['last']['action_id'] == error.value.action['action_id']
    assert bridge.navigate.sent == []
    assert not bridge.pending_goal and not bridge.actions.busy


def test_precheck_identity_failure_and_queue_expiry_keep_request_identity(bridge):
    data = payload(); data['map_id'] = 'other-map'
    with pytest.raises(ControlError) as error:
        bridge.dispatch('goal', data)
    assert error.value.action['request_id'] == data['request_id']
    assert error.value.action['state'] == 'REJECTED'
    with pytest.raises(ControlError) as error:
        bridge.reject_queued_action('goal', payload('expired'), 'queued operation expired')
    assert error.value.action['request_id'] == 'expired' and error.value.action['terminal']
    assert bridge.navigate.sent == []


def test_action_server_rejection_has_its_own_terminal_result(bridge):
    receipt, acceptance = start(bridge)
    acceptance.set_result(Handle(False))
    result = bridge.actions.get(receipt['action_id'])
    assert result['state'] == 'REJECTED' and result['terminal']
    assert bridge.goal_result['action_id'] == receipt['action_id']
    assert not bridge.actions.busy


@pytest.mark.parametrize('result', [{'success': False}, {'missed_waypoints': [2]}, {'error_code': 7}])
def test_success_transport_status_does_not_hide_failed_task_result(bridge, result):
    receipt, acceptance = start(bridge)
    handle = Handle(); acceptance.set_result(handle)
    finish(handle, **result)
    assert bridge.actions.get(receipt['action_id'])['state'] == 'FAILED'


def test_stop_waits_for_real_cancel_result_and_cancels_late_acceptance(bridge):
    goal, acceptance = start(bridge)
    stop = bridge.dispatch('stop', dict(request_id='stop-1', target_request_id=goal['request_id']))
    assert stop['state'] == 'STOP_REQUESTED' and not stop['terminal']
    assert bridge.actions.get(goal['action_id'])['state'] == 'CANCEL_REQUESTED'
    handle = Handle(); acceptance.set_result(handle)
    assert len(handle.cancels) == 1
    assert bridge.actions.busy
    assert not bridge.actions.get(stop['action_id'])['terminal']
    handle.cancels[0].set_result(NS(return_code=0))
    assert not bridge.actions.get(stop['action_id'])['terminal']  # cancel ACK is not completion
    finish(handle, 5)
    assert bridge.actions.get(goal['action_id'])['state'] == 'CANCELED'
    done = bridge.actions.get(stop['action_id'])
    assert done['state'] == 'DISPATCHED' and done['terminal'] and done['seq'] > stop['seq']
    assert not bridge.actions.busy


def test_stop_invalidates_unsent_arming_request(bridge):
    bridge.session['robot'] = 'fwmini'
    bridge.samples['fwmini'] = NS(data=json.dumps({'state': 'READY'}))
    data = payload(); data['robot'] = 'fwmini'
    goal = bridge.dispatch('goal', data)
    arm = bridge.service_calls[0][1]
    assert bridge.navigate.sent == []
    stop = bridge.dispatch('stop', {'request_id': 'stop-1'})
    assert stop['action']['state'] == 'DISPATCHED' and stop['terminal']
    assert bridge.actions.get(goal['action_id'])['state'] == 'CANCELED'
    arm.set_result(NS(success=True))
    assert bridge.navigate.sent == []


def test_stop_tombstone_blocks_original_request_arriving_after_stop(bridge):
    stop = bridge.dispatch('stop', dict(request_id='stop-1', target_request_id='late-request'))
    assert stop['terminal'] and stop['action']['state'] == 'DISPATCHED'
    with pytest.raises(ControlError, match='canceled by an earlier stop'):
        bridge.dispatch('goal', payload('late-request'))
    assert bridge.navigate.sent == []
    assert bridge.actions.snapshot()['last']['state'] == 'REJECTED'


@pytest.mark.parametrize('phase', ['acceptance', 'result'])
def test_unobservable_sent_action_remains_locked_as_unknown(bridge, phase):
    receipt, acceptance = start(bridge)
    if phase == 'acceptance':
        acceptance.set_exception(RuntimeError('transport lost'))
    else:
        handle = Handle(); acceptance.set_result(handle)
        handle.result.set_exception(RuntimeError('transport lost'))
    action = bridge.actions.get(receipt['action_id'])
    assert action['state'] == 'CANCEL_REQUESTED' and not action['terminal']
    assert 'unknown' in action['reason'] and bridge.actions.busy
    with pytest.raises(ControlError): bridge.dispatch('goal', payload('second'))
    assert len(bridge.navigate.sent) == 1


def test_send_interface_exception_is_unknown_and_never_replayed(bridge):
    bridge.navigate.send_goal_async = Mock(side_effect=RuntimeError('send transport lost'))
    with pytest.raises(ControlError) as error:
        bridge.dispatch('goal', payload())
    action = error.value.action
    assert error.value.outcome_unknown
    assert action['state'] == 'CANCEL_REQUESTED' and not action['terminal']
    assert bridge.actions.busy
    with pytest.raises(ControlError) as repeated:
        bridge.dispatch('goal', payload())
    assert repeated.value.action['action_id'] == action['action_id']
    assert bridge.navigate.send_goal_async.call_count == 1


def test_cancel_rejection_visible_and_explicit_second_stop_can_resend(bridge):
    goal, acceptance = start(bridge)
    handle = Handle(); acceptance.set_result(handle)
    first_stop = bridge.dispatch('stop', {'request_id': 'stop-1'})
    handle.cancels[0].set_result(NS(return_code=1))
    assert 'rejected cancellation' in bridge.actions.get(goal['action_id'])['reason']
    assert not bridge.actions.get(first_stop['action_id'])['terminal']
    bridge.dispatch('stop', {'request_id': 'stop-2'})
    assert len(handle.cancels) == 2
    finish(handle, 5)
    assert bridge.actions.get(first_stop['action_id'])['terminal']


def test_late_old_result_does_not_complete_a_new_action(bridge):
    first, acceptance = start(bridge)
    old_handle = Handle(); acceptance.set_result(old_handle); finish(old_handle)
    second, _ = start(bridge, 'request-2')
    bridge.completed(NS(status=4, result=NS()), old_handle, first['action_id'])
    assert bridge.actions.get(second['action_id'])['state'] == 'REQUESTED'
    assert bridge.pending_goal and bridge.actions.busy
    assert bridge.goal_result['action_id'] == first['action_id']


@pytest.mark.parametrize('invalid', [{}, [], '', 'has space'])
def test_bad_optional_stop_correlation_cannot_prevent_zero_or_cancel(bridge, invalid):
    _, acceptance = start(bridge)
    handle = Handle(); acceptance.set_result(handle)
    stop = bridge.dispatch('stop', {'request_id': 'stop-1', 'target_request_id': invalid})
    assert bridge.cmd.messages and bridge.deadman.messages[-1].data is False
    assert len(handle.cancels) == 1
    assert stop['action']['correlation_warning']
    assert not stop['terminal']


def test_route_dispatch_preserves_route_identity_and_revalidates_endpoints(bridge):
    points = [dict(x=1., y=2., yaw=.1), dict(x=2., y=3., yaw=.2)]
    validate_route = Mock(return_value={'id': 'route-1', 'points': points})
    bridge.namespace['Routes'] = lambda root: NS(validate=validate_route)
    receipt = bridge.dispatch('route', dict(request_id='route-request', robot='bunker',
                                             map_id='map', route_id='route-1'))
    assert receipt['action']['kind'] == 'route' and receipt['action']['route_id'] == 'route-1'
    assert len(bridge.waypoints.sent) == 1 and not bridge.navigate.sent
    validate_route.assert_called_once_with('route-1', 'map')
    assert bridge.validate.call_args.args[1] == points
    handle = Handle(); bridge.waypoints.sent[0][2].set_result(handle)
    finish(handle, missed_waypoints=[])
    assert bridge.actions.get(receipt['action_id'])['state'] == 'SUCCEEDED'


@pytest.mark.parametrize('camera_ready', [True, False])
def test_photo_mission_keeps_health_gate_and_correlated_result(bridge, tmp_path, camera_ready):
    directory = tmp_path / 'route-1'; directory.mkdir()
    mission = directory / 'mission.yaml'; mission.write_text('fixture')
    route = dict(id='route-1', points=[dict(x=1., y=2., yaw=.1, photo=True)], mission_file=str(mission))
    bridge.namespace['Routes'] = lambda root: NS(directory=tmp_path, validate=lambda *args: route)
    bridge.namespace['CapabilityHealth'] = object()
    bridge.namespace['get_action_names_and_types'] = lambda node: [
        ('/camera_gimbal/acquire_view', ['camera_gimbal_interfaces/action/AcquireView'])]
    bridge.samples['camera'] = NS(state=0, STATE_READY=0, camera_alive=camera_ready,
                                 gimbal_serial_connected=True, gimbal_feedback_alive=True,
                                 move_action_ready=True, busy=False)
    bridge.fresh = lambda *args: True
    data = dict(request_id='mission-request', robot='bunker', map_id='map', route_id='route-1')
    if not camera_ready:
        with pytest.raises(ControlError, match='health is not READY') as error:
            bridge.dispatch('route', data)
        assert error.value.action['state'] == 'REJECTED' and not bridge.mission.sent
        return
    receipt = bridge.dispatch('route', data)
    assert receipt['executor'] == 'agt_navigation_runtime/mission_runtime'
    assert len(bridge.mission.sent) == 1
    handle = Handle(); bridge.mission.sent[0][2].set_result(handle)
    finish(handle, success=False, error_code=12, message='capture failed')
    assert bridge.goal_result['action_id'] == receipt['action_id']
    assert bridge.goal_result['state'] == 'FAILED'
