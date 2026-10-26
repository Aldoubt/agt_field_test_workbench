from agt_robot_studio.operator_actions import ActionTracker


def action(request, state='ACCEPTED', seq=2, **extra):
    return dict(request_id=request['request_id'], action_id='a1',bridge_id='b1',
                state=state,terminal=state in ('SUCCEEDED','REJECTED','DISPATCHED'),seq=seq,**extra)


def test_acknowledgement_and_idle_snapshot_do_not_unlock_motion():
    tracker=ActionTracker();request=tracker.begin('goal')
    tracker.acknowledge({'action':action(request)})
    tracker.observe({'bridge_id':'b1','recent':[]})
    assert tracker.busy
    tracker.observe({'bridge_id':'b1','recent':[action(request,'SUCCEEDED',3)]})
    assert not tracker.busy


def test_unrelated_and_old_terminal_results_cannot_complete_new_request():
    tracker=ActionTracker();request=tracker.begin('route')
    tracker.acknowledge({'action':action(request,seq=4)})
    tracker.observe({'bridge_id':'b1','recent':[action({'request_id':'old'},'SUCCEEDED',9),action(request,'SUCCEEDED',3)]})
    assert tracker.busy


def test_timeout_requires_correlated_stop_confirmation_and_never_retries():
    tracker=ActionTracker();request=tracker.begin('goal');tracker.failed_transport('goal','timeout')
    assert tracker.busy
    stop=tracker.begin('stop');assert stop['target_request_id']==request['request_id']
    receipt=action(stop,'DISPATCHED',8)
    tracker.acknowledge({'action':receipt})
    assert tracker.busy
    tracker.observe({'bridge_id':'b1','recent':[receipt]})
    assert not tracker.busy
    assert '停稳' in tracker.summary()


def test_bridge_restart_cannot_inherit_old_completion():
    tracker=ActionTracker();request=tracker.begin('goal');tracker.acknowledge({'action':action(request)})
    tracker.observe({'bridge_id':'new','recent':[dict(action(request,'SUCCEEDED',9),bridge_id='new')]})
    assert tracker.busy and '重启' in tracker.summary()


def test_rejected_result_is_not_erased_by_transport_error_or_late_ack():
    tracker=ActionTracker();request=tracker.begin('goal');receipt=action(request,'REJECTED',5)
    tracker.acknowledge({'action':receipt});tracker.failed_transport('goal','HTTP400')
    tracker.observe({'bridge_id':'b1','recent':[receipt]})
    tracker.acknowledge({'action':action(request,seq=4)})
    assert not tracker.busy and tracker.motion['state']=='REJECTED'
