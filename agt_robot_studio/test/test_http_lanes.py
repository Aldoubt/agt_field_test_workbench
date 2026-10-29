"""HTTP lane regressions using localhost fixtures; no ROS or robot is started."""
import json
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.error import URLError

import pytest

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
pytest.importorskip('PyQt5')
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QApplication

from agt_robot_studio.workbench_client import ApiWorker, TelemetryWorker, WorkbenchClient, WorkbenchError


@pytest.fixture(scope='module')
def app():
    return QApplication.instance() or QApplication([])


@pytest.fixture
def server():
    calls = []

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def do_POST(self):
            calls.append(self.path)
            if self.path == '/api/slow':
                time.sleep(.15)
                body, code = {'accepted': True}, 200
            elif self.path in ('/api/unknown', '/api/unallocated'):
                body, code = {'error': 'bridge RPC timed out; outcome unknown',
                              'outcome_unknown': True, 'request_id': 'r'*128}, 400
                if self.path == '/api/unknown':
                    body['action'] = {'bridge_id': 'bridge-1', 'action_id': 'a1', 'request_id': 'r'*128,
                                      'state': 'CANCEL_REQUESTED', 'terminal': False, 'seq': 5}
            else:
                body, code = {'error': 'motion permission expired', 'action': {
                    'bridge_id': 'bridge-' + 'x'*128, 'action_id': 'a1', 'request_id': 'r1',
                    'state': 'REJECTED', 'terminal': True, 'seq': 4, 'reason': 'stale feedback',
                    'payload': {'secret': 'never expose'}, 'unknown': 'not public'}}, 400
            data = json.dumps(body).encode()
            self.send_response(code)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(data)))
            self.end_headers()
            try:
                self.wfile.write(data)
            except (BrokenPipeError, ConnectionResetError):
                pass

    http = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
    http.daemon_threads = True
    thread = threading.Thread(target=lambda: http.serve_forever(poll_interval=.01), daemon=True)
    thread.start()
    yield WorkbenchClient('http://127.0.0.1:' + str(http.server_port)), calls
    http.shutdown()
    http.server_close()
    thread.join(1)


def recorder(worker):
    events, completed = [], []
    worker.request_event.connect(events.append, Qt.DirectConnection)
    worker.completed.connect(lambda *args: completed.append(args), Qt.DirectConnection)
    return events, completed


def test_http_rejection_preserves_safe_action_identity_and_legacy_text(server):
    client, calls = server
    with pytest.raises(RuntimeError, match='motion permission expired') as failure:
        client.post('/api/reject', {'private': 'command'})
    error = failure.value
    assert isinstance(error, WorkbenchError)
    assert (error.kind, error.status_code, error.method, error.path) == ('http', 400, 'POST', '/api/reject')
    assert error.action['bridge_id'] == 'bridge-' + 'x'*128
    assert (error.action['action_id'], error.action['seq'], error.action['terminal']) == ('a1', 4, True)
    assert 'payload' not in error.action and 'unknown' not in error.action
    assert calls == ['/api/reject']


def test_real_http_timeout_does_not_resubmit_post(server):
    client, calls = server
    with pytest.raises(WorkbenchError) as failure:
        client.request('POST', '/api/slow', {'request_id': 'posted-request'}, timeout=.03)
    assert failure.value.kind == 'timeout'
    assert failure.value.outcome_unknown and failure.value.request_id == 'posted-request'
    assert calls == ['/api/slow']  # The server already received it; outcome is unknown.


@pytest.mark.parametrize('path', ['/api/unknown', '/api/unallocated'])
def test_rpc_unknown_outcome_survives_http_worker_without_becoming_terminal(app, server, path):
    client, calls = server
    worker = ApiWorker(client)
    events, completed = recorder(worker)
    done = threading.Event()
    worker.completed.connect(lambda *args: done.set(), Qt.DirectConnection)
    worker.start(); worker.call('goal', 'POST', path, {'request_id': 'r'*128})
    try:
        assert done.wait(1)
        assert completed == [('goal', False, 'bridge RPC timed out; outcome unknown')]
        event = next(item for item in events if item['state'] == 'completed')
        assert event['http_status'] == 400 and event['outcome_unknown'] is True
        assert event['error_request_id'] == 'r'*128
        assert event['request_id'] != event['error_request_id']  # HTTP lane identity is distinct.
        if path == '/api/unknown':
            assert event['error_action']['state'] == 'CANCEL_REQUESTED'
            assert event['error_action']['terminal'] is False
        else:
            assert event['error_action'] is None
        assert calls == [path]
    finally:
        assert worker.shutdown(wait_ms=1000)


def test_error_categories_distinguish_offline_and_invalid_response(monkeypatch):
    from agt_robot_studio import workbench_client as module
    def offline(*args, **kwargs):
        raise URLError(ConnectionRefusedError('offline'))
    monkeypatch.setattr(module, 'urlopen', offline)
    with pytest.raises(WorkbenchError) as failure:
        WorkbenchClient().get('/api/live')
    assert failure.value.kind == 'connection'
    class Response:
        def __enter__(self): return self
        def __exit__(self, *args): pass
        def read(self): return b'not json'
    monkeypatch.setattr(module, 'urlopen', lambda *a, **k: Response())
    with pytest.raises(WorkbenchError) as failure:
        WorkbenchClient().get('/api/live')
    assert failure.value.kind == 'response'


def test_control_fifo_payload_snapshot_and_structured_failure(app):
    calls, done = [], threading.Event()
    class Client:
        def request(self, method, path, payload, **kwargs):
            calls.append((path, payload))
            if path == '/api/first':
                raise WorkbenchError('blocked', kind='http', method=method, path=path,
                                     status_code=400, action={'action_id': 'a1', 'seq': 3})
            return {'ok': True}
    worker = ApiWorker(Client())
    events, completed = recorder(worker)
    worker.completed.connect(lambda name, *args: done.set() if name == 'second' else None, Qt.DirectConnection)
    payload = {'points': [{'x': 1.0}]}
    worker.call('first', 'POST', '/api/first', payload)
    worker.call('second', 'POST', '/api/second', {})
    payload['points'][0]['x'] = 99
    worker.start()
    try:
        assert done.wait(2)
        assert calls == [('/api/first', {'points': [{'x': 1.0}]}), ('/api/second', {})]
        assert completed == [('first', False, 'blocked'), ('second', True, {'ok': True})]
        failed = next(event for event in events if event['name'] == 'first' and event['state'] == 'completed')
        assert failed['http_status'] == 400 and failed['error_action'] == {'action_id': 'a1', 'seq': 3}
        assert failed['queue_wait_sec'] >= 0 and failed['elapsed_sec'] >= 0
        assert all('payload' not in event for event in events)
        assert worker.status_snapshot()['pending'] == 0
    finally:
        assert worker.shutdown(wait_ms=1000)


def test_telemetry_keeps_flowing_while_asset_lane_is_blocked(app):
    asset_entered, release, live_done = threading.Event(), threading.Event(), threading.Event()
    seen = []
    class Client:
        def request(self, method, path, payload, timeout=None):
            seen.append((method, path, timeout))
            if path == '/api/maps/diff':
                asset_entered.set()
                assert release.wait(2)
            return {'ok': True}
    client = Client()
    asset, telemetry = ApiWorker(client), TelemetryWorker(client)
    telemetry.completed.connect(lambda *args: live_done.set(), Qt.DirectConnection)
    asset.start(); telemetry.start()
    try:
        asset.call('asset', 'POST', '/api/maps/diff', {})
        assert asset_entered.wait(1)
        telemetry.call('live', 'GET', '/api/live')
        assert live_done.wait(1)
        assert asset.status_snapshot()['active_request'] == 'asset'
        assert ('GET', '/api/live', 2.) in seen
    finally:
        release.set()
        assert asset.shutdown(wait_ms=1000)
        assert telemetry.shutdown(wait_ms=1000)


def test_telemetry_coalesces_pending_reads_and_rejects_mutations(app):
    calls, done = [], threading.Event()
    class Client:
        def request(self, *args, **kwargs):
            calls.append((args, kwargs)); return {}
    worker = TelemetryWorker(Client(), max_queue=2, timeout=.4)
    events, completed = recorder(worker)
    assert worker.call('live', 'GET', '/api/live', timeout=30)
    assert not worker.call('live', 'GET', '/api/live')
    assert not worker.call('stop', 'POST', '/api/control/stop', {})
    assert not worker.local('callback', lambda: pytest.fail('local executed'))
    assert worker.status_snapshot()['queued'] == 1
    assert worker.status_snapshot()['coalesced'] == 1
    worker.completed.connect(lambda name, *args: done.set() if name == 'live' else None, Qt.DirectConnection)
    worker.start()
    try:
        assert done.wait(1)
        assert len(calls) == 1 and calls[0][1]['timeout'] == .4
        assert [event['state'] for event in events if event['name'] == 'live'] == [
            'queued', 'coalesced', 'running', 'completed']
    finally:
        assert worker.shutdown(wait_ms=1000)


def test_shutdown_cancels_unsent_requests_but_retains_active_result(app):
    entered, release, finished = threading.Event(), threading.Event(), threading.Event()
    calls = []
    class Client:
        def request(self, method, path, payload, **kwargs):
            calls.append(path); entered.set(); assert release.wait(2); return {'accepted': True}
    worker = ApiWorker(Client())
    events, completed = recorder(worker)
    worker.completed.connect(lambda name, *args: finished.set() if name == 'active' else None, Qt.DirectConnection)
    worker.start(); worker.call('active', 'POST', '/api/active', {})
    try:
        assert entered.wait(1)
        worker.call('queued', 'POST', '/api/queued', {})
        assert not worker.shutdown(wait_ms=1)
        assert worker.isRunning() and worker.has_pending({'active'})
        assert not worker.has_pending({'queued'})
        assert not worker.call('late', 'POST', '/api/late', {})
        assert calls == ['/api/active']
        release.set()
        assert finished.wait(1)
        assert ('active', True, {'accepted': True}) in completed
        assert any(event['name'] == 'queued' and event['state'] == 'cancelled' for event in events)
    finally:
        release.set()
        assert worker.shutdown(wait_ms=1000)


def test_queue_limit_and_shutdown_before_start_do_not_execute_commands(app):
    class Client:
        def request(self, *args, **kwargs): pytest.fail('request sent')
    worker = ApiWorker(Client(), max_queue=1)
    events, completed = recorder(worker)
    assert worker.call('first', 'POST', '/api/first', {})
    assert not worker.call('overflow', 'POST', '/api/second', {})
    assert worker.has_pending({'first'})
    assert any(event.get('error_kind') == 'queue_full' for event in events)
    assert worker.shutdown(wait_ms=0)
    assert worker.status_snapshot()['pending'] == 0
    assert worker.status_snapshot()['queued'] == 0


def test_direct_event_handler_can_close_without_worker_lock_deadlock(app):
    entered, release = threading.Event(), threading.Event()
    closed = []
    class Client:
        def request(self, *args, **kwargs):
            entered.set(); assert release.wait(2); return {}
    worker = ApiWorker(Client(), max_queue=1)
    def on_event(event):
        if event['state'] == 'rejected':
            release.set()
            closed.append(worker.shutdown(wait_ms=1000))
    worker.request_event.connect(on_event, Qt.DirectConnection)
    worker.start(); worker.call('active', 'POST', '/api/active', {})
    try:
        assert entered.wait(1)
        worker.call('queued', 'POST', '/api/queued', {})
        assert not worker.call('overflow', 'POST', '/api/overflow', {})
        assert closed == [True]
    finally:
        release.set()
        assert worker.shutdown(wait_ms=1000)
