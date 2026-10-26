"""Execute the production Unix RPC and HTTP routing without ROS dependencies."""
import ast
import io
import json
from pathlib import Path
import queue
import re
import socket
import subprocess
import threading
import time
from types import SimpleNamespace as NS

import pytest

from workbench.action_status import ActionLedger
from workbench.control_error import ControlError

REPO = Path(__file__).resolve().parents[1]


def production_function(path, name, namespace):
    tree = ast.parse((REPO / path).read_text())
    function = next(node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name == name)
    exec(compile(ast.Module(body=[function], type_ignores=[]), path, 'exec'), namespace)
    return namespace[name]


def production_method(path, class_name, name, namespace):
    tree = ast.parse((REPO / path).read_text())
    cls = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == class_name)
    method = next(node for node in cls.body if isinstance(node, ast.FunctionDef) and node.name == name)
    exec(compile(ast.Module(body=[method], type_ignores=[]), path, 'exec'), namespace)
    return namespace[name]


@pytest.mark.parametrize('unknown', [False, True])
def test_unix_rpc_preserves_error_action_and_outcome_unknown(tmp_path, unknown):
    action, _ = ActionLedger().begin('goal', {'request_id': 'req-1'})
    expected = ControlError('rejected' if not unknown else 'outcome unknown', action,
                            outcome_unknown=unknown, request_id='req-1').as_response()
    work = tmp_path / 'work'; work.mkdir()
    server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    server.bind(str(work / 'ros_bridge.sock')); server.listen(1)
    observed = []
    def serve():
        connection, _ = server.accept()
        with connection:
            observed.append(json.loads(connection.recv(65536)))
            connection.sendall((json.dumps(expected) + '\n').encode())
    thread = threading.Thread(target=serve); thread.start()
    rpc = production_function('workbench/session.py', 'rpc',
                              dict(Path=Path, socket=socket, json=json, ControlError=ControlError))
    try:
        with pytest.raises(ControlError) as error:
            rpc(tmp_path, 'goal', {'request_id': 'req-1'})
        assert isinstance(error.value, ValueError)
        assert error.value.as_response() == expected
        assert observed[0]['payload']['request_id'] == 'req-1'
    finally:
        thread.join(timeout=2); server.close()
    assert not thread.is_alive()


def test_http_400_keeps_structured_action_and_legacy_error_string():
    action, _ = ActionLedger().begin('goal', {'request_id': 'req-1'})
    failure = ControlError('outcome unknown', action, outcome_unknown=True, request_id='req-1')
    def rpc(*args): raise failure
    method = production_method('workbench/server.py', 'Handler', 'do_POST',
                               dict(re=re, rpc=rpc, ROOT=Path('/unused'), subprocess=subprocess,
                                    YAMLError=type('YAMLError', (Exception,), {})))
    replies = []
    fake = NS(path='/api/control/goal', headers={'Content-Type': 'application/json'},
              body=lambda: {'request_id': 'req-1'}, response=lambda *args: replies.append(args))
    method(fake)
    assert replies == [(400, failure.as_response())]


@pytest.mark.parametrize('has_receipt', [False, True])
def test_bridge_queue_timeout_is_read_only_unknown_and_never_empty_error(has_receipt):
    ledger = ActionLedger()
    action = ledger.begin('goal', {'request_id': 'req-1'})[0] if has_receipt else None
    before = ledger.snapshot()['sequence']
    def timeout(**kwargs): raise queue.Empty()
    incoming = {'operation': 'goal', 'payload': {'request_id': 'req-1'}}
    requests = []
    namespace = dict(json=json, queue=NS(Queue=lambda size: NS(get=timeout), Empty=queue.Empty),
                     time=time, ControlError=ControlError)
    method = production_method('scripts/ros_bridge.py', 'Handler', 'handle', namespace)
    fake = NS(rfile=io.BytesIO((json.dumps(incoming) + '\n').encode()), wfile=io.BytesIO(),
              server=NS(node=NS(inbox=NS(put=requests.append), actions=ledger)))
    method(fake)
    reply = json.loads(fake.wfile.getvalue())
    assert reply['error'] and reply['outcome_unknown'] and reply['request_id'] == 'req-1'
    assert reply.get('action') == action
    assert ledger.snapshot()['sequence'] == before
    assert len(requests) == 1  # read the existing receipt; never retry dispatch
