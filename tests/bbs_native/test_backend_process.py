"""Run the real ROS wrapper's subprocess methods without importing ROS.

Only the class's ROS services/parameters are replaced. Processes, process groups,
stdout/stderr pipes, cancellation, and termination use the OS implementation.
"""
import ast
import os
from pathlib import Path
import shlex
import signal
import subprocess
import sys
import threading
import time
from types import SimpleNamespace

import pytest

REPO = Path(__file__).resolve().parents[2]
WRAPPER_REL = Path('navigation/localization/agt_global_relocalization/'
                   'agt_global_relocalization/global_relocalization.py')


def wrapper_source():
    explicit = os.environ.get('BBS_WRAPPER_SOURCE')
    root = Path(explicit) if explicit else REPO.parent/'agt_navigation_v3'
    path = root if root.is_file() else root/WRAPPER_REL
    if not path.is_file():
        if explicit:
            pytest.fail('BBS_WRAPPER_SOURCE does not contain the runtime wrapper')
        pytest.skip('patched wrapper unavailable; set BBS_WRAPPER_SOURCE')
    return path


@pytest.fixture
def runtime():
    source = ast.parse(wrapper_source().read_text())
    production = next(node for node in source.body if isinstance(node, ast.ClassDef)
                      and node.name == 'GlobalRelocalization')
    methods = [node for node in production.body if isinstance(node, ast.FunctionDef)
               and node.name in ('_stop_backend', '_run_backend')]
    assert len(methods) == 2
    isolated = ast.Module(body=[ast.ClassDef(name='Runtime', bases=[], keywords=[],
                                            body=methods, decorator_list=[])], type_ignores=[])
    processes = []

    def popen(*args, **kwargs):
        process = subprocess.Popen(*args, **kwargs)
        processes.append(process)
        return process

    namespace = dict(os=os, signal=signal, time=time, shlex=shlex,
                     subprocess=SimpleNamespace(Popen=popen, PIPE=subprocess.PIPE,
                         TimeoutExpired=subprocess.TimeoutExpired,
                         CompletedProcess=subprocess.CompletedProcess))
    exec(compile(ast.fix_missing_locations(isolated), str(wrapper_source()), 'exec'), namespace)
    node = namespace['Runtime']()
    node._stopping = node._sc_cancelled = False
    node._backend_process = None
    node.backend = 'bbs'
    node.get_parameter = lambda name: SimpleNamespace(value=node.backend)
    node.get_logger = lambda: SimpleNamespace(warning=lambda message: None)
    yield node, processes
    for process in processes:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=1)


def command(code):
    return shlex.join([sys.executable, '-u', '-c', code])


@pytest.mark.parametrize('returncode', [0, 4])
def test_completed_process_retains_both_streams(runtime, returncode):
    node, processes = runtime
    proc = node._run_backend(command(
        "import sys; print('{\"success\":false}'); "
        f"print('candidate trace',file=sys.stderr); sys.exit({returncode})"), 2.)
    assert proc.returncode == returncode
    assert proc.stdout == '{"success":false}\n'
    assert proc.stderr == 'candidate trace\n'
    assert len(processes) == 1 and processes[0].poll() == returncode
    assert node._backend_process is None


def test_timeout_kills_process_and_keeps_partial_evidence(runtime):
    node, processes = runtime
    started = time.monotonic()
    with pytest.raises(TimeoutError, match='backend timeout') as error:
        node._run_backend(command(
            "import sys,time; print('query ready',flush=True); "
            "print('GICP_REFINING',file=sys.stderr,flush=True); time.sleep(5)"), .3)
    assert time.monotonic() - started < 3.
    assert error.value.backend_stdout == 'query ready\n'
    assert error.value.backend_stderr == 'GICP_REFINING\n'
    assert error.value.backend_returncode < 0
    assert len(processes) == 1 and processes[0].poll() is not None
    assert node._backend_process is None


def test_sc_motion_cancellation_keeps_last_stage_and_stops_child(runtime):
    node, processes = runtime
    node.backend = 'sc_kiss'
    timer = threading.Timer(.25, lambda: setattr(node, '_sc_cancelled', True))
    timer.start()
    try:
        with pytest.raises(RuntimeError, match='backend cancelled') as error:
            node._run_backend(command(
                "import sys,time; print('candidate ready',flush=True); "
                "print('KISS_REFINING',file=sys.stderr,flush=True); time.sleep(5)"), 2.)
    finally:
        timer.cancel()
        timer.join(timeout=1.)
    assert error.value.backend_stdout == 'candidate ready\n'
    assert error.value.backend_stderr == 'KISS_REFINING\n'
    assert len(processes) == 1 and processes[0].poll() is not None
    assert node._backend_process is None


def test_stopped_runtime_does_not_spawn_process(runtime):
    node, processes = runtime
    node._stopping = True
    with pytest.raises(RuntimeError, match='localization stopped'):
        node._run_backend(command("raise AssertionError('must not execute')"), 1.)
    assert not processes and node._backend_process is None
