"""Reproduce the sudo/setsid failure with fake services and a pseudo-terminal."""
import errno
import os
from pathlib import Path
import pty
import select
import shutil
import signal
import subprocess
import sys
import time

import pytest

from workbench import field_start


def test_clock_reexec_precedes_detached_component_jobs(monkeypatch,tmp_path):
    events=[]
    monkeypatch.delenv('AGT_FIELD_MODE_CLOCK_SCOPE',raising=False)
    monkeypatch.setattr(field_start,'make_mode_plan',lambda args:dict(components=[]))
    class Probe:
        def reject_existing(self,mode):events.append(('check',mode))
        def close(self):events.append(('close',))
    monkeypatch.setattr(field_start,'Readiness',Probe)
    monkeypatch.setattr(field_start,'Jobs',lambda *_:pytest.fail('detached Jobs created before foreground sudo'))
    class Reexec(Exception):pass
    def execute(program,command):
        events.append(('exec',command));raise Reexec
    monkeypatch.setattr(field_start.os,'execvp',execute)
    with pytest.raises(Reexec):field_start.main(['distance','--no-rviz'])
    assert events[:2]==[('check','distance'),('close',)]
    command=events[-1][1]
    assert command==field_start.foreground_clock_command(['distance','--no-rviz'])
    assert command[-2:]==['distance','--no-rviz']
    assert 'AGT_FIELD_MODE_CLOCK_SCOPE=foreground' in command


@pytest.mark.parametrize('interrupt',[False,True])
def test_password_prompt_and_restore_keep_controlling_tty(tmp_path,monkeypatch,interrupt):
    source=field_start.REPO
    repo=tmp_path/'repo';scripts=repo/'scripts';scripts.mkdir(parents=True)
    shutil.copy2(source/'scripts/hold_field_clock.sh',scripts/'hold_field_clock.sh')
    tools=tmp_path/'bin';tools.mkdir()
    (tools/'timedatectl').write_text('#!/bin/sh\necho yes\n')
    (tools/'systemctl').write_text('''#!/bin/sh
if [ "$1" = is-active ]; then
  [ "$3" = systemd-timesyncd.service ]
else
  echo "$*" >> "$TEST_SERVICE_ACTIONS"
fi
''')
    (tools/'sudo').write_text('#!'+sys.executable+'''
import os,sys
try: fd=os.open('/dev/tty',os.O_WRONLY)
except OSError:
    print('MOCK_SUDO_NO_CONTROLLING_TERMINAL',file=sys.stderr);sys.exit(1)
os.write(fd,b'MOCK_SUDO_PASSWORD_PROMPT\\n');os.close(fd)
a=sys.argv[1:]
if a==['-v']:sys.exit(0)
if a[0]=='-n':a=a[1:]
os.execvp(a[0],a)
''')
    for tool in tools.iterdir():tool.chmod(0o755)
    worker=tmp_path/'worker.py'
    worker.write_text('''import os,subprocess,sys,time
assert os.environ['AGT_FIELD_MODE_CLOCK_SCOPE']=='foreground'
code="import os; f=os.open('/dev/tty',os.O_RDONLY)"
r=subprocess.run([sys.executable,'-c',code],start_new_session=True,capture_output=True)
assert r.returncode != 0, 'component unexpectedly retained a controlling terminal'
open(os.environ['TEST_COMPONENT_STARTED'],'w').write('safe detached fake worker only')
if os.environ.get('TEST_INTERRUPT')=='yes':
    try:
        while True:time.sleep(.05)
    except KeyboardInterrupt:
        time.sleep(.15)
        with open(os.environ['TEST_SERVICE_ACTIONS'],'a') as f:f.write('worker_stopped\\n')
        sys.exit(130)
''')
    # Fixture replacement for the ROS launcher: it can only run the fake worker.
    (scripts/'field_start.sh').write_text('#!/bin/sh\nexec '+sys.executable+' "'+str(worker)+'"\n')
    monkeypatch.setattr(field_start,'REPO',repo)
    env=dict(os.environ,PATH=str(tools)+':'+os.environ['PATH'],XDG_RUNTIME_DIR=str(tmp_path),
             TEST_SERVICE_ACTIONS=str(tmp_path/'services'),TEST_COMPONENT_STARTED=str(tmp_path/'started'),
             TEST_INTERRUPT='yes' if interrupt else 'no')
    command=field_start.foreground_clock_command(['distance'])
    failed=subprocess.run(command,env=env,start_new_session=True,capture_output=True,text=True,timeout=5.)
    assert failed.returncode != 0 and 'MOCK_SUDO_NO_CONTROLLING_TERMINAL' in failed.stderr
    assert not (tmp_path/'started').exists() and not (tmp_path/'services').exists()
    pid,master=pty.fork()
    if pid==0:
        os.execvpe(command[0],command,env)
    output=b'';status=None;deadline=time.monotonic()+10.;sent=False
    try:
        while time.monotonic()<deadline:
            if interrupt and not sent and (tmp_path/'started').exists():
                os.killpg(pid,signal.SIGINT);sent=True
            if select.select([master],[],[],.05)[0]:
                try:output+=os.read(master,8192)
                except OSError as exc:
                    if exc.errno!=errno.EIO:raise
            done,value=os.waitpid(pid,os.WNOHANG)
            if done:
                status=value;break
        assert status is not None,'mock foreground clock scope did not exit'
        assert os.waitstatus_to_exitcode(status)==(130 if interrupt else 0),output.decode(errors='replace')
        assert b'MOCK_SUDO_PASSWORD_PROMPT' in output
        assert (tmp_path/'started').is_file()
        assert (tmp_path/'services').read_text().splitlines()==[
            'stop systemd-timesyncd.service',*(['worker_stopped'] if interrupt else []),
            'start systemd-timesyncd.service']
    finally:
        os.close(master)
        if status is None:
            os.killpg(pid,signal.SIGKILL);os.waitpid(pid,0)
