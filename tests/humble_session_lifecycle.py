#!/usr/bin/env python3
"""Actual owned session/bridge launch and stop, sensors disabled, no CAN."""
import json,sys,time
from pathlib import Path
REPO=Path(__file__).resolve().parents[1];sys.path.insert(0,str(REPO))
from workbench.runtime import Jobs,ROOT
from workbench.session import Sessions,rpc
if (ROOT/'work/current_session.json').exists():raise SystemExit('preserve existing field session; test not run')
jobs=Jobs();sessions=Sessions(jobs);result={}
try:
    deadline=time.monotonic()+15
    while True:
        try:sessions.bridge();break
        except ValueError:
            if time.monotonic()>deadline:raise
            time.sleep(.2)
    job=sessions.start(dict(robot='bunker',task='sensors',enable_sensor=False))
    current=sessions.current()
    assert job['id']==current['job_id'] and job['id']!=current['id']
    assert (ROOT/'runs'/job['id']/'run.json').is_file()
    time.sleep(3)
    assert next(j for j in jobs.list() if j['id']==job['id'])['state']=='RUNNING'
    status=rpc(ROOT,'status');assert not status['feedback_fresh'][0]
    sessions.stop();assert not sessions.current()
    result=dict(state='PASS',scope='HUMBLE_SOFTWARE_NO_CAN_NO_SENSOR',job_id=job['id'],session_id=current['id'],checks=['unique job identity','actual bridge RPC','RSP launch','owned process stop','session pointer cleared'],physical_motion='NOT_RUN')
finally:jobs.stop_all()
(REPO/'docs/evidence/field_loop_20261023/session_lifecycle.json').write_text(json.dumps(result,indent=2));print(json.dumps(result))
