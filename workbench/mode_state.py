"""Local mode ownership/readiness; stale files never gate the split workflow."""
import os
from pathlib import Path
import json
from .runtime import _pid_start_ticks, write_json


def publish_mode_state(root, mode, map_id, state):
    write_json(Path(root)/'work/field_mode_state.json',dict(
        mode=mode,map_id=map_id,state=state,pid=os.getpid(),
        process_start_ticks=_pid_start_ticks(os.getpid())))


def require_mode_ready(root):
    path=Path(root)/'work/field_mode_state.json'
    if not path.is_file():
        return  # Existing independently started runtime.
    try:
        state=json.loads(path.read_text())
    except (OSError,ValueError) as exc:
        raise ValueError('组合启动状态暂不可读；本次未提交任务') from exc
    if not isinstance(state,dict):
        raise ValueError('组合启动状态格式错误；本次未提交任务')
    pid=state.get('pid')
    identity=state.get('process_start_ticks')
    if not isinstance(pid,int) or pid<=0 or identity is None or _pid_start_ticks(pid)!=identity:
        return  # Previous mode exited, or its PID has been reused.
    if state.get('state')!='READY':
        raise ValueError('组合入口尚未就绪（'+str(state.get('state'))+
            '）；本次未提交任务。等待启动终端显示就绪后再运行。')
