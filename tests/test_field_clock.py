"""Verify NTP hold/restore lifecycle without touching host services or hardware."""
import os
from pathlib import Path
import subprocess

import pytest

SCRIPT = Path(__file__).resolve().parents[1]/'scripts/hold_field_clock.sh'


@pytest.mark.parametrize('active,child_status', [(True, 0), (True, 7), (False, 0), (True, 'INT')])
def test_restore_only_service_owned_by_this_session(tmp_path, active, child_status):
    tools = tmp_path/'bin';tools.mkdir()
    mocks = {
        'timedatectl': '#!/bin/bash\necho "${TEST_SYNC:-yes}"\n',
        'systemctl': '''#!/bin/bash
if [[ "$1" == is-active ]]; then
  [[ "$3" == systemd-timesyncd.service && "$TEST_ACTIVE" == yes ]]
else
  echo "$*" >> "$TEST_ACTIONS"
fi
''',
        'sudo': '#!/bin/bash\n[[ "$1" == -v ]] && exit 0\n[[ "$1" == -n ]] && shift\nexec "$@"\n',
    }
    for name, content in mocks.items():
        path = tools/name;path.write_text(content);path.chmod(0o755)
    actions = tmp_path/'actions'
    env = dict(os.environ, PATH=str(tools)+':'+os.environ['PATH'],
               XDG_RUNTIME_DIR=str(tmp_path), TEST_ACTIONS=str(actions),
               TEST_ACTIVE='yes' if active else 'no', TEST_SYNC='yes')
    child_command = 'kill -INT "$PPID"; exit 130' if child_status == 'INT' else f'exit {child_status}'
    result = subprocess.run(['bash', str(SCRIPT), 'bash', '-c', child_command],
                            env=env, capture_output=True, text=True)
    assert result.returncode == (130 if child_status == 'INT' else child_status), result.stderr
    assert (actions.read_text().splitlines() if actions.exists() else []) == (
        ['stop systemd-timesyncd.service', 'start systemd-timesyncd.service'] if active else [])
    # Unsynchronized preflight must neither launch the child nor change services.
    if actions.exists():actions.unlink()
    env['TEST_SYNC'] = 'no'
    child = tmp_path/'child_started'
    result = subprocess.run(['bash', str(SCRIPT), 'touch', str(child)], env=env, capture_output=True)
    assert result.returncode == 2 and not child.exists() and not actions.exists()
