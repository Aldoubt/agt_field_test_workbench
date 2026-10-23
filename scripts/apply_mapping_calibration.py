#!/usr/bin/env python3
"""Apply a tiny, pinned integration patch; do not change the upstream branch."""
import hashlib
import os
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[3]
REPO = ROOT / 'src' / 'agt-lio-pgo-mapping'
EXPECTED = '62e80002cf2db12fce8332688dc63f4a94b87f12'
OLD = "    lio_config = share / 'config' / 'fastlio2_mid360.yaml'"
NEW = ("    lio_config = Path(os.environ.get('AGT_FIELD_TEST_FASTLIO_CONFIG', "
       "str(share / 'config' / 'fastlio2_mid360.yaml'))).resolve()\n"
       "    if not lio_config.is_file():\n"
       "        raise RuntimeError(f'FAST-LIO2 calibration not found: {lio_config}')")
OWNER_OLD = "'mapping_sensor_owner': 'yhs_sensor_only' if sensor_only else 'bunker_legacy_hardware',"
OWNER_NEW = ("'mapping_sensor_owner': os.environ.get('AGT_FIELD_TEST_ROBOT', 'yhs_sensor_only') "
             "if sensor_only else 'bunker_legacy_hardware',")


def main():
    head = subprocess.check_output(['git', '-c', 'safe.directory='+str(REPO), '-C', str(REPO), 'rev-parse', 'HEAD'], text=True).strip()
    if head != EXPECTED:
        raise SystemExit(f'mapping HEAD {head} differs from pinned {EXPECTED}')
    for relative in ('session_launch.py', 'live_launch.py'):
        path = REPO / 'bringup/agt_mapping_bringup/agt_mapping_bringup' / relative
        text = path.read_text()
        if NEW not in text:
            if text.count(OLD) != 1 or 'import os\n' in text:
                raise SystemExit(f'{relative}: source changed; patch not applied')
            text = text.replace('import atexit\n', 'import atexit\nimport os\n').replace(OLD, NEW)
        if relative == 'live_launch.py' and OWNER_NEW not in text:
            if text.count(OWNER_OLD) != 1:
                raise SystemExit('live_launch.py owner contract changed; patch not applied')
            text = text.replace(OWNER_OLD, OWNER_NEW)
        path.write_text(text)
        print(relative, hashlib.sha256(path.read_bytes()).hexdigest())


if __name__ == '__main__':
    main()
