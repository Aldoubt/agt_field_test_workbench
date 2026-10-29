#!/usr/bin/env python3
"""Apply BBS candidate recovery and bounded evidence after the SC runtime patch."""
from pathlib import Path
import subprocess
import sys

repo = Path(__file__).resolve().parents[1]
raise SystemExit(subprocess.call([
    sys.executable, str(repo / 'scripts/apply_control_integration.py'),
    '--manifest', str(repo / 'patches/bbs_recovery_manifest.json'), *sys.argv[1:]]))
