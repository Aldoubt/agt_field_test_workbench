#!/usr/bin/env python3
"""Apply the thin, checksum-guarded experiment after the existing control patch."""
import subprocess
import sys
from pathlib import Path
repo=Path(__file__).resolve().parents[1]
raise SystemExit(subprocess.call([sys.executable,str(repo/'scripts/apply_control_integration.py'),
    '--manifest',str(repo/'patches/sc_kiss_manifest.json'),*sys.argv[1:]]))
