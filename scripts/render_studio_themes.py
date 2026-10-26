#!/usr/bin/env python3
"""Render the actual Qt demo in all themes. Never starts ROS or Workbench."""
import argparse
import os
from pathlib import Path
import sys

os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
REPO=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(REPO/'agt_robot_studio'))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',default=str(REPO/'docs/ui_styles'))
    args=parser.parse_args()
    from PyQt5.QtWidgets import QApplication
    from agt_robot_studio.ui import Studio,DEFAULT_CONFIG
    from agt_robot_studio.operator_style import THEMES
    output=Path(args.output);output.mkdir(parents=True,exist_ok=True)
    app=QApplication([]);app.setStyle('Fusion')
    window=Studio(DEFAULT_CONFIG,demo=True)
    window.timer.stop();window.display_timer.stop();window.setMinimumSize(1,1);window.resize(1600,1040)
    window.show();window.load_demo_scene();window.nav_view.setFocus()
    for theme in THEMES:
        window.set_theme(theme);app.processEvents()
        if not window.grab().save(str(output/(theme+'.png'))):raise RuntimeError('screenshot write failed')
    window.set_theme('field');window.operator_tabs.setCurrentIndex(0);app.processEvents()
    window.grab().save(str(output/'camera.png'))
    window.show_diagnostics();app.processEvents();window.diagnostics.grab().save(str(output/'diagnostics.png'))
    window.close();print('Rendered offline Qt demo:',output)


if __name__=='__main__':main()
