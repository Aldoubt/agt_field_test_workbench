"""Read-only operator data-path inspection; contains no transport or control code."""
import json
from dataclasses import asdict
from datetime import datetime, timezone
from pathlib import Path

from PyQt5.QtCore import Qt
from PyQt5.QtGui import QColor
from PyQt5.QtWidgets import (QDialog, QVBoxLayout, QHBoxLayout, QLabel, QTableWidget,
    QTableWidgetItem, QHeaderView, QPushButton, QFileDialog, QMessageBox, QTextEdit)


class DiagnosticsDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setObjectName('navigation_setup')
        self.setWindowTitle('数据链路 · 状态来源与操作回执')
        self.resize(980, 700)
        self.snapshot = {}
        self.colors = dict(good='#386c1c', warn='#92600c', bad='#b6372a', muted='#59654f')
        box = QVBoxLayout(self)
        intro = QLabel('按来源检查：设备 / ROS → Workbench → 界面。点击一行查看话题与原因。')
        intro.setWordWrap(True); box.addWidget(intro)
        self.table = QTableWidget(0, 5)
        self.table.setHorizontalHeaderLabels(['链路', '状态', '频率', '数据年龄', '说明'])
        self.table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.table.setSelectionBehavior(QTableWidget.SelectRows)
        self.table.setSelectionMode(QTableWidget.SingleSelection)
        self.table.verticalHeader().hide()
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeToContents)
        self.table.horizontalHeader().setSectionResizeMode(4, QHeaderView.Stretch)
        self.table.itemSelectionChanged.connect(self._selection_changed)
        box.addWidget(self.table, 3)
        self.detail = QLabel('尚未收到数据。'); self.detail.setWordWrap(True);box.addWidget(self.detail)
        self.channels = QLabel('HTTP 通道：等待'); self.channels.setWordWrap(True);box.addWidget(self.channels)
        box.addWidget(QLabel('最近请求（传输完成不等于机器人任务完成）'))
        self.events = QTextEdit();self.events.setReadOnly(True);self.events.setMaximumHeight(150)
        box.addWidget(self.events, 1)
        row = QHBoxLayout();box.addLayout(row)
        export = QPushButton('导出诊断快照');export.clicked.connect(self.export_snapshot);row.addWidget(export)
        row.addStretch();close = QPushButton('返回操作');close.clicked.connect(self.close);row.addWidget(close)

    def set_snapshot(self, links, *, channels=None, events=None, context=None):
        records = [asdict(link) if not isinstance(link, dict) else dict(link) for link in links]
        self.snapshot = dict(observed_at=datetime.now(timezone.utc).isoformat(),
                             links=records, channels=channels or {}, events=events or [], context=context or {})
        labels = {'READY':'正常','WAITING':'等待数据','STALE':'已过期','ERROR':'错误','OFFLINE':'离线','BLOCKED':'未满足许可'}
        self.table.setRowCount(len(records))
        for row, link in enumerate(records):
            rate, age = link.get('rate_hz'), link.get('age_s')
            values = [link['label'], labels.get(link['state'], link['state']),
                      '--' if rate is None else f'{rate:.1f} Hz', '--' if age is None else f'{age:.2f} s', link.get('detail','')]
            color = self.colors['good' if link['state']=='READY' else 'bad' if link['state'] in ('ERROR','OFFLINE') else 'warn']
            for col, text in enumerate(values):
                item = QTableWidgetItem(text or '');item.setToolTip((link.get('source') or '')+'\n'+(link.get('error') or '')+'\n'+(link.get('detail') or ''))
                if col==1:item.setForeground(QColor(color))
                self.table.setItem(row,col,item)
        summaries = []
        for name, value in (channels or {}).items():
            summaries.append(f"{name}：排队 {value.get('queued',0)} · 当前 {value.get('active_request') or '空闲'}")
        self.channels.setText('HTTP 通道：'+'；'.join(summaries))
        lines=[]
        for event in (events or [])[-12:]:
            lines.append(f"{event.get('channel','')} / {event.get('name','')} · {event.get('state',event.get('phase',''))} · {event.get('error','')}")
        self.events.setPlainText('\n'.join(lines) or '暂无请求。')
        self._selection_changed()

    def _selection_changed(self):
        row=self.table.currentRow();links=self.snapshot.get('links',[])
        if 0<=row<len(links):
            link=links[row];self.detail.setText((link.get('source') or '')+'\n'+(link.get('error') or '')+' '+(link.get('detail') or ''))

    def export_snapshot(self):
        path,_=QFileDialog.getSaveFileName(self,'保存诊断快照',str(Path.home()/'agt-diagnostics.json'),'JSON (*.json)',options=QFileDialog.DontUseNativeDialog)
        if not path:return
        try:Path(path).write_text(json.dumps(self.snapshot,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
        except OSError as exc:QMessageBox.warning(self,'导出失败',str(exc))
