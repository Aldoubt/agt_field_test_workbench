"""Live, independent orthographic views of a shared transformed cloud snapshot."""
import math
import time
from PyQt5.QtCore import Qt, QPointF, QTimer, pyqtSignal
from PyQt5.QtGui import QColor, QPainter, QPen, QPolygonF
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QPushButton, QLabel


class ProjectionView(QWidget):
    follow_changed=pyqtSignal(bool)
    def __init__(self, projection):
        super().__init__()
        self.projection = projection
        self.snapshot = None
        self.pose = None
        self.pose_stale = True
        self.scale = 1.
        self.base_span=1.
        self.center = None
        self.follow = False
        self.drag = None
        self.setMinimumSize(220, 200)
        self.setToolTip('滚轮独立缩放；拖动平移；双击居中机器人')

    def set_cloud(self, snapshot):
        if self.snapshot is None:self.base_span=snapshot['span']
        self.snapshot = snapshot
        if self.center is None:
            c = snapshot['center']
            self.center = [float(c[0]), float(c[1 if self.projection == 'top' else 2])]
        if self.follow:self.center_robot()
        self.update()

    def set_robot(self, pose):
        self.pose_stale = pose is None
        if pose is not None:self.pose = pose
        if self.follow and pose is not None:
            self.center_robot()
        self.update()

    def center_robot(self):
        if self.pose and self.snapshot and self.pose['frame_id'] == self.snapshot['frame_id']:
            self.center = [self.pose['x'], self.pose['y' if self.projection == 'top' else 'z']]
            self.update()

    def factor(self):
        return min(self.width(), self.height()) * .43 / self.base_span * self.scale

    def wheelEvent(self, e):
        self.scale = max(.05, min(100., self.scale * (1.17 if e.angleDelta().y() > 0 else 1/1.17)))
        self.update()

    def mousePressEvent(self, e):
        self.drag = e.pos()

    def mouseMoveEvent(self, e):
        if self.drag and e.buttons():
            d = e.pos()-self.drag
            self.follow = False
            self.follow_changed.emit(False)
            if self.center:
                self.center[0] -= d.x()/self.factor()
                self.center[1] += d.y()/self.factor()
            self.drag = e.pos()
            self.update()

    def mouseReleaseEvent(self, e):
        self.drag = None

    def mouseDoubleClickEvent(self, e):
        self.center_robot()

    def paintEvent(self, event):
        p = QPainter(self)
        try:
            p.fillRect(self.rect(), QColor('#0c151c'))
            if not self.snapshot:
                p.setPen(QColor('#98adb8'))
                p.drawText(self.rect(), Qt.AlignCenter, '等待有效点云 / TF')
                return
            s = self.snapshot
            plane = s['projections'][self.projection]
            fac = self.factor()
            colors = ['#496b77', '#36bb9f', '#9ddbd3', '#e9c48d', '#c9e4f0']
            for k, color in enumerate(colors):
                points = plane[s['bins'] == k]
                screen = (points-self.center)*[fac, -fac] + [self.width()/2, self.height()/2]
                p.setPen(QPen(QColor(color), 2))
                p.drawPoints(QPolygonF([QPointF(float(x), float(y)) for x, y in screen]))
            pose = self.pose
            if pose and pose['frame_id'] == s['frame_id']:
                x = self.width()/2+(pose['x']-self.center[0])*fac
                y = self.height()/2-(pose['y' if self.projection == 'top' else 'z']-self.center[1])*fac
                p.setPen(QPen(QColor('#e5b144' if self.pose_stale else '#ff697e'), 3)); p.drawEllipse(QPointF(x, y), 6, 6)
                if self.projection == 'top':
                    p.drawLine(QPointF(x, y), QPointF(x+22*math.cos(pose['yaw']), y-22*math.sin(pose['yaw'])))
            p.setPen(QColor('#b5c5ce'))
            age = time.monotonic()-s['received']
            p.drawText(10, 22, s['frame_id']+' · '+('数据过期，保留最后有效画面' if age > 3 else s['source']))
        except Exception:
            p.setPen(QColor('#ff959c')); p.drawText(10, 45, '本视图绘制失败，等待下一帧')
        finally:
            p.end()


class TriViewWorkbench(QWidget):
    def __init__(self, grid):
        super().__init__()
        self.grid = grid
        self.top = ProjectionView('top')
        self.side = ProjectionView('side')
        self.frames = []
        self.layout_grid = QGridLayout(self)
        self.layout_grid.setContentsMargins(0, 0, 0, 0)
        self.expanded = None
        for index, (title, view) in enumerate((('二维栅格', grid), ('三维俯视 XY', self.top), ('三维侧视 XZ', self.side))):
            panel = QWidget(); box = QVBoxLayout(panel); row = QHBoxLayout(); box.addLayout(row)
            row.addWidget(QLabel(title))
            center = QPushButton('居中'); center.clicked.connect(lambda _, v=view: v.center_robot()); row.addWidget(center)
            follow = QPushButton('跟随'); follow.setCheckable(True)
            follow.toggled.connect(lambda value, v=view: self.set_follow(v, value)); row.addWidget(follow)
            view.follow_changed.connect(follow.setChecked)
            expand = QPushButton('放大/恢复'); expand.clicked.connect(lambda _, i=index: self.expand(i)); row.addWidget(expand)
            box.addWidget(view, 1); self.frames.append(panel)
        self.restore_layout()
        self.timer = QTimer(self); self.timer.timeout.connect(self.refresh_status); self.timer.start(500)

    def set_follow(self, view, value):
        view.follow = value
        if value: view.center_robot()

    def restore_layout(self):
        for panel in self.frames: self.layout_grid.removeWidget(panel); panel.show()
        self.layout_grid.addWidget(self.frames[0], 0, 0, 2, 1)
        self.layout_grid.addWidget(self.frames[1], 0, 1)
        self.layout_grid.addWidget(self.frames[2], 1, 1)
        self.expanded = None

    def expand(self, index):
        if self.expanded == index:
            self.restore_layout(); return
        for i, panel in enumerate(self.frames):
            self.layout_grid.removeWidget(panel); panel.setVisible(i == index)
        self.layout_grid.addWidget(self.frames[index], 0, 0, 2, 2)
        self.expanded = index

    def set_cloud(self, snapshot):
        for view in (self.top, self.side): view.set_cloud(snapshot)

    def set_robot(self, pose):
        for view in (self.grid, self.top, self.side): view.set_robot(pose)

    def refresh_status(self):
        for view in (self.grid, self.top, self.side): view.update()
