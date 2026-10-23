"""AGT Robot Studio: clean Qt5 desk app, separate from ROS motion safety."""
import argparse
import math
import os
import sys
from datetime import datetime
from pathlib import Path
import numpy as np
from PyQt5.QtCore import Qt, QPoint, QProcess, QTimer, QRectF
from PyQt5.QtGui import QColor, QFont, QImage, QPainter, QPen
from PyQt5.QtWidgets import (QApplication, QWidget, QMainWindow, QVBoxLayout, QHBoxLayout,
    QFrame, QLabel, QPushButton, QStackedWidget, QDoubleSpinBox, QComboBox, QCheckBox,
    QMessageBox, QFileDialog, QTextEdit, QFormLayout, QSizePolicy, QSplitter)
from .map_io import occupancy_to_gray, save_nav2_map

DEFAULT_CONFIG = {
    'robot':'bunker', 'points_topic':'/glim_rosnode/map',
    'live_points_topic':'/glim_rosnode/aligned_points', 'map_topic':'/map',
    'odom_topic':'/odom', 'raw_lidar_topic':'/livox/lidar', 'imu_topic':'/livox/imu',
    'drive_action':'/drive_on_heading', 'spin_action':'/spin',
    'live_motion_enabled':False,'drive_speed_mps':.08,'allow_reverse':False,
    'safety_permitted_topic':'/agt/safety/motion_permitted',
    'max_linear_mps':0.10, 'max_angular_rps':0.25, 'max_acceleration_mps2':0.15,
    'guard_max_age_s':1.5,
    'motion_timeout_s':20.0, 'bag_root':'~/agt_studio_runs', 'glim_config_path':''}

STYLE = '''
* {font-family: "Noto Sans CJK SC", "Microsoft YaHei", "DejaVu Sans"; font-size: 13px;}
QMainWindow,QWidget#root{background:#0c1117;color:#e8edf4}
QWidget{color:#e8edf4;background:transparent}
QFrame#sidebar{background:#101820;border-right:1px solid #27323f}
QFrame#card{background:#151f2a;border:1px solid #273440;border-radius:15px}
QFrame#viewer{background:#0d151c;border:1px solid #283643;border-radius:15px}
QLabel#title{font-size:20px;font-weight:700;color:#f5f9fc}
QLabel#h2{font-size:16px;font-weight:650;color:#f5f9fc}
QLabel#muted{color:#90a1ae;font-size:12px}
QLabel#number{font-size:24px;font-weight:700;color:#e8fff8}
QPushButton{background:#24333d;border:1px solid #344552;color:#edf3f6;border-radius:9px;padding:10px 15px;font-weight:500}
QPushButton:hover{background:#2e4e59;border:1px solid #4e827a}
QPushButton:pressed{background:#166a65}
QPushButton:disabled{color:#687780;background:#19232d}
QPushButton#primary{background:#1d776b;color:#fff;border:1px solid #299887}
QPushButton#primary:hover{background:#238b7d}
QPushButton#danger{background:#30232b;border:1px solid #8f4754;color:#ff959c}
QPushButton#nav{border:0;background:transparent;text-align:left;padding:13px 14px;color:#9eb0be}
QPushButton#nav:checked{background:#23323a;color:#fff;border-left:3px solid #53c0ae}
QPushButton#nav:hover{background:#20313b}
QDoubleSpinBox,QComboBox,QTextEdit{background:#111b24;color:#eaf2f5;border:1px solid #364957;border-radius:8px;padding:8px}
QComboBox::drop-down{border:0}
QCheckBox{spacing:10px;color:#ced8dc}
QSplitter::handle{background:#24333e}
'''


def button(text, kind=None):
    b=QPushButton(text)
    if kind:b.setObjectName(kind)
    b.setCursor(Qt.PointingHandCursor)
    return b


def text_label(s, name='muted'):
    x=QLabel(s); x.setObjectName(name);x.setWordWrap(True);return x


def card(parent_layout, title, min_height=0):
    frame=QFrame();frame.setObjectName('card');frame.setMinimumHeight(min_height)
    outer=QVBoxLayout(frame);outer.setContentsMargins(18,15,18,15);outer.setSpacing(12)
    if title:outer.addWidget(text_label(title,'h2'))
    parent_layout.addWidget(frame)
    return outer


class CloudView(QWidget):
    """Interactive 3D orthographic point-cloud projection, no optional OpenGL dependencies."""
    def __init__(self):
        super().__init__()
        self.xyz=np.empty((0,3),dtype=np.float32)
        self.azimuth=-.7;self.elevation=.75;self.scale=1.0
        self.last=None;self.source='未连接'
        self.setMinimumSize(360,245)
        self.setMouseTracking(True)
        self.setToolTip('拖动旋转三维地图 / 滚轮缩放 / 双击复位视角')

    def set_cloud(self, data):
        xyz=np.asarray(data.get('xyz',[]),dtype=np.float32)
        if xyz.ndim!=2 or xyz.shape[1]!=3:return
        self.xyz=xyz
        self.source=data.get('source','点云')
        self.update()

    def mousePressEvent(self,e):
        self.last=e.pos()
    def mouseMoveEvent(self,e):
        if self.last and e.buttons() & Qt.LeftButton:
            d=e.pos()-self.last;self.azimuth+=d.x()*.008
            self.elevation=max(.04,min(1.5,self.elevation+d.y()*.007));self.last=e.pos();self.update()
    def mouseReleaseEvent(self,e):
        self.last=None
    def mouseDoubleClickEvent(self,e):
        self.azimuth=-.7;self.elevation=.75;self.scale=1.0;self.update()
    def wheelEvent(self,e):
        self.scale=max(.15,min(16,self.scale*(1.15 if e.angleDelta().y()>0 else 1/1.15)))
        self.update()

    def paintEvent(self,e):
        p=QPainter(self);p.setRenderHint(QPainter.Antialiasing,False)
        p.fillRect(self.rect(),QColor('#0c151c'))
        w,h=self.width(),self.height()
        p.setPen(QColor('#405462'))
        for i in range(1,12):
            y=int(h*i/12);p.drawLine(0,y,w,y)
        p.setPen(QColor('#768e9a'));p.drawText(15,25,'3D  /  '+self.source)
        if self.xyz.size==0:
            p.setPen(QColor('#7c949e'))
            p.drawText(self.rect(),Qt.AlignCenter,'等待 GLIM PointCloud2\n或启动演示模式')
            return
        pts=self.xyz.astype(np.float64,copy=False)
        c=np.median(pts,axis=0)
        xyz=pts-c
        a,b=self.azimuth,self.elevation
        ca,sa,cb,sb=math.cos(a),math.sin(a),math.cos(b),math.sin(b)
        rx=xyz[:,0]*ca-xyz[:,1]*sa
        ry=xyz[:,0]*sa+xyz[:,1]*ca
        px=rx
        py=ry*sb-xyz[:,2]*cb
        span=max(np.percentile(abs(px),97),np.percentile(abs(py),97),1.)
        fac=min(w,h)*.43/span*self.scale
        sx=w/2+px*fac;sy=h/2-py*fac
        keep=(sx>=2)&(sx<w-2)&(sy>=2)&(sy<h-2)
        z=xyz[:,2]
        # bin colors: floor in teal, elevated structures in pale blue/orange.
        bins=np.digitize(z,[-.4,.12,.8,1.8])
        colors=['#496b77','#36bb9f','#9ddbd3','#e9c48d','#c9e4f0']
        for k,color in enumerate(colors):
            ix=np.flatnonzero(keep&(bins==k))
            p.setPen(QPen(QColor(color),1.7))
            for j in ix:
                p.drawPoint(int(sx[j]),int(sy[j]))
        p.setPen(QColor('#6d8592'))
        p.drawText(15,h-17,f'{len(pts):,} pts  |  鼠标拖动旋转 · 滚轮缩放')


class GridView(QWidget):
    def __init__(self):
        super().__init__()
        self.image=QImage();self.res=.05;self.origin=(0.,0.,0.)
        self.paint_value=None;self.undo=[];self.scale=1.0;self.drag=None
        self.setMinimumSize(350,240)
        self.setToolTip('地图编辑模式：左键绘制；右键或中键拖动；滚轮缩放')

    def set_grid(self,info):
        try:
            gray=occupancy_to_gray(info['data'],int(info['width']),int(info['height']))
            h,w=gray.shape
            img=QImage(gray.data,w,h,w,QImage.Format_Grayscale8).copy()
        except Exception as e:
            return
        self.image=img;self.res=float(info['resolution']);self.origin=tuple(info['origin'])
        self.undo=[];self.scale=1.0;self.update()

    def load_pgm(self,path):
        img=QImage(path).convertToFormat(QImage.Format_Grayscale8)
        if img.isNull():raise ValueError('PGM 读取失败')
        self.image=img;self.undo=[];self.scale=1.0;self.update()

    def _rect(self):
        if self.image.isNull():return QRectF()
        fit=min(self.width()/self.image.width(),self.height()/self.image.height())*.93*self.scale
        w=self.image.width()*fit;h=self.image.height()*fit
        return QRectF((self.width()-w)/2,(self.height()-h)/2,w,h)

    def paintEvent(self,e):
        p=QPainter(self)
        p.fillRect(self.rect(),QColor('#0d151c'))
        if self.image.isNull():
            p.setPen(QColor('#8199a5'))
            p.drawText(self.rect(),Qt.AlignCenter,'等待 /map\n或导入二维 PGM')
        else:
            p.setRenderHint(QPainter.SmoothPixmapTransform,False)
            p.drawImage(self._rect(),self.image)
        p.setPen(QColor('#8096a3'))
        p.drawText(12,22,'2D 栅格  /  左键画笔 · 右键拖动 · 滚轮缩放')

    def wheelEvent(self,e):
        self.scale=max(.25,min(12,self.scale*(1.17 if e.angleDelta().y()>0 else 1/1.17)));self.update()

    def _paint(self,pos):
        if self.paint_value is None or self.image.isNull():return
        r=self._rect()
        if not r.contains(pos):return
        x=int((pos.x()-r.x())/r.width()*self.image.width())
        y=int((pos.y()-r.y())/r.height()*self.image.height())
        radius=3
        painter=QPainter(self.image)
        painter.setPen(Qt.NoPen);painter.setBrush(QColor(self.paint_value,self.paint_value,self.paint_value))
        painter.drawEllipse(x-radius,y-radius,2*radius+1,2*radius+1)
        painter.end();self.update()

    def mousePressEvent(self,e):
        if e.button()==Qt.LeftButton and self.paint_value is not None and not self.image.isNull():
            self.undo.append(self.image.copy());self.undo=self.undo[-12:];self._paint(e.pos())
        elif e.button() in (Qt.RightButton,Qt.MiddleButton):self.drag=e.pos()

    def mouseMoveEvent(self,e):
        if e.buttons()&Qt.LeftButton:self._paint(e.pos())
        if self.drag and e.buttons()&(Qt.RightButton|Qt.MiddleButton):
            # Pan will be a follow-up; retain deterministic transform while editing.
            self.drag=e.pos()

    def mouseReleaseEvent(self,e):self.drag=None

    def undo_last(self):
        if self.undo:self.image=self.undo.pop();self.update()

    def pixels(self):
        if self.image.isNull():raise ValueError('No map loaded')
        image=self.image.convertToFormat(QImage.Format_Grayscale8)
        bits=image.bits();bits.setsize(image.byteCount())
        a=np.frombuffer(bits,dtype=np.uint8).reshape(image.height(),image.bytesPerLine())
        return a[:,:image.width()].copy()


class Studio(QMainWindow):
    def __init__(self,cfg,demo=False):
        super().__init__();self.cfg=dict(cfg);self.demo=demo;self.worker=None
        self.glim=None;self.bag=None;self.rviz=None
        self.setWindowTitle('AGT Robot Studio  ·  ROS 2 Humble')
        self.resize(1320,860);self.setMinimumSize(950,640)
        root=QWidget();root.setObjectName('root');self.setCentralWidget(root)
        lay=QHBoxLayout(root);lay.setContentsMargins(0,0,0,0);lay.setSpacing(0)
        self._sidebar(lay)
        right=QWidget();lay.addWidget(right,1)
        r=QVBoxLayout(right);r.setContentsMargins(23,18,23,18);r.setSpacing(15)
        self._topbar(r)
        self.stack=QStackedWidget();r.addWidget(self.stack,1)
        self.cloud_view=CloudView();self.map_view=GridView()
        self._page_dashboard();self._page_mapping();self._page_maps();self._page_motion();self._page_logs()
        self.pages['实时建图'].click()
        self.setStyleSheet(STYLE)
        self.write_log('界面已启动；' + ('演示模式不连接机器人' if demo else '等待 ROS 连接'))
        self.timer=QTimer(self);self.timer.timeout.connect(self._poll_processes);self.timer.start(800)
        if demo:self._setup_demo()
        else:
            from .ros_worker import RosWorker
            self.worker=RosWorker(self.cfg)
            self.worker.state.connect(self._on_state)
            self.worker.cloud.connect(self._on_cloud)
            self.worker.grid.connect(self._on_grid)
            self.worker.notice.connect(self.write_log)
            self.worker.start()

    def _sidebar(self,layout):
        side=QFrame();side.setObjectName('sidebar');side.setFixedWidth(218);layout.addWidget(side)
        s=QVBoxLayout(side);s.setContentsMargins(15,23,15,18);s.setSpacing(12)
        s.addWidget(text_label('▣  AGT', 'title'))
        s.addWidget(text_label('Robot Studio  /  FIELD EDITION'))
        s.addSpacing(24);s.addWidget(text_label('工 作 区'))
        self.pages={}
        for i,(name,icon) in enumerate([('运行总览','◫'),('实时建图','◎'),('地图编辑','▦'),('运动标定','⌁'),('运行日志','≡')]):
            b=button('  '+icon+'   '+name);b.setObjectName('nav');b.setCheckable(True)
            b.clicked.connect(lambda on,n=i:self._set_page(n))
            s.addWidget(b);self.pages[name]=b
        s.addStretch(1)
        s.addWidget(text_label('ROS 2 HUMBLE   ·   v0.1.1'))
        s.addWidget(text_label('真实运动默认禁止'))

    def _topbar(self,layout):
        row=QHBoxLayout();layout.addLayout(row)
        title=QVBoxLayout();title.addWidget(text_label('AGT Robot Studio','title'))
        title.addWidget(text_label('连接设备 → 建图 → 地图编辑 → 运动标定 → 验收'))
        row.addLayout(title);row.addStretch()
        self.robot_box=QComboBox();self.robot_box.addItems(['Bunker','FW-mini'])
        self.robot_box.setCurrentText('FW-mini' if self.cfg['robot']=='fwmini' else 'Bunker')
        self.robot_box.setToolTip('仅选择显示和记录的车辆，实际驱动配置需独立检查')
        row.addWidget(self.robot_box)
        self.status=text_label('● 未连接')
        self.status.setStyleSheet('color:#e5aa74;padding-left:12px')
        row.addWidget(self.status)

    def _page_shell(self):
        page=QWidget();self.stack.addWidget(page)
        l=QVBoxLayout(page);l.setContentsMargins(0,0,0,0);l.setSpacing(14)
        return l

    def _page_dashboard(self):
        l=self._page_shell()
        l.addWidget(text_label('系统总览','title'))
        c=card(l,'连接与运行状态')
        self.dash_message=text_label('等待 ROS 反馈与传感器数据')
        c.addWidget(self.dash_message)
        for name,description in [('GLIM','/glim_rosnode/map  /  /glim_rosnode/aligned_points'),
                                  ('里程计','/odom'),('运动安全','独立 Motion Guard + Nav2 局部碰撞检查'),
                                  ('控制出口','Nav2 DriveOnHeading / Spin Actions')]:
            c.addWidget(text_label(f'{name}    ·    {description}'))
        l.addStretch()

    def _page_mapping(self):
        l=self._page_shell()
        r=QHBoxLayout();l.addLayout(r)
        r.addWidget(text_label('实时三维建图','title'));r.addStretch()
        self.mapping_state=text_label('未启动 / 外部启动可直接订阅')
        r.addWidget(self.mapping_state)
        frame=QFrame();frame.setObjectName('viewer');l.addWidget(frame,1)
        fl=QVBoxLayout(frame);fl.setContentsMargins(10,10,10,10)
        fl.addWidget(self.cloud_view,1)
        row=QHBoxLayout();fl.addLayout(row)
        self.cloud_count=text_label('点云：等待数据');row.addWidget(self.cloud_count)
        row.addStretch();row.addWidget(text_label('拖动旋转  ·  滚轮缩放'))
        actions=QHBoxLayout();l.addLayout(actions)
        self.glim_btn=button('启动 GLIM','primary');self.glim_btn.clicked.connect(self.toggle_glim);actions.addWidget(self.glim_btn)
        self.rviz_btn=button('打开 RViz 3D');self.rviz_btn.clicked.connect(self.open_rviz);actions.addWidget(self.rviz_btn)
        self.bag_btn=button('开始录包');self.bag_btn.clicked.connect(self.toggle_bag);actions.addWidget(self.bag_btn)
        actions.addStretch()
        row=QHBoxLayout();l.addLayout(row)
        for name in ('PointCloud2','IMU / Odom','Raw Rosbag2'):
            cc=QFrame();cc.setObjectName('card');v=QVBoxLayout(cc)
            v.addWidget(text_label(name,'h2'))
            v.addWidget(text_label('实时状态由 ROS 提供'))
            row.addWidget(cc)

    def _page_maps(self):
        l=self._page_shell();l.addWidget(text_label('二维栅格地图编辑','title'))
        l.addWidget(text_label('仅修改 2D OccupancyGrid；三维地图仍由 GLIM Map Editor 清理。另存不会覆盖原始地图。'))
        frame=QFrame();frame.setObjectName('viewer');l.addWidget(frame,1)
        v=QVBoxLayout(frame);v.addWidget(self.map_view)
        row=QHBoxLayout();l.addLayout(row)
        self.map_brush=QComboBox()
        self.map_brush.addItem('浏览（不绘制）',None)
        self.map_brush.addItem('障碍（黑）',0)
        self.map_brush.addItem('空闲（白）',254)
        self.map_brush.addItem('未知（灰）',205)
        self.map_brush.currentIndexChanged.connect(lambda _: setattr(self.map_view,'paint_value',self.map_brush.currentData()))
        row.addWidget(self.map_brush)
        undo=button('撤销');undo.clicked.connect(self.map_view.undo_last);row.addWidget(undo)
        imp=button('导入 PGM');imp.clicked.connect(self.import_pgm);row.addWidget(imp)
        save=button('另存 Nav2 地图','primary');save.clicked.connect(self.export_map);row.addWidget(save)
        row.addStretch()
        self.map_info=text_label('地图：未加载');l.addWidget(self.map_info)

    def _page_motion(self):
        l=self._page_shell();l.addWidget(text_label('运动标定','title'))
        l.addWidget(text_label('Nav2 Behavior Server 执行定距与旋转；复用 rolling costmap + footprint 碰撞检查，Qt 不直接发速度'))
        c=card(l,'定距与转角');form=QHBoxLayout();c.addLayout(form)
        self.distance=QDoubleSpinBox();self.distance.setRange(.02,5.0);self.distance.setDecimals(2)
        self.distance.setSingleStep(.1);self.distance.setValue(.5);self.distance.setSuffix(' m');form.addWidget(self.distance)
        self.angle=QDoubleSpinBox();self.angle.setRange(2,180);self.angle.setValue(90)
        self.angle.setSuffix(' °');form.addWidget(self.angle)
        row=QHBoxLayout();c.addLayout(row)
        for title,kind,sign in [('前进','distance',1),('后退','distance',-1),('左转','rotation',1),('右转','rotation',-1)]:
            b=button(title);b.clicked.connect(lambda _,k=kind,s=sign:self._request_motion(k,s));row.addWidget(b)
        stop=button('停止','danger');stop.clicked.connect(self.stop_motion);row.addWidget(stop)
        guard=card(l,'运动准入（必须有现场人员和独立硬件安全链）')
        self.ack=QCheckBox('我已验证实体急停、遥控接管、区域净空，并全程监督')
        self.arm=QCheckBox('人工使能本次运动测试（关闭立即停车）')
        guard.addWidget(self.ack);guard.addWidget(self.arm)
        self.arm.stateChanged.connect(self._arm_changed)
        self.guard_text=text_label('门禁：等待 Nav2 Behavior Server 与独立运动许可')
        guard.addWidget(self.guard_text)
        info=card(l,'实时反馈')
        self.odom_label=text_label('里程计：未收到');info.addWidget(self.odom_label)
        self.progress_label=text_label('任务：停止');info.addWidget(self.progress_label)
        info.addWidget(text_label('工作台不直接发送 /cmd_vel；速度必须经 Collision Monitor / Motion Guard 和底盘超时停车。'))
        l.addStretch()

    def _page_logs(self):
        l=self._page_shell();l.addWidget(text_label('运行日志与实验记录','title'))
        self.log=QTextEdit();self.log.setReadOnly(True);l.addWidget(self.log,1)
        row=QHBoxLayout();l.addLayout(row)
        s=button('保存日志');s.clicked.connect(self.save_logs);row.addWidget(s)
        row.addStretch()

    def _set_page(self,i):
        self.stack.setCurrentIndex(i)
        for j,b in enumerate(self.pages.values()):b.setChecked(i==j)

    def write_log(self,message):
        if not hasattr(self,'log'):return
        entry=f'[{datetime.now().strftime("%H:%M:%S")}] {message}'
        self.log.append(entry)
        if self.log.document().blockCount()>350:
            cursor=self.log.textCursor();cursor.movePosition(cursor.Start)
            cursor.select(cursor.BlockUnderCursor);cursor.removeSelectedText();cursor.deleteChar()

    def _on_state(self,s):
        online=s.get('connected',False)
        self.status.setText('● ROS 已连接' if online else '● 连接断开')
        self.status.setStyleSheet('color:#55c9a9' if online else 'color:#ec9c83')
        od=s.get('odom')
        self.odom_label.setText('里程计（仅显示） X={:.2f} Y={:.2f} Yaw={:.1f}° | 距上次接收 {:.1f}s'.format(od[0],od[1],math.degrees(od[2]),s.get('odom_age_s') or 0) if od else '里程计：未收到（不影响 Qt 运行）')
        guards=s.get('guards',{})
        names={'nav2_behaviors':'Nav2行为','motion_guard':'外部运动许可','operator':'人工使能','configuration':'实车锁'}
        self.guard_text.setText('    '.join(('✓' if guards.get(k) else '×')+n for k,n in names.items()))
        self.guard_text.setStyleSheet('color:#65bda9' if all(guards.values()) and guards else 'color:#e8b178')
        kind=s.get('moving')
        self.progress_label.setText(f'执行：{kind}  |  进度 {s.get("progress",0):.3f}  /  {s.get("target",0):.3f}' if kind else '任务：停止')
        self.dash_message.setText('ROS 在线；'+self.guard_text.text())

    def _on_cloud(self,d):
        self.cloud_view.set_cloud(d);self.cloud_count.setText(f'{d.get("source")}  ·  {len(d["xyz"]):,} 点（抽样渲染）')
        self.mapping_state.setText('正在接收实时点云')

    def _on_grid(self,info):
        self.map_view.set_grid(info)
        self.map_info.setText(f'{info["width"]} × {info["height"]}  |  {info["resolution"]} m/cell')

    def _arm_changed(self):
        ok=self.ack.isChecked() and self.arm.isChecked()
        if self.arm.isChecked() and not self.ack.isChecked():
            self.arm.blockSignals(True);self.arm.setChecked(False);self.arm.blockSignals(False)
            self.write_log('拒绝使能：需先确认现场急停/接管/区域')
            ok=False
        if self.worker:self.worker.send('arm',value=ok)
        if not ok:self.stop_motion()

    def _request_motion(self,kind,sign):
        if self.demo:
            self.write_log('演示模式：未发送任何运动指令');return
        if not self.ack.isChecked() or not self.arm.isChecked():
            QMessageBox.warning(self,'运动禁止','请完成现场人工确认及使能，并确保实时安全反馈有效。');return
        val=(self.distance.value() if kind=='distance' else math.radians(self.angle.value()))*sign
        label=f'{val:+.2f} m' if kind=='distance' else f'{math.degrees(val):+.1f}°'
        if QMessageBox.question(self,'再次确认运动',f'确定执行 {label}？\n现场须有人掌握物理急停；失去安全许可立即停止。',QMessageBox.Yes|QMessageBox.No,QMessageBox.No)!=QMessageBox.Yes:return
        self.worker.send('move',kind=kind,target=val)

    def stop_motion(self):
        if self.worker:self.worker.send('stop')

    def _proc(self,command,args,name):
        p=QProcess(self);p.setProgram(command);p.setArguments(args)
        p.setProcessChannelMode(QProcess.MergedChannels)
        p.readyReadStandardOutput.connect(lambda: self._proc_output(p,name))
        p.errorOccurred.connect(lambda _: self.write_log(f'{name} 启动错误：{p.errorString()}'))
        p.finished.connect(lambda code,_:self.write_log(f'{name} 已退出，状态码 {code}'))
        p.start()
        if not p.waitForStarted(1600):
            self.write_log(f'无法启动 {name}: {p.errorString()}');return None
        self.write_log(f'{name} 已启动 (PID {p.processId()})')
        return p

    def _proc_output(self,p,name):
        message=bytes(p.readAllStandardOutput()).decode('utf8','replace').strip()
        if message:self.write_log(name+': '+message[-500:])

    def toggle_glim(self):
        if self.demo:self.write_log('演示模式禁止启动 GLIM');return
        if self.glim and self.glim.state()!=QProcess.NotRunning:
            self.glim.terminate();self.glim_btn.setText('启动 GLIM');self.write_log('向 GLIM 请求正常退出并保存 dump');return
        path=os.path.expanduser(str(self.cfg.get('glim_config_path','')))
        if not path or not Path(path).is_dir():
            QMessageBox.warning(self,'需要 GLIM 配置','请先在 config/studio.yaml 设置真实的 glim_config_path，包含已确认的 IMU 内参和话题。');return
        self.glim=self._proc('ros2',['run','glim_ros','glim_rosnode','--ros-args','-p','config_path:='+path],'GLIM')
        if self.glim:self.glim_btn.setText('结束 GLIM 建图')

    def open_rviz(self):
        if self.demo:self.write_log('演示模式未启动 RViz');return
        if self.rviz and self.rviz.state()!=QProcess.NotRunning:
            self.rviz.raise_() if hasattr(self.rviz,'raise_') else None;return
        self.rviz=self._proc('rviz2',[],'RViz')

    def toggle_bag(self):
        if self.demo:self.write_log('演示模式禁止录包');return
        if self.bag and self.bag.state()!=QProcess.NotRunning:
            self.bag.terminate();self.bag_btn.setText('开始录包');return
        root=Path(str(self.cfg['bag_root'])).expanduser();root.mkdir(parents=True,exist_ok=True)
        output=root/('studio_'+datetime.now().strftime('%Y%m%d_%H%M%S'))
        topics=[self.cfg['raw_lidar_topic'],self.cfg['imu_topic'],self.cfg['odom_topic'],self.cfg['safety_permitted_topic'],'/cmd_vel','/tf','/tf_static']
        topics=list(dict.fromkeys(topics))
        self.bag=self._proc('ros2',['bag','record','-o',str(output),*topics],'rosbag2')
        if self.bag:self.bag_btn.setText('停止录包')

    def _poll_processes(self):
        if self.glim and self.glim.state()==QProcess.NotRunning:self.glim_btn.setText('启动 GLIM')
        if self.bag and self.bag.state()==QProcess.NotRunning:self.bag_btn.setText('开始录包')

    def import_pgm(self):
        path,_=QFileDialog.getOpenFileName(self,'导入二维 PGM',str(Path.home()),'PGM (*.pgm);;所有图片 (*.png *.pgm)')
        if not path:return
        try:
            self.map_view.load_pgm(path)
            y=Path(path).with_suffix('.yaml')
            if y.exists():
                try:
                    import yaml
                    doc=yaml.safe_load(y.read_text())
                    self.map_view.res=float(doc.get('resolution',.05))
                    self.map_view.origin=tuple(doc.get('origin',[0,0,0]))
                except Exception as ex:self.write_log('YAML 未能载入，使用默认坐标：'+str(ex))
            self.map_info.setText(f'已导入 {path}；导出前检查分辨率与原点')
        except Exception as ex:QMessageBox.warning(self,'地图读取失败',str(ex))

    def export_map(self):
        directory=QFileDialog.getExistingDirectory(self,'选择新的地图版本父目录',str(Path.home()))
        if not directory:return
        output=Path(directory)/('map_v'+datetime.now().strftime('%Y%m%d_%H%M%S'))
        try:
            path=save_nav2_map(output,self.map_view.pixels(),self.map_view.res,self.map_view.origin)
            self.write_log('已另存二维地图：'+path)
            QMessageBox.information(self,'地图已保存','新地图：\n'+path)
        except Exception as ex:QMessageBox.warning(self,'导出失败',str(ex))

    def save_logs(self):
        path,_=QFileDialog.getSaveFileName(self,'保存日志',str(Path.home()/'agt_studio.log'),'日志 (*.log *.txt)')
        if path:Path(path).write_text(self.log.toPlainText()+'\n',encoding='utf8')

    def _setup_demo(self):
        # Deterministic synthetic orchard; clearly marked DEMO and never sent to ROS.
        x=np.linspace(-14,14,150)
        floor=np.column_stack([np.tile(x,40),np.repeat(np.linspace(-7,7,40),len(x)),np.zeros(len(x))])
        z=np.linspace(0,3.2,65)
        columns=[]
        for xx in np.arange(-12,13,6):
            for yy in (-5.6,5.6):
                columns.append(np.column_stack([np.full_like(z,xx),np.full_like(z,yy),z]))
        self.cloud_view.set_cloud({'xyz':np.vstack([floor,*columns]),'source':'合成点云 DEMO'})
        self.cloud_count.setText('DEMO 合成点云 · 未连接 MID360')
        h,w=150,200
        cells=np.zeros((h,w),dtype=np.int8)
        cells[:8,:]=100;cells[-8:,:]=100;cells[:,20:25]=100;cells[:,172:178]=100
        cells[35:115,88:100]=-1
        self._on_grid({'width':w,'height':h,'data':cells.ravel(), 'resolution':.05,'origin':(-5.,-3.75,0.)})
        self.status.setText('● 演示模式');self.status.setStyleSheet('color:#eac18a')
        self.dash_message.setText('离线演示：UI、地图绘制、3D 点云均为模拟，不会控制底盘')

    def closeEvent(self,event):
        if self.worker:
            self.worker.shutdown()
            self.worker.wait(2000)
        for p in (self.bag,self.glim,self.rviz):
            if p and p.state()!=QProcess.NotRunning:
                p.terminate()
                if not p.waitForFinished(1600):p.kill();p.waitForFinished(700)
        event.accept()


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--config',default='')
    parser.add_argument('--demo',action='store_true',help='Synthetic UI only; no ROS or motion')
    args=parser.parse_args()
    cfg=dict(DEFAULT_CONFIG)
    if args.config:
        import yaml
        with open(os.path.expanduser(args.config),encoding='utf8') as f:
            x=yaml.safe_load(f)
        if not isinstance(x,dict):parser.error('config YAML should be a mapping')
        cfg.update(x)
    app=QApplication(sys.argv[:1]);app.setStyle('Fusion')
    window=Studio(cfg,args.demo);window.show()
    return app.exec_()

if __name__=='__main__':
    sys.exit(main())
