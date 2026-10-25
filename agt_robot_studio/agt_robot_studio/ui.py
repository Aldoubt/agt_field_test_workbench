"""AGT Robot Studio: clean Qt5 desk app, separate from ROS motion safety."""
import argparse
import base64
import json
import math
import os
import re
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
import numpy as np
from PyQt5.QtCore import Qt, QProcess, QTimer, QRectF, QPointF, pyqtSignal, QUrl
from PyQt5.QtGui import QColor, QImage, QPainter, QPen, QPolygonF, QDesktopServices, QPixmap
from PyQt5.QtWidgets import (QApplication, QWidget, QMainWindow, QVBoxLayout, QHBoxLayout, QGridLayout, QLayout, QSizePolicy,
    QFrame, QLabel, QPushButton, QStackedWidget, QDoubleSpinBox, QComboBox,
    QMessageBox, QFileDialog, QTextEdit, QCheckBox, QLineEdit, QSpinBox,
    QFormLayout, QScrollArea)
from .map_io import occupancy_to_gray, save_nav2_map
from .workbench_client import ApiWorker, WorkbenchClient
from .tri_views import TriViewWorkbench

DEFAULT_CONFIG = {
    'robot':'bunker', 'points_topic':'/glim_rosnode/map',
    'live_points_topic':'/glim_rosnode/aligned_points', 'map_topic':'/map',
    'odom_topic':'/agt/odometry/local', 'raw_lidar_topic':'/livox/lidar', 'imu_topic':'/livox/imu',
    'self_filter_status_topic':'/mapping/self_filter/status',
    'safety_permitted_topic':'/agt/safety/motion_permitted',
    'bag_root':'~/agt_studio_runs', 'glim_config_path':'',
    'workbench_api':'http://127.0.0.1:8765', 'workbench_root':'',
    'auto_start_workbench':True, 'calibration_bundle':'', 'mount_profile':'',
    'geometry_file':'', 'livox_config':'', 'safety_report':'', 'session_checks':{},
    'record_template':'full_field_test', 'record_topics':[],
    'global_backend':'bbs', 'sc_database':'', 'lio_backend':'fastlio2', 'mapping_backend':'fastlio',
    'save_mapping_data':True, 'enable_camera_gimbal':True,
    'camera_device_path':'/dev/video0',
    'gimbal_port_name':'/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0',
    'projection':{'resolution':.05,'ground_cell_size':.5,'ground_percentile':.10,
                  'ground_tolerance':.10,'obstacle_min_height':.12,
                  'obstacle_max_height':1.80,'occupied_threshold':2},
    'map_name':'', 'map_frame':'map', 'glim_package':'glim_ros', 'glim_node':'glim_rosnode',
    'glim_editor':'map_editor'}

USER_CONFIG = Path('~/.config/agt_robot_studio/studio.yaml').expanduser()
RECORD_TEMPLATES = ('full_field_test','mapping','localization','calibration',
                    'chassis_motion','motion_diagnostic')
TOPIC_PATTERN = re.compile(r'/[A-Za-z0-9_]+(?:/[A-Za-z0-9_]+)*')

STYLE = '''
* {font-family: "Noto Sans CJK SC", "Microsoft YaHei", "DejaVu Sans"; font-size: 13px;}
QMainWindow,QWidget#root{background:#0c1117;color:#e8edf4}
QWidget{color:#e8edf4;background:transparent}
QFrame#sidebar{background:#101820;border-right:1px solid #27323f}
QFrame#card{background:#151f2a;border:1px solid #273440;border-radius:15px}
QFrame#viewer{background:#0d151c;border:1px solid #283643;border-radius:15px}
QFrame#global_status{background:#111c25;border:1px solid #283643;border-radius:10px}
QLabel#status_item{font-size:12px;font-weight:600;padding:2px 5px}
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
QDoubleSpinBox,QSpinBox,QComboBox,QTextEdit,QLineEdit{background:#111b24;color:#eaf2f5;border:1px solid #364957;border-radius:8px;padding:8px}
QComboBox::drop-down{border:0}
QComboBox QAbstractItemView{background:#15232e;color:#edf3f6;border:1px solid #47606f;
selection-background-color:#206e69;selection-color:#ffffff;outline:0}
QComboBox QAbstractItemView::item{background:#15232e;color:#edf3f6;min-height:26px}
QComboBox QAbstractItemView::item:selected{background:#206e69;color:#ffffff}
QCheckBox{spacing:10px;color:#ced8dc}
QSplitter::handle{background:#24333e}
'''


def fit_to_screen(window):
    """Choose only the initial geometry; subsequent manual resizing is untouched."""
    screen=window.screen() or QApplication.primaryScreen()
    available=screen.availableGeometry()
    dpi_scale=max(.85,min(1.60,screen.logicalDotsPerInch()/96.0))
    min_width=min(950,max(480,int(available.width()*.96)),available.width())
    min_height=min(640,max(360,int(available.height()*.90)),available.height())
    width=max(min_width,min(int(1320*dpi_scale),int(available.width()*.94)))
    height=max(min_height,min(int(860*dpi_scale),int(available.height()*.90)))
    window.resize(min(width,available.width()),min(height,available.height()))
    window.setMinimumSize(min_width,min_height)
    window.move(available.center()-window.rect().center())
    sidebar_width=max(155,min(int(190*dpi_scale),int(available.width()*.20)))
    return dpi_scale,sidebar_width


def scaled_style(scale):
    style=STYLE
    for original,base in (('13px',13),('20px',20),('16px',16),('12px',12),('24px',24)):
        style=style.replace('font-size: '+original,'font-size: '+str(max(10,round(base*scale)))+'px')
    style=style.replace('padding:10px 15px',
        f'padding:{round(10*scale)}px {round(15*scale)}px')
    style=style.replace('padding:13px 14px',
        f'padding:{round(13*scale)}px {round(14*scale)}px')
    return style


def parse_topics(value):
    """Parse absolute ROS topic names from a list or a comma/line separated field."""
    items=value if isinstance(value,list) else re.split(r'[\s,]+',str(value or '').strip())
    if not isinstance(items,list) or len(items)>128:raise ValueError('录包话题最多 128 个')
    result=[]
    for item in items:
        topic=str(item).strip()
        if not topic:continue
        if len(topic)>255 or not TOPIC_PATTERN.fullmatch(topic):
            raise ValueError('无效 ROS 绝对话题名：'+topic)
        if topic not in result:result.append(topic)
    return result


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
            p.drawText(self.rect(),Qt.AlignCenter,'等待建图 PointCloud2\n或启动演示模式')
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


def pixel_to_world(column, row, width, height, resolution, origin):
    """Convert a top-left PGM pixel to a yaw-aware map-frame position."""
    if not 0 <= column < width or not 0 <= row < height:
        raise ValueError('pixel outside map')
    x = (column + .5) * resolution
    y = (height - row - .5) * resolution
    yaw = float(origin[2])
    return (float(origin[0]) + math.cos(yaw)*x - math.sin(yaw)*y,
            float(origin[1]) + math.sin(yaw)*x + math.cos(yaw)*y)


class GridView(QWidget):
    goal_clicked = pyqtSignal(object)
    follow_changed = pyqtSignal(bool)

    def __init__(self, navigation_mode=False):
        super().__init__()
        self.image=QImage();self.res=.05;self.origin=(0.,0.,0.)
        self.paint_value=None;self.undo=[];self.scale=1.0;self.drag=None
        self.pan_x=0.;self.pan_y=0.;self.navigation_mode=navigation_mode
        self.keepout_mode=False;self.keepout=[];self.polygon=[]
        self.waypoints=[]
        self.pending_goal=None;self.hover_goal=None
        self.robot=None;self.robot_stale=True;self.follow=False;self.frame_id='map';self.received=None;self.plan=[]
        self.setFocusPolicy(Qt.StrongFocus)
        self.setMinimumSize(350,240)
        self.setToolTip('左键第一次选目标位置，第二次选航向；Esc 取消' if navigation_mode else '左键绘制；右键或中键拖动；滚轮缩放')

    def set_grid(self,info):
        try:
            gray=info['gray'] if 'gray' in info else occupancy_to_gray(info['data'],int(info['width']),int(info['height']))
            h,w=gray.shape
            img=QImage(gray.data,w,h,w,QImage.Format_Grayscale8).copy()
        except Exception as e:
            return
        self.image=img;self.res=float(info['resolution']);self.origin=tuple(info['origin'])
        self.frame_id=info.get('frame_id','map');self.received=info.get('received',time.monotonic())
        if self.follow:self.center_robot()
        self.update()

    def set_robot(self,pose):
        self.robot_stale=pose is None
        if pose is not None:self.robot=pose
        if self.follow and pose is not None:self.center_robot()
        self.update()

    def world_screen(self,x,y):
        rect=self._rect();dx=x-self.origin[0];dy=y-self.origin[1]
        c=math.cos(self.origin[2]);s=math.sin(self.origin[2])
        return QPointF(rect.x()+(c*dx+s*dy)/self.res/self.image.width()*rect.width(),
                       rect.y()+(self.image.height()-(-s*dx+c*dy)/self.res)/self.image.height()*rect.height())

    def center_robot(self):
        if self.robot and self.robot['frame_id']==self.frame_id and not self.image.isNull():
            point=self.world_screen(self.robot['x'],self.robot['y'])
            self.pan_x+=self.width()/2-point.x();self.pan_y+=self.height()/2-point.y();self.update()

    def load_pgm(self,path):
        img=QImage(path).convertToFormat(QImage.Format_Grayscale8)
        if img.isNull():raise ValueError('PGM 读取失败')
        self.image=img;self.undo=[];self.keepout=[];self.polygon=[];self.scale=1.0;self.pan_x=0.;self.pan_y=0.;self.reset_heading();self.update()

    def _rect(self):
        if self.image.isNull():return QRectF()
        fit=min(self.width()/self.image.width(),self.height()/self.image.height())*.93*self.scale
        w=self.image.width()*fit;h=self.image.height()*fit
        return QRectF((self.width()-w)/2+self.pan_x,(self.height()-h)/2+self.pan_y,w,h)

    def paintEvent(self,e):
        p=QPainter(self)
        p.fillRect(self.rect(),QColor('#0d151c'))
        if self.image.isNull():
            p.setPen(QColor('#8199a5'))
            p.drawText(self.rect(),Qt.AlignCenter,'选择地图版本\n或从三维 PCD 生成')
        else:
            p.setRenderHint(QPainter.SmoothPixmapTransform,False)
            p.drawImage(self._rect(),self.image)
            rect=self._rect()
            def scene(point):
                return QPointF(rect.x()+(point[0]+.5)/self.image.width()*rect.width(),
                               rect.y()+(point[1]+.5)/self.image.height()*rect.height())
            p.setPen(QPen(QColor('#ef6572'),2));p.setBrush(QColor(239,101,114,38))
            for polygon in self.keepout:
                if len(polygon)>=3:p.drawPolygon(QPolygonF([scene(point) for point in polygon]))
            if self.polygon:
                points=QPolygonF([scene(point) for point in self.polygon])
                p.setBrush(Qt.NoBrush);p.setPen(QPen(QColor('#f0bb68'),2));p.drawPolyline(points)
                for point in points:p.drawEllipse(point,4,4)
        if self.navigation_mode and not self.image.isNull():
            rect=self._rect()
            p.setPen(QPen(QColor('#59a8ed'),2))
            if len(self.plan)>1:p.drawPolyline(QPolygonF([self.world_screen(v['x'],v['y']) for v in self.plan]))
            if len(self.waypoints)>1:
                p.setPen(QPen(QColor('#55c9b0'),2,Qt.DashLine))
                p.drawPolyline(QPolygonF([self.world_screen(v['x'],v['y']) for v in self.waypoints]))
            for index,point in enumerate(self.waypoints):
                local_x=(point['x']-self.origin[0])*math.cos(self.origin[2])+(point['y']-self.origin[1])*math.sin(self.origin[2])
                local_y=-(point['x']-self.origin[0])*math.sin(self.origin[2])+(point['y']-self.origin[1])*math.cos(self.origin[2])
                sx=rect.x()+(local_x/self.res-.5)/self.image.width()*rect.width()
                sy=rect.y()+(self.image.height()-local_y/self.res-.5)/self.image.height()*rect.height()
                p.setPen(QPen(QColor('#f2b66d') if point.get('photo') else QColor('#55c9b0'),2))
                p.drawEllipse(int(sx-6),int(sy-6),12,12)
                p.drawText(int(sx+9),int(sy-8),str(index+1))
            if self.robot and self.robot['frame_id']==self.frame_id:
                point=self.world_screen(self.robot['x'],self.robot['y'])
                yaw=self.robot['yaw']-self.origin[2]
                p.setPen(QPen(QColor('#e5b144' if self.robot_stale else '#ff697e'),3));p.drawEllipse(point,6,6)
                p.drawLine(point,QPointF(point.x()+24*math.cos(yaw),point.y()-24*math.sin(yaw)))
        if self.navigation_mode and self.pending_goal is not None and not self.image.isNull():
            rect=self._rect()
            def point_screen(world):
                dx=world[0]-self.origin[0];dy=world[1]-self.origin[1]
                c=math.cos(self.origin[2]);sn=math.sin(self.origin[2])
                local_x=c*dx+sn*dy
                local_y=-sn*dx+c*dy
                return QPointF(rect.x()+(local_x/self.res-.5)/self.image.width()*rect.width(),
                               rect.y()+(self.image.height()-local_y/self.res-.5)/self.image.height()*rect.height())
            begin=point_screen(self.pending_goal)
            p.setPen(QPen(QColor('#ffc36a'),2));p.setBrush(QColor('#ffc36a'))
            p.drawEllipse(begin,5,5)
            if self.hover_goal is not None:
                end=point_screen(self.hover_goal)
                p.drawLine(begin,end)
            p.setPen(QColor('#ffc36a'))
            p.drawText(12,42,'等待第二次左键选择朝向 · Esc 取消')
        p.setPen(QColor('#8096a3'))
        p.drawText(12,22,'2D 栅格 / 左键选位置→左键选方向 / Esc 取消' if self.navigation_mode else '2D 栅格  /  左键画笔 · 右键拖动 · 滚轮缩放')
        if self.navigation_mode:
            p.drawText(12,self.height()-12, '栅格缺失' if self.image.isNull() else
                '栅格过期，保留最后有效画面' if self.received and time.monotonic()-self.received>5 else '最新有效栅格')

    def reset_heading(self):
        self.pending_goal=None;self.hover_goal=None;self.update()

    def select_heading_point(self,x,y):
        """Two map-frame left clicks: position then orientation, no motion on first."""
        if not self.navigation_mode or self.frame_id!='map':return
        if not all(math.isfinite(v) for v in (x,y)):return
        if self.pending_goal is None:
            self.pending_goal=(float(x),float(y))
            self.hover_goal=None;self.setFocus();self.update();return
        x0,y0=self.pending_goal
        if math.hypot(x-x0,y-y0)<max(self.res*1.5,0.10):
            # Too close to reliably select heading; keep initial point.
            return
        yaw=math.atan2(y-y0,x-x0)
        self.reset_heading()
        self.goal_clicked.emit({'x':x0,'y':y0,'yaw':yaw})

    def keyPressEvent(self,e):
        if self.navigation_mode and e.key()==Qt.Key_Escape:
            self.reset_heading();e.accept();return
        super().keyPressEvent(e)

    def set_waypoints(self,points):
        self.waypoints=list(points);self.update()

    def wheelEvent(self,e):
        self.scale=max(.25,min(12,self.scale*(1.17 if e.angleDelta().y()>0 else 1/1.17)));self.update()

    def _image_point(self,pos):
        rect=self._rect()
        if self.image.isNull() or not rect.contains(pos):return None
        return (min(self.image.width()-1,max(0,int((pos.x()-rect.x())/rect.width()*self.image.width()))),
                min(self.image.height()-1,max(0,int((pos.y()-rect.y())/rect.height()*self.image.height()))))

    def _snapshot(self):
        self.undo.append((self.image.copy(),[[list(point) for point in polygon] for polygon in self.keepout]))
        self.undo=self.undo[-12:]

    def _paint(self,pos):
        if self.paint_value is None or self.keepout_mode:return
        point=self._image_point(pos)
        if point is None:return
        x,y=point;radius=3
        painter=QPainter(self.image)
        painter.setPen(Qt.NoPen);painter.setBrush(QColor(self.paint_value,self.paint_value,self.paint_value))
        painter.drawEllipse(x-radius,y-radius,2*radius+1,2*radius+1)
        painter.end();self.update()

    def finish_keepout(self):
        if not self.keepout_mode or self.image.isNull() or len(self.polygon)<3:return False
        self._snapshot()
        polygon=[[int(point[0]),int(point[1])] for point in self.polygon]
        painter=QPainter(self.image);painter.setPen(Qt.NoPen);painter.setBrush(QColor(0,0,0))
        painter.drawPolygon(QPolygonF([QPointF(point[0],point[1]) for point in polygon]));painter.end()
        self.keepout.append(polygon);self.polygon=[];self.update();return True

    def mousePressEvent(self,e):
        if e.button()==Qt.LeftButton and self.navigation_mode and not self.image.isNull():
            point=self._image_point(e.pos())
            if point is not None:
                x,y=pixel_to_world(*point,self.image.width(),self.image.height(),self.res,self.origin)
                self.select_heading_point(x,y)
        elif e.button()==Qt.LeftButton and self.keepout_mode and not self.image.isNull():
            point=self._image_point(e.pos())
            if point is not None:self.polygon.append(point);self.update()
        elif e.button()==Qt.LeftButton and self.paint_value is not None and not self.image.isNull():
            self._snapshot();self._paint(e.pos())
        elif e.button() in (Qt.RightButton,Qt.MiddleButton):self.drag=e.pos()

    def mouseDoubleClickEvent(self,e):
        if e.button()==Qt.LeftButton and self.navigation_mode:self.mousePressEvent(e)
        elif e.button()==Qt.LeftButton and self.keepout_mode:self.finish_keepout()

    def mouseMoveEvent(self,e):
        if self.navigation_mode and self.pending_goal is not None:
            point=self._image_point(e.pos())
            if point is not None:
                self.hover_goal=pixel_to_world(*point,self.image.width(),self.image.height(),self.res,self.origin)
                self.update()
        if e.buttons()&Qt.LeftButton and not self.navigation_mode:self._paint(e.pos())
        if self.drag and e.buttons()&(Qt.RightButton|Qt.MiddleButton):
            self.follow=False
            self.follow_changed.emit(False)
            delta=e.pos()-self.drag;self.pan_x+=delta.x();self.pan_y+=delta.y()
            self.drag=e.pos();self.update()

    def mouseReleaseEvent(self,e):self.drag=None

    def undo_last(self):
        if self.undo:
            self.image,self.keepout=self.undo.pop();self.polygon=[];self.update()

    def set_keepout(self,polygons):
        self.keepout=[[list(point) for point in polygon] for polygon in (polygons or [])]
        self.polygon=[];self.undo=[];self.update()

    def pixels(self):
        if self.image.isNull():raise ValueError('No map loaded')
        image=self.image.convertToFormat(QImage.Format_Grayscale8)
        bits=image.bits();bits.setsize(image.byteCount())
        a=np.frombuffer(bits,dtype=np.uint8).reshape(image.height(),image.bytesPerLine())
        return a[:,:image.width()].copy()


class Studio(QMainWindow):
    def __init__(self,cfg,demo=False):
        super().__init__();self.cfg=dict(cfg);self.demo=demo;self.worker=None;self.api=None;self.control_api=None
        self.glim=None;self.bag=None;self.rviz=None;self.editor=None;self.backend=None
        self.current_run=None;self.current_map_id=None;self.maps_by_id={};self.clouds_by_id={}
        self.current_map_name=str(cfg.get('map_name') or '').strip()[:120]
        self.mapping_save_data=bool(cfg.get('save_mapping_data',True))
        self.live_status={};self.live_in_flight=False;self.pending_session=None
        self.ros_display_connected=False;self.topic_rates={}
        self.recorder_id=None;self.recorder_output='';self.jobs_in_flight=False;self.mapping_job_id=None
        self.mapping_saving=False;self.map_saving=False;self.grid_building=False;self.pending_load_map_id=None
        self.camera_job_state={'state':'STOPPED'};self.camera_health_state={};self.jobs_cache=[];self.last_capture_job_id=None
        self.camera_health_seen=0.;self.camera_health_pending=False
        self.components_in_flight=False
        self.pending_mapping_start=False
        self.routes_by_id={};self.route_points=[];self.current_route_id=None
        self.setWindowTitle('AGT Robot Studio  ·  ROS 2 Humble')
        self.ui_scale,self.sidebar_width=fit_to_screen(self)
        root=QWidget();root.setObjectName('root');self.setCentralWidget(root)
        lay=QHBoxLayout(root);lay.setContentsMargins(0,0,0,0);lay.setSpacing(0)
        self._sidebar(lay)
        right=QWidget();lay.addWidget(right,1)
        r=QVBoxLayout(right);r.setContentsMargins(23,18,23,18);r.setSpacing(15)
        self._topbar(r)
        self._global_status(r)
        self.stack=QStackedWidget();r.addWidget(self.stack,1)
        self.cloud_view=CloudView();self.clean_view=CloudView();self.map_view=GridView();self.nav_view=GridView(navigation_mode=True)
        self.tri_view=TriViewWorkbench(self.nav_view)
        self.nav_view.goal_clicked.connect(self.send_goal)
        self._page_mapping();self._page_devices();self._page_maps();self._page_navigation();self._page_parameters();self._page_logs()
        self.pages['设备与传感器'].click();self.setStyleSheet(scaled_style(self.ui_scale))
        self.write_log('界面已启动；' + ('演示模式不连接机器人' if demo else '正在连接现有 Workbench 与 ROS'))
        self.timer=QTimer(self);self.timer.timeout.connect(self._poll);self.timer.start(1500)
        self.display_timer=QTimer(self);self.display_timer.timeout.connect(self._drain_display);self.display_timer.start(100)
        if demo:self._setup_demo()
        else:
            self._setup_backend()
            self._ensure_ros_worker()

    def _ensure_ros_worker(self):
        if self.demo:return
        if self.worker:
            if self.worker.isRunning():return
            self.worker.wait(2000);self.worker=None
        from .ros_worker import RosWorker
        self.worker=RosWorker(self.cfg)
        self.worker.state.connect(self._on_state);self.worker.notice.connect(self.write_log)
        self.worker.start()

    def _drain_display(self):
        if not self.worker:return
        for kind,value in self.worker.frames.take().items():
            try:
                if kind=='cloud':self._on_cloud(value)
                elif kind=='grid':self._on_grid(value)
            except Exception as exc:self.write_log('显示帧已隔离：'+str(exc))

    def _sidebar(self,layout):
        side=QFrame();side.setObjectName('sidebar');side.setFixedWidth(self.sidebar_width);layout.addWidget(side)
        s=QVBoxLayout(side);s.setContentsMargins(15,23,15,18);s.setSpacing(12)
        s.addWidget(text_label('▣  AGT','title'));s.addWidget(text_label('Robot Studio  /  FIELD EDITION'))
        s.addSpacing(24);s.addWidget(text_label('工 作 区'));self.pages={}
        for i,(name,icon) in enumerate([('建图','◎'),('设备与传感器','◉'),('地图','▦'),('定位与导航','⌁'),('参数','⚙'),('日志','≡')]):
            b=button('  '+icon+'   '+name);b.setObjectName('nav');b.setCheckable(True)
            b.clicked.connect(lambda _,n=i:self._set_page(n));s.addWidget(b);self.pages[name]=b
        s.addStretch(1);s.addWidget(text_label('ROS 2 HUMBLE   ·   v0.2.0'))
        s.addWidget(text_label('所有操作均在界面内完成'))

    def _topbar(self,layout):
        row=QHBoxLayout();layout.addLayout(row)
        title=QVBoxLayout();title.addWidget(text_label('AGT Robot Studio','title'))
        title.addWidget(text_label('建图 → 生成与编辑地图 → 全局重定位 → 点击导航'))
        row.addLayout(title);row.addStretch()
        self.status=text_label('● 未连接')
        self.status.setStyleSheet('color:#e5aa74;padding-left:12px');row.addWidget(self.status)

    def _global_status(self,layout):
        self.global_status=QFrame();self.global_status.setObjectName('global_status')
        row=QGridLayout(self.global_status);row.setContentsMargins(12,7,12,7)
        row.setHorizontalSpacing(8);row.setVerticalSpacing(5)
        self.robot_state_label=text_label('设备：--','status_item')
        self.mode_state_label=text_label('状态：未连接','status_item')
        self.chassis_state_label=text_label('底盘：未连接','status_item')
        self.lidar_state_label=text_label('雷达：离线 -- Hz','status_item')
        self.imu_state_label=text_label('IMU：离线 -- Hz','status_item')
        self.odom_state_label=text_label('里程计：离线 -- Hz','status_item')
        self.camera_state_label=text_label('云台：未启动','status_item')
        for index,label in enumerate((self.robot_state_label,self.mode_state_label,self.chassis_state_label,
                      self.lidar_state_label,self.imu_state_label,self.odom_state_label,self.camera_state_label)):
            label.setMinimumWidth(0)
            label.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Preferred)
            row.addWidget(label,index//4,index%4)
        for col in range(4):row.setColumnStretch(col,1)
        layout.addWidget(self.global_status)

    def _page_shell(self, scroll=False):
        page=QWidget()
        l=QVBoxLayout(page)
        l.setContentsMargins(0,0,7 if scroll else 0,0)
        l.setSpacing(14)
        if scroll:
            l.setSizeConstraint(QLayout.SetMinAndMaxSize)
            viewport=QScrollArea()
            viewport.setFrameShape(QFrame.NoFrame)
            viewport.setWidgetResizable(True)
            viewport.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
            viewport.setWidget(page)
            self.stack.addWidget(viewport)
        else:
            self.stack.addWidget(page)
        return l

    def _page_devices(self):
        """Independent peripherals and explicit operator motion checklist."""
        l=self._page_shell(scroll=True);l.addWidget(text_label('设备与传感器','title'))
        card(l,'已连接设备')
        sensors=card(l,'一、传感器、底盘与组合启动')
        sensors.addWidget(text_label('传感器、底盘、控制守护、定位和导航由后端独立管理；停止有依赖的组件前需先结束使用者。'))
        row=QHBoxLayout();sensors.addLayout(row)
        start=button('启动 MID360S / IMU');start.clicked.connect(self.connect_sensors);row.addWidget(start)
        chassis=button('连接底盘（受监督）');chassis.clicked.connect(self.connect_chassis);row.addWidget(chassis)
        stop_chassis=button('停止底盘');stop_chassis.clicked.connect(lambda:self._api('component_stop','POST','/api/components/chassis/stop',{}));row.addWidget(stop_chassis)
        stop_sensors=button('停止传感器');stop_sensors.clicked.connect(lambda:self._api('component_stop','POST','/api/components/sensors/stop',{}));row.addWidget(stop_sensors)
        row=QHBoxLayout();sensors.addLayout(row)
        for title,path in (('停止导航','navigation/stop'),('停止定位','localization/stop'),
                           ('停止手动守护','guard/stop'),('恢复手动守护','guard/start')):
            control=button(title);control.clicked.connect(lambda _,p=path:self._api('component_stop','POST','/api/components/'+p,{}));row.addWidget(control)
        self.component_status_label=text_label('组件状态：等待后端健康观测');sensors.addWidget(self.component_status_label)
        self.motion_checks={}
        checks=card(l,'二、底盘启动前现场确认')
        checks.addWidget(text_label('默认全部未勾选；不代替实体急停、遥控接管和后端运动许可。'))
        for key,name in (('wiring','线束/供电/CAN 已核对'),
                         ('physical_estop','实体急停已验证'),
                         ('remote_takeover','遥控接管有效'),
                         ('clear_area','测试区域已清空'),
                         ('supervised','现场有人监督')):
            control=QCheckBox(name);checks.addWidget(control);self.motion_checks[key]=control
        camera=card(l,'三、C1 云台相机（不阻碍导航）')
        camera.addWidget(text_label('相机设备及云台串口在「参数」页配置。'
            '相机驱动独立于 Nav2；进程启动后仍需实测图像和云台反馈。'))
        row=QHBoxLayout();camera.addLayout(row)
        self.camera_start_btn=button('启动 C1 云台')
        self.camera_start_btn.clicked.connect(self.start_camera);row.addWidget(self.camera_start_btn)
        self.camera_stop_btn=button('停止 C1 云台')
        self.camera_stop_btn.clicked.connect(self.stop_camera);row.addWidget(self.camera_stop_btn)
        self.camera_status_label=text_label('云台：未启动');camera.addWidget(self.camera_status_label)
        angle_row=QHBoxLayout();camera.addLayout(angle_row)
        self.camera_heading_edit=QDoubleSpinBox();self.camera_heading_edit.setRange(-160,160)
        self.camera_heading_edit.setDecimals(1);self.camera_heading_edit.setSuffix('° 航向');angle_row.addWidget(self.camera_heading_edit)
        self.camera_pitch_edit=QDoubleSpinBox();self.camera_pitch_edit.setRange(-90,90)
        self.camera_pitch_edit.setDecimals(1);self.camera_pitch_edit.setSuffix('° 俯仰');angle_row.addWidget(self.camera_pitch_edit)
        self.camera_move_btn=button('仅移动云台');self.camera_move_btn.clicked.connect(self.move_camera);angle_row.addWidget(self.camera_move_btn)
        self.camera_capture_btn=button('移动并拍照','primary');self.camera_capture_btn.clicked.connect(self.capture_camera);angle_row.addWidget(self.camera_capture_btn)
        health_row=QHBoxLayout();camera.addLayout(health_row)
        health=button('检查相机健康');health.clicked.connect(self.check_camera_health);health_row.addWidget(health)
        open_dir=button('打开采集目录');open_dir.clicked.connect(self.open_capture_directory);health_row.addWidget(open_dir)
        export=button('导出采集与任务记录');export.clicked.connect(self.export_capture_session);health_row.addWidget(export)
        self.capture_status_label=text_label('采集：尚未执行');camera.addWidget(self.capture_status_label)
        records=card(l,'四、独立录包')
        row=QHBoxLayout();records.addLayout(row)
        record=button('开始/停止独立录包')
        record.clicked.connect(self.toggle_recording);row.addWidget(record)
        records.addWidget(text_label('使用参数页的录包模板/自选话题，复用原有唯一 recorder。'
            'FAST-LIO2 建图过程中不可再开第二份原始录包。'))
        stop=card(l,'五、现场恢复')
        stop_all=button('停止全部 Workbench ROS 进程','danger')
        stop_all.clicked.connect(self.close_ros_nodes);stop.addWidget(stop_all)
        stop.addWidget(text_label('此按钮不是实体急停；可选相机故障不要调用此按钮处理。'))

    def start_camera(self):
        if self.demo:
            self.write_log('演示模式：云台相机不启动');return
        self.camera_status_label.setText('云台：请求启动中')
        self._api('camera_start','POST','/api/devices/camera/start',
            {'device_path':self.camera_device_edit.text().strip(),
             'port_name':self.gimbal_port_edit.text().strip()})

    def stop_camera(self):
        if self.demo:
            self.write_log('演示模式：无云台进程');return
        self._api('camera_stop','POST','/api/devices/camera/stop',{})

    def check_camera_health(self):
        if self.demo:self.write_log('演示模式没有真实相机健康状态');return
        self._api('camera_health','GET','/api/devices/camera?probe=1')

    def move_camera(self):
        self._camera_request(save_image=False)

    def capture_camera(self):
        self._camera_request(save_image=True)

    def _camera_request(self,save_image):
        if self.demo:self.write_log('演示模式不执行云台运动或拍照');return
        pose=self.live_status.get('pose')
        metadata={'robot':self._robot(),'task':(self.live_status.get('session') or {}).get('task','manual_capture'),
                  'session_id':(self.live_status.get('session') or {}).get('id'),
                  'map_version':self.current_map_id,
                  'pose':dict(x=pose['x'],y=pose['y'],yaw=pose['yaw']) if pose else None}
        tag='capture_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        payload={'heading':self.camera_heading_edit.value(),'pitch':self.camera_pitch_edit.value(),
                 'roll':0.0,'tag':tag,'metadata':metadata}
        self.capture_status_label.setText(('拍照请求：' if save_image else '云台移动请求：')+tag)
        self._api('camera_capture' if save_image else 'camera_move','POST',
                  '/api/devices/camera/capture' if save_image else '/api/devices/camera/move',payload)

    def open_capture_directory(self):
        active_root=self.camera_job_state.get('capture_root')
        root=str(Path(active_root).parent) if active_root else None
        if not root:
            QMessageBox.information(self,'采集目录','请先启动一次 C1 相机，创建独立采集会话。');return
        path=Path(root)
        QDesktopServices.openUrl(QUrl.fromLocalFile(str(path)))

    def export_capture_session(self):
        if self.demo:self.write_log('演示模式没有可导出的照片');return
        base=str(self.cfg.get('workbench_api','http://127.0.0.1:8765')).rstrip('/')
        QDesktopServices.openUrl(QUrl(base+'/api/devices/captures/export'))

    def _page_mapping(self):
        l=self._page_shell(scroll=True);r=QHBoxLayout();l.addLayout(r)
        r.addWidget(text_label('建图','title'));r.addStretch()
        self.mapping_state=text_label('未启动');r.addWidget(self.mapping_state)
        self.mapping_save_status=text_label('保存状态：尚未开始');l.addWidget(self.mapping_save_status)
        self.self_filter_label=text_label('自体回波过滤：等待 MID360S / TF 状态（默认开启）')
        l.addWidget(self.self_filter_label)
        c=card(l,'1. 选择车辆并连接设备')
        row=QHBoxLayout();c.addLayout(row)
        self.robot_box=QComboBox();self.robot_box.addItems(['Bunker','FW-mini'])
        self.robot_box.setCurrentText('FW-mini' if self.cfg['robot']=='fwmini' else 'Bunker')
        self.robot_box.currentTextChanged.connect(lambda _text:self._update_global_status())
        self.robot_box.setToolTip('选择 Workbench 中已加载的车辆 Profile；切换会话前请先停止当前会话')
        row.addWidget(self.robot_box,1)
        self.sensors_btn=button('连接传感器');self.sensors_btn.clicked.connect(self.connect_sensors);row.addWidget(self.sensors_btn)
        self.chassis_btn=button('连接底盘');self.chassis_btn.clicked.connect(self.connect_chassis);row.addWidget(self.chassis_btn)
        row=QHBoxLayout();c.addLayout(row)
        self.record_btn=button('开始录制');self.record_btn.clicked.connect(self.toggle_recording);row.addWidget(self.record_btn)
        self.stop_ros_btn=button('停止全部 ROS','danger');self.stop_ros_btn.clicked.connect(self.close_ros_nodes);row.addWidget(self.stop_ros_btn)
        self.stop_ros_btn.setToolTip('仅关闭 Studio 与 Workbench 启动的 ROS 进程；不是硬件急停')
        row.addStretch()
        self.sensor_rate_label=text_label('MID360S：-- Hz    IMU：-- Hz    base_link 里程计：-- Hz')
        c.addWidget(self.sensor_rate_label)
        status_row=QHBoxLayout();c.addLayout(status_row)
        self.device_label=text_label('设备会话：未连接');status_row.addWidget(self.device_label)
        self.record_label=text_label('录制：未启动');status_row.addWidget(self.record_label,1)
        c=card(l,'2. 选择建图后端并保存')
        backend_row=QHBoxLayout();c.addLayout(backend_row)
        backend_row.addWidget(text_label('建图后端'))
        self.mapping_backend_box=QComboBox()
        self.mapping_backend_box.addItem('FAST-LIO2 · 无 PGO','fastlio')
        self.mapping_backend_box.addItem('GLIM · 实验','glim_experimental')
        index=self.mapping_backend_box.findData(str(self.cfg.get('mapping_backend','fastlio')))
        self.mapping_backend_box.setCurrentIndex(max(0,index));backend_row.addWidget(self.mapping_backend_box,1)
        name_row=QHBoxLayout();c.addLayout(name_row)
        name_row.addWidget(text_label('地图名称'))
        self.map_name_edit=QLineEdit(self.current_map_name or '温室地图_'+datetime.now().strftime('%Y%m%d_%H%M'))
        self.map_name_edit.setMaxLength(120);self.map_name_edit.setPlaceholderText('例如：一号温室_东侧')
        name_row.addWidget(self.map_name_edit,1)
        c.addWidget(text_label('后续按名称选择；不可变版本 ID 仍由 Workbench 管理'))
        row=QHBoxLayout();c.addLayout(row)
        self.glim_btn=button('开始建图','primary');self.glim_btn.clicked.connect(self.toggle_glim);row.addWidget(self.glim_btn)
        self.save_mapping_box=QCheckBox('保存 GLIM dump + rosbag')
        self.save_mapping_box.setChecked(bool(self.cfg.get('save_mapping_data',True)));row.addWidget(self.save_mapping_box)
        row.addStretch()
        row=QHBoxLayout();c.addLayout(row)
        self.glim_cfg_btn=button('GLIM 配置');self.glim_cfg_btn.clicked.connect(self.choose_glim_config);row.addWidget(self.glim_cfg_btn)
        self.editor_btn=button('编辑三维地图');self.editor_btn.clicked.connect(self.open_map_editor);row.addWidget(self.editor_btn)
        self.rviz_btn=button('RViz');self.rviz_btn.clicked.connect(self.open_rviz);row.addWidget(self.rviz_btn);row.addStretch()
        self.run_label=text_label('')
        c.addWidget(self.run_label)
        self.asset_label=text_label('重定位资产：选择地图后显示；AGT Polar Context 不是 Scan Context（SC）。')
        c.addWidget(self.asset_label)
        self.mapping_backend_box.currentIndexChanged.connect(self._mapping_backend_changed)
        self._mapping_backend_changed()
        frame=QFrame();frame.setObjectName('viewer');l.addWidget(frame,1)
        fl=QVBoxLayout(frame);fl.setContentsMargins(10,10,10,10);fl.addWidget(self.cloud_view,1)
        row=QHBoxLayout();fl.addLayout(row);self.cloud_count=text_label('点云：等待数据')
        row.addWidget(self.cloud_count);row.addStretch();self.odom_label=text_label('base_link 局部位姿：未收到');row.addWidget(self.odom_label)

    def _page_maps(self):
        l=self._page_shell(scroll=True);l.addWidget(text_label('地图生成与编辑','title'))
        cloud=card(l,'1. 从 FAST-LIO2／GLIM 导出三维 PCD，再生成二维格栅地图')
        row=QHBoxLayout();cloud.addLayout(row);self.cloud_box=QComboBox();row.addWidget(self.cloud_box,1)
        imp=button('导入 PCD');imp.clicked.connect(self.import_pcd);row.addWidget(imp)
        gen=button('生成二维地图','primary');gen.clicked.connect(self.generate_grid);row.addWidget(gen)
        refresh=button('刷新');refresh.clicked.connect(self.refresh_library);row.addWidget(refresh)
        self.cloud_box.currentIndexChanged.connect(self.preview_cloud)
        clean=card(l,'1a. 三维 PCD 全量清洗（保留原图，清洗后再生成栅格）')
        clean.addWidget(text_label('预览仅显示采样点；删除 ROI 在 C++ PCL 后端作用于全量 PCD。输入地图坐标系 XYZ 包围盒，不是相对车体的坐标。'))
        self.cloud_info=text_label('选择上方 PCD 后加载预览；请核对后方杆件在地图中的实际坐标')
        clean.addWidget(self.cloud_info)
        preview=button('预览选中 PCD');preview.clicked.connect(self.preview_cloud)
        clean.addWidget(preview)
        self.clean_view.setMinimumSize(350,210)
        clean.addWidget(self.clean_view)
        limits=QGridLayout();clean.addLayout(limits)
        self.cloud_roi_edits={}
        for column,axis in enumerate(('X','Y','Z')):
            for row,(bound,label) in enumerate((('min','下限'),('max','上限'))):
                spin=QDoubleSpinBox();spin.setRange(-1000.,1000.)
                spin.setDecimals(3);spin.setSingleStep(.05);spin.setValue(0.)
                self.cloud_roi_edits[(axis.lower(),bound)]=spin
                limits.addWidget(QLabel(axis+label+' (m)'),row,column*2)
                limits.addWidget(spin,row,column*2+1)
        self.cloud_clean_btn=button('删除框内杆件/残影 → 新 PCD 版本','primary')
        self.cloud_clean_btn.clicked.connect(self.clean_cloud_roi)
        clean.addWidget(self.cloud_clean_btn)
        clean.addWidget(text_label('警告：重复结构或沿全程移动的车体残影不应盲目用大范围框删除。先预览小 ROI，再核对新版本；相同 PCD 必须用于定位和栅格转换。'))
        frame=QFrame();frame.setObjectName('viewer');l.addWidget(frame,1)
        v=QVBoxLayout(frame);v.addWidget(self.map_view)
        row=QHBoxLayout();l.addLayout(row);self.map_box=QComboBox();row.addWidget(self.map_box,1)
        load=button('加载版本');load.clicked.connect(self.load_selected_map);row.addWidget(load)
        self.map_brush=QComboBox();self.map_brush.addItem('浏览（不绘制）',None)
        self.map_brush.addItem('障碍（黑）',0);self.map_brush.addItem('空闲（白）',254);self.map_brush.addItem('未知（灰）',205)
        self.map_brush.addItem('电子围栏（多边形）','keepout')
        self.map_brush.currentIndexChanged.connect(self._map_tool_changed)
        row=QHBoxLayout();l.addLayout(row)
        row.addWidget(self.map_brush,1)
        undo=button('撤销');undo.clicked.connect(self.map_view.undo_last);row.addWidget(undo)
        fence_row=QHBoxLayout();l.addLayout(fence_row)
        finish=button('完成围栏');finish.clicked.connect(self.finish_keepout);fence_row.addWidget(finish)
        cancel=button('取消当前多边形');cancel.clicked.connect(self.cancel_keepout);fence_row.addWidget(cancel)
        fence_row.addWidget(text_label('清除残影：用“空闲”或“未知”画笔覆盖。电子围栏：逐点点击，双击或按“完成围栏”；保存时同时写入 PGM 和 keepout_zones.yaml。'),1)
        row=QHBoxLayout();l.addLayout(row);imp2=button('导入 PGM + YAML');imp2.clicked.connect(self.import_pgm);row.addWidget(imp2)
        draft=button('保存草稿');draft.clicked.connect(lambda:self.export_map('DRAFT'));row.addWidget(draft)
        save=button('保存为导航地图','primary');save.clicked.connect(lambda:self.export_map('ACCEPTED'));row.addWidget(save)
        diff=card(l,'3. 正式地图差异更新（自动比较，只生成待审核版本）')
        diff.addWidget(text_label('基准必须是 ACCEPTED 地图；候选必须是来自新 PCD 的 DRAFT。分辨率、地图朝向与栅格原点需可对齐。不会自动覆盖正式地图。'))
        diff_row=QHBoxLayout();diff.addLayout(diff_row)
        self.diff_base_box=QComboBox();self.diff_base_box.setMinimumWidth(180);diff_row.addWidget(self.diff_base_box,1)
        self.diff_candidate_box=QComboBox();self.diff_candidate_box.setMinimumWidth(180);diff_row.addWidget(self.diff_candidate_box,1)
        self.diff_alignment_evidence=QLineEdit();self.diff_alignment_evidence.setPlaceholderText('不同 PCD 必填：外部配准/测量共同坐标系的核验依据（本工具不估计配准）');diff.addWidget(self.diff_alignment_evidence)
        compare=button('比较并生成更新候选','primary');compare.clicked.connect(self.compare_map_versions);diff_row.addWidget(compare)
        self.diff_status_label=text_label('差异报告：尚未运行');diff.addWidget(self.diff_status_label)
        self.diff_preview_label=QLabel('差异图将在此显示');self.diff_preview_label.setAlignment(Qt.AlignCenter)
        self.diff_preview_label.setMinimumSize(500,220);self.diff_preview_label.setMaximumHeight(300)
        self.diff_preview_label.setStyleSheet('background:#101820;border:1px solid #2b3b46;border-radius:8px;color:#90a1ae')
        diff.addWidget(self.diff_preview_label)
        review_row=QHBoxLayout();diff.addLayout(review_row)
        self.diff_review_box=QComboBox();review_row.addWidget(self.diff_review_box,1)
        self.diff_review_box.activated.connect(lambda _:self._api('map_diff_preview','GET',f'/api/maps/{self.diff_review_box.currentData()}/diff'))
        self.diff_review_reason=QLineEdit();self.diff_review_reason.setPlaceholderText('填写人工审核依据');review_row.addWidget(self.diff_review_reason,1)
        for title,status in (('审核通过','ACCEPTED'),('拒绝候选','REJECTED')):
            review=button(title);review.clicked.connect(lambda _,s=status:self.review_map_update(s));review_row.addWidget(review)
        row.addStretch();self.map_info=text_label('地图：未加载；Unknown 始终保持未知，不会自动变空闲');l.addWidget(self.map_info)
        self.map_write_status=text_label('地图写入：空闲');l.addWidget(self.map_write_status)

    def _map_tool_changed(self):
        keepout=self.map_brush.currentData()=='keepout'
        self.map_view.keepout_mode=keepout
        self.map_view.paint_value=None if keepout else self.map_brush.currentData()
        self.map_view.polygon=[]
        self.map_view.setToolTip('逐点点击绘制围栏，双击完成；右键拖动；滚轮缩放' if keepout else
                                 '左键绘制；右键或中键拖动；滚轮缩放')
        self.map_view.update()

    def finish_keepout(self):
        if not self.map_view.finish_keepout():
            self.map_info.setText('电子围栏至少需要 3 个顶点；请先选择“电子围栏（多边形）”。');return
        self.map_info.setText(f'已绘制 {len(self.map_view.keepout)} 个电子围栏；保存为新地图版本后生效。')

    def cancel_keepout(self):
        self.map_view.polygon=[];self.map_view.update()

    def _page_navigation(self):
        l=self._page_shell(scroll=True);l.addWidget(text_label('定位与自主导航','title'))
        c=card(l,'1. 地图与定位后端')
        row=QHBoxLayout();c.addLayout(row);self.nav_map_box=QComboBox();row.addWidget(self.nav_map_box,1)
        self.nav_map_box.activated.connect(self.select_navigation_map)
        self.lio_backend_box=QComboBox()
        self.lio_backend_box.addItem('Batch-LIO → base_link','batch_lio')
        self.lio_backend_box.addItem('FAST-LIO2 → base_link','fastlio2')
        self.lio_backend_box.setCurrentIndex(max(0,self.lio_backend_box.findData(self.cfg.get('lio_backend','batch_lio'))))
        self.lio_backend_box.setToolTip('两个后端互斥；FAST-LIO2 经 agt_fastlio_adapter 转换为 base_link 后输出 /agt/odometry/local')
        row=QHBoxLayout();c.addLayout(row)
        row.addWidget(text_label('里程计后端'));row.addWidget(self.lio_backend_box,1)
        self.global_backend_box=QComboBox()
        self.global_backend_box.addItem('3D-BBS + GICP','bbs')
        self.global_backend_box.addItem('Scan Context + KISS-Matcher + GICP（实验）','sc_kiss')
        self.global_backend_box.setCurrentIndex(max(0,self.global_backend_box.findData(self.cfg.get('global_backend','bbs'))))
        self.global_backend_box.setToolTip('每次只使用一个粗定位后端；精化均为 GICP')
        row=QHBoxLayout();c.addLayout(row)
        row.addWidget(text_label('重定位后端'));row.addWidget(self.global_backend_box,1)
        row=QHBoxLayout();c.addLayout(row)
        for title,task in [('启动定位','localization'),('启动导航','navigation')]:
            b=button(title,'primary' if task=='navigation' else None)
            b.clicked.connect(lambda _,t=task:self.start_field_session(t));row.addWidget(b)
        stop=button('结束会话');stop.clicked.connect(self.stop_field_session);row.addWidget(stop)
        row=QHBoxLayout();c.addLayout(row)
        reloc=button('全局重定位');reloc.clicked.connect(self.relocalize);row.addWidget(reloc)
        stop_goal=button('停止当前目标 / 任务','danger');stop_goal.clicked.connect(self.stop_goal);row.addWidget(stop_goal)
        row.addWidget(text_label('云台在设备页独立启动'))
        self.localization_label=text_label('定位状态：等待 Workbench');self.localization_label.setWordWrap(True);row.addWidget(self.localization_label,1)
        frame=QFrame();frame.setObjectName('viewer');l.addWidget(frame,1)
        v=QVBoxLayout(frame);v.addWidget(self.tri_view,1)
        self.view_status_label=text_label('实时视图：等待地图、点云和 map → base_link TF');v.addWidget(self.view_status_label)
        row=QHBoxLayout();v.addLayout(row)
        self.target_mode_box=QComboBox();self.target_mode_box.addItem('两次点击导航','goal');self.target_mode_box.addItem('两次点击添加航点','route')
        self.target_mode_box.currentIndexChanged.connect(lambda _:self.nav_view.reset_heading())
        row.addWidget(self.target_mode_box)
        self.photo_box=QCheckBox('到点拍照');self.photo_box.setChecked(True);row.addWidget(self.photo_box)
        self.gimbal_heading=QDoubleSpinBox();self.gimbal_heading.setRange(-160,160);self.gimbal_heading.setSuffix('° 航向');row.addWidget(self.gimbal_heading)
        self.gimbal_pitch=QDoubleSpinBox();self.gimbal_pitch.setRange(-90,90);self.gimbal_pitch.setSuffix('° 俯仰');row.addWidget(self.gimbal_pitch)
        clear=button('清空航点');clear.clicked.connect(self.clear_route);row.addWidget(clear)
        row=QHBoxLayout();v.addLayout(row)
        self.route_box=QComboBox();row.addWidget(self.route_box,1)
        load_route=button('加载路线');load_route.clicked.connect(self.load_route);row.addWidget(load_route)
        save_route=button('保存路线');save_route.clicked.connect(self.save_route);row.addWidget(save_route)
        self.run_route_btn=button('开始航点任务','primary');self.run_route_btn.clicked.connect(self.run_route);row.addWidget(self.run_route_btn)
        self.mission_label=text_label('左键两次：先选择位置，再选择航向；航点模式下可逐点保存路线。')
        v.addWidget(self.mission_label)
        c=card(l,'2. 相对运动')
        row=QHBoxLayout();c.addLayout(row);self.distance=QDoubleSpinBox();self.distance.setRange(.02,5.)
        self.distance.setDecimals(2);self.distance.setSingleStep(.1);self.distance.setValue(.5);self.distance.setSuffix(' m')
        row.addWidget(self.distance)
        for title,sign in [('向前',1),('向后',-1)]:
            b=button(title);b.clicked.connect(lambda _,s=sign:self.move_distance(s));row.addWidget(b)
        self.pose_label=text_label('base_link 当前位置：未知');row.addWidget(self.pose_label,1)

    def _page_parameters(self):
        l=self._page_shell();l.addWidget(text_label('参数配置','title'))
        scroll=QScrollArea();scroll.setWidgetResizable(True);scroll.setFrameShape(QFrame.NoFrame)
        content=QWidget();scroll.setWidget(content);l.addWidget(scroll,1)
        body=QVBoxLayout(content);body.setContentsMargins(0,0,8,0);body.setSpacing(14)
        topics=card(body,'传感器与显示话题')
        form=QFormLayout();form.setFieldGrowthPolicy(QFormLayout.AllNonFixedFieldsGrow);topics.addLayout(form)
        self.topic_edits={}
        for key,label in [('raw_lidar_topic','MID360S 原始 CustomMsg'),('imu_topic','IMU'),
                          ('odom_topic','base_link 里程计'),('points_topic','GLIM 地图'),
                          ('live_points_topic','GLIM 实时点云'),('map_topic','Nav2 二维地图')]:
            editor=QLineEdit(str(self.cfg.get(key,'')));self.topic_edits[key]=editor;form.addRow(label,editor)
        topics.addWidget(text_label('显示话题的修改在重新连接 ROS 显示适配器后生效；会话与设备参数在下次启动会话时生效。'))
        mapping=card(body,'建图与输出')
        form=QFormLayout();mapping.addLayout(form)
        self.glim_config_edit=QLineEdit(str(self.cfg.get('glim_config_path','')))
        form.addRow('GLIM 配置目录',self.glim_config_edit)
        self.bag_root_edit=QLineEdit(str(self.cfg.get('bag_root','~/agt_studio_runs')))
        form.addRow('建图输出目录',self.bag_root_edit)
        recording=card(body,'独立录包与建图录包')
        form=QFormLayout();recording.addLayout(form)
        self.record_template_box=QComboBox();self.record_template_box.addItems(RECORD_TEMPLATES)
        index=self.record_template_box.findText(str(self.cfg.get('record_template','full_field_test')))
        self.record_template_box.setCurrentIndex(max(0,index));form.addRow('独立录包模板',self.record_template_box)
        self.record_topics_edit=QTextEdit('\n'.join(parse_topics(self.cfg.get('record_topics',[]))))
        self.record_topics_edit.setPlaceholderText('留空：使用上方模板。\n自定义：每行一个绝对话题，也可用逗号分隔。\n例如 /livox/lidar、/camera/image_raw、/tf')
        self.record_topics_edit.setMaximumHeight(105);form.addRow('自定义录包话题',self.record_topics_edit)
        recording.addWidget(text_label('独立录包只选择当前 ROS 中实际存在的话题，并记录缺失话题、类型、QoS、配置及数据 SHA。FAST-LIO2 建图录包由 Workbench 独占；GLIM 实验建图使用同一自定义列表。'))
        projection=card(body,'三维点云投影为二维栅格')
        form=QFormLayout();projection.addLayout(form)
        p=dict(DEFAULT_CONFIG['projection']);p.update(self.cfg.get('projection') or {})
        self.projection_edits={}
        for key,label,low,high,step in [('resolution','栅格分辨率（m）',.01,1.,.01),
                ('ground_cell_size','局部地面单元（m）',.1,5.,.1),
                ('ground_percentile','地面高度分位数',0.,1.,.05),
                ('ground_tolerance','地面点容差（m）',0.,2.,.02),
                ('obstacle_min_height','相对地面障碍下限（m）',0.,5.,.02),
                ('obstacle_max_height','相对地面障碍上限（m）',.01,10.,.1)]:
            editor=QDoubleSpinBox();editor.setRange(low,high);editor.setDecimals(3);editor.setSingleStep(step)
            editor.setValue(float(p[key]));self.projection_edits[key]=editor;form.addRow(label,editor)
        self.min_points_box=QSpinBox();self.min_points_box.setRange(1,10000)
        self.min_points_box.setValue(int(p.get('occupied_threshold',p.get('min_points',2))))
        form.addRow('判障碍最少点数',self.min_points_box)
        projection.addWidget(text_label('复用 agt_pcd2grid_exporter/local_ground：按局部地面高度判障碍。未观测区域保留 Unknown；静止扫描不会凭空补出可通行区域。'))
        chassis=card(body,'Bunker 运动限制与响应参数（版本化 YAML）')
        form=QFormLayout();chassis.addLayout(form)
        self.bunker_profile=None;self.bunker_profile_edits={}
        for key,label,high,step in [
                ('forward_mps','最大前进速度（m/s）',3.,.02),
                ('reverse_mps','最大后退速度（m/s）',3.,.02),
                ('angular_radps','最大转向角速度（rad/s）',3.,.02),
                ('linear_accel_mps2','线加速度上限（m/s²）',4.,.02),
                ('linear_decel_mps2','线减速度上限（m/s²）',4.,.02),
                ('angular_accel_radps2','角加速度上限（rad/s²）',6.,.02),
                ('angular_decel_radps2','角减速度上限（rad/s²）',6.,.02),
                ('command_timeout_sec','命令超时（s）',2.,.02)]:
            editor=QDoubleSpinBox();editor.setRange(.001,high);editor.setDecimals(3)
            editor.setSingleStep(step);editor.setEnabled(False)
            self.bunker_profile_edits[key]=editor;form.addRow(label,editor)
        buttons=QHBoxLayout();chassis.addLayout(buttons)
        load_profile_btn=button('读取当前底盘 YAML')
        load_profile_btn.clicked.connect(lambda:self._api('profile_get','GET','/api/profiles'))
        buttons.addWidget(load_profile_btn)
        save_profile_btn=button('保存新版 Bunker YAML','primary')
        save_profile_btn.clicked.connect(self.save_bunker_profile)
        buttons.addWidget(save_profile_btn);buttons.addStretch()
        self.bunker_profile_status=text_label('尚未读取 Bunker 配置')
        chassis.addWidget(self.bunker_profile_status)
        chassis.addWidget(text_label(
            '仅编辑 Workbench 速度、加减速与时效限制；不是电机固件 PID，也不是 Nav2 跟踪增益。'
            ' 保存会创建新 YAML 和 SHA，须先结束现场会话；下次启动才使用。'
            ' 不改变急停、遥控接管和运动许可。'))

        camera=card(body,'C1 云台相机')
        form=QFormLayout();camera.addLayout(form)
        self.camera_device_edit=QLineEdit(str(self.cfg.get('camera_device_path','/dev/video0')))
        self.gimbal_port_edit=QLineEdit(str(self.cfg.get('gimbal_port_name','')))
        form.addRow('视频设备',self.camera_device_edit);form.addRow('云台串口',self.gimbal_port_edit)
        research=card(body,'农科院重定位实验')
        self.sc_database_edit=QLineEdit(str(self.cfg.get('sc_database','')))
        self.sc_database_edit.setPlaceholderText('独立 Scan Context 数据库目录，含 manifest.json；BBS 不使用')
        research.addWidget(self.sc_database_edit)
        assets=card(body,'重定位后端与地图资产（当前原生实现）')
        assets.addWidget(text_label('3D-BBS + GICP：有候选资产时加载 polar_context.db 与 voxelmaps_coords。AGT Polar Context 先以 ring key 从全库预筛选 40 个，再做 sector 相似度／位移排序，取 candidate_top_k = 2 个执行局部 BBS，最后 small_gicp GICP 精化；无数据库的旧地图走整图 BBS 回退。'))
        assets.addWidget(text_label('Scan Context + KISS-Matcher + GICP：从独立 SC 数据库检索 Top-K 位置与航向，KISS 匹配候选局部地图，再调用原 small_gicp 精化。数据库必须与选定 PCD、导航地图及 LIO 标定一致；失败后停止定位并手动切回 BBS。'))
        row=QHBoxLayout();body.addLayout(row)
        save=button('保存参数','primary');save.clicked.connect(self.save_parameters);row.addWidget(save)
        self.parameter_status=text_label('保存到用户配置：'+str(USER_CONFIG));row.addWidget(self.parameter_status,1)
        body.addStretch(1)

    def _page_logs(self):
        l=self._page_shell();l.addWidget(text_label('运行日志','title'))
        self.log=QTextEdit();self.log.setReadOnly(True);l.addWidget(self.log,1)
        row=QHBoxLayout();l.addLayout(row);s=button('保存日志');s.clicked.connect(self.save_logs);row.addWidget(s);row.addStretch()

    def _setup_backend(self):
        workspace=os.environ.get('AGT_FIELD_TEST_WS')
        default_root=Path(workspace)/'src/agt_field_test_workbench' if workspace else Path(__file__).resolve().parents[2]
        root=Path(str(self.cfg.get('workbench_root') or default_root)).expanduser().resolve()
        self.cfg['workbench_root']=str(root)
        client=WorkbenchClient(self.cfg['workbench_api'])
        self.api=ApiWorker(client);self.api.completed.connect(self._on_api);self.api.start()
        # Serialize control/lifecycle operations separately from long asset I/O.
        # A goal followed by stop retains order, without waiting for a PCD diff.
        self.control_api=ApiWorker(client);self.control_api.completed.connect(self._on_api);self.control_api.start()
        self.api.call('backend_probe','GET','/api/maps',timeout=.45)
        QTimer.singleShot(1300,lambda:self._api('profile_get','GET','/api/profiles'))
        QTimer.singleShot(1400,self.refresh_library);QTimer.singleShot(1600,self.refresh_live)
        QTimer.singleShot(1800,self.refresh_jobs)

    def _launch_backend(self):
        root=Path(self.cfg['workbench_root'])
        if self.cfg.get('auto_start_workbench',True) and (root/'workbench/server.py').is_file():
                env=os.environ.copy();env['PYTHONPATH']=str(root)+os.pathsep+env.get('PYTHONPATH','')
                # Workbench owns live mapping/recording jobs. It must outlive
                # the GUI; otherwise saving/crashing Qt orphans active jobs.
                from PyQt5.QtCore import QProcessEnvironment, QIODevice
                process_env=QProcessEnvironment.systemEnvironment()
                process_env.insert('PYTHONPATH',env['PYTHONPATH'])
                server=QProcess()
                server.setProgram('python3')
                server.setArguments(['-m','workbench.server','--host','127.0.0.1','--port','8765'])
                server.setWorkingDirectory(str(root))
                server.setProcessEnvironment(process_env)
                (root/'work').mkdir(parents=True,exist_ok=True)
                server.setStandardOutputFile(str(root/'work/studio_workbench.log'),QIODevice.Append)
                server.setProcessChannelMode(QProcess.MergedChannels)
                started,pid=server.startDetached()
                if started:
                    self.write_log(f'Workbench 独立后台服务已启动（PID {pid}）；关闭 Qt 不会中断建图或录包')
                else:
                    self.write_log('Workbench 后台启动失败：'+server.errorString())

    def _api(self,name,method,path,payload=None):
        if self.demo:self.write_log('演示模式：未调用 Workbench');return
        if not self.api:self.write_log('Workbench 尚未初始化');return
        timeout=None
        if name in ('record_start','session_stop_all'):timeout=120
        elif name=='cloud_project':timeout=1500
        elif name in ('cloud_import','cloud_clean'):timeout=600
        elif name in ('map_revise','map_diff','map_review'):timeout=120
        elif name=='component_stop':timeout=60
        elif name=='cloud_preview':timeout=30
        elif name in ('record_stop','session_stop') or name.startswith(('session_','device_')):timeout=30
        if name=='session_sensors':self.pending_mapping_start=True
        control=method=='POST' and path.startswith(('/api/control/','/api/session/','/api/components/','/api/mapping/'))
        worker=self.control_api if self.control_api and control else self.api
        worker.call(name,method,path,payload,timeout)

    def refresh_library(self):
        self._api('refresh_maps','GET','/api/maps');self._api('refresh_clouds','GET','/api/clouds')
        self._api('refresh_routes','GET','/api/routes')

    def compare_map_versions(self):
        base_id=self.diff_base_box.currentData()
        candidate_id=self.diff_candidate_box.currentData()
        if not base_id or not candidate_id:
            QMessageBox.warning(self,'地图版本不足','需要一个 ACCEPTED 基准和一个有同源 PCD 的 DRAFT 候选。');return
        if base_id==candidate_id:
            QMessageBox.warning(self,'版本选择错误','基准和候选不能是同一个地图版本。');return
        self.diff_status_label.setText('正在对齐栅格并计算地图差异……')
        payload={'base_id':base_id,'candidate_id':candidate_id}
        if self.diff_alignment_evidence.text().strip():
            payload['alignment']={'method':'EXTERNAL_ALIGNMENT_REVIEWED','evidence':self.diff_alignment_evidence.text().strip(),
                'base_pcd_sha256':self.maps_by_id[base_id].get('source_pcd_sha256'),
                'candidate_pcd_sha256':self.maps_by_id[candidate_id].get('source_pcd_sha256')}
        self._api('map_diff','POST','/api/maps/diff',payload)

    def review_map_update(self,status):
        ident=self.diff_review_box.currentData();reason=self.diff_review_reason.text().strip()
        if not ident or not reason or ident!=getattr(self,'diff_displayed_id',None):
            QMessageBox.warning(self,'审核依据','请选择更新候选并填写审核依据。');return
        self._api('map_review','POST',f'/api/maps/{ident}/status',{'status':status,'reason':reason})

    def refresh_live(self):
        if not self.live_in_flight:
            self.live_in_flight=True;self._api('refresh_live','GET','/api/live')

    def refresh_jobs(self):
        if not self.jobs_in_flight:
            self.jobs_in_flight=True;self._api('refresh_jobs','GET','/api/jobs')

    @staticmethod
    def _fill_combo(combo,records,current):
        combo.blockSignals(True);combo.clear()
        for item in records:
            name=str(item.get('name') or '').strip();short_id=item['id'].rsplit('-',1)[-1]
            label=(f"{name}  ·  {short_id}" if name else item['id'])+f"  ·  {item.get('status','DRAFT')}"
            combo.addItem(label,item['id'])
        if current:
            index=combo.findData(current)
            if index>=0:combo.setCurrentIndex(index)
        combo.blockSignals(False)

    def _on_api(self,name,ok,value):
        if name=='refresh_components':
            self.components_in_flight=False
            self.component_status_label.setText(' · '.join(f"{name}: {state['process_state']}/{state['readiness']}" for name,state in value.items()) if ok else '组件状态读取失败：'+str(value))
            if ok:self.component_status_label.setToolTip('\n'.join(name+': '+str(state.get('reason') or '') for name,state in value.items()))
            return
        if name=='backend_probe':
            if not ok:self._launch_backend()
            return
        if name=='camera_health':self.camera_health_pending=False
        if name=='map_loaded':
            self.nav_view.navigation_mode=True
            if ok:
                manifest,pixels=value
                if manifest['id']==getattr(self,'pending_display_map_id',None):self._load_map(manifest,pixels)
            else:self.write_log('地图读取失败：'+str(value))
            return
        if name=='refresh_live':self.live_in_flight=False
        if name=='refresh_jobs':self.jobs_in_flight=False
        if not ok:
            self.write_log(f'{name}: {value}')
            if name=='session_stop':self.pending_session=None
            if not name.startswith('refresh_'):QMessageBox.warning(self,'操作未完成',str(value))
            if name in ('session_sensors','mapping_start'):self.mapping_state.setText('启动失败')
            if name in ('camera_start','camera_stop'):
                self.camera_status_label.setText('云台操作失败：'+str(value)[:140])
            if name in ('camera_capture','camera_move'):
                self.capture_status_label.setText('云台/拍照失败：'+str(value)[:160])
            if name=='map_diff' and hasattr(self,'diff_status_label'):
                self.diff_status_label.setText('地图差异失败：'+str(value)[:180])
            if name=='map_revise':
                self.map_saving=False
                self.map_write_status.setText('保存失败：'+str(value)[:160])
            if name=='cloud_project':
                self.grid_building=False
                self.map_write_status.setText('二维地图/索引生成失败：'+str(value)[:160])
            if name=='mapping_finish':
                self.glim_btn.setEnabled(True);self.mapping_saving=False
                self.mapping_state.setText('保存请求失败；建图任务仍在运行')
                self.mapping_save_status.setText('保存请求失败：'+str(value)[:160])
            if name=='cloud_clean':self.cloud_clean_btn.setEnabled(True)
            return
        if name=='profile_get':
            value=next((p for p in value if p.get('robot')=='bunker'),None)
            if not value or not value.get('runtime_valid',False):
                self.bunker_profile=None
                self.bunker_profile_status.setText('Bunker 配置无效；请检查 YAML：'+str((value or {}).get('error','not found')))
                return
            self.bunker_profile=dict(value)
            for key,editor in self.bunker_profile_edits.items():
                if key not in value:
                    self.bunker_profile=None
                    self.bunker_profile_status.setText('缺少必要参数：'+key)
                    return
                editor.setEnabled(True);editor.setValue(float(value[key]))
            self.bunker_profile_status.setText('当前版本：'+str(value.get('profile_path',''))
                +'  SHA '+str(value.get('profile_sha256',''))[:12])
            return
        if name=='profile_save':
            self.bunker_profile_status.setText('已保存并激活下次会话的 Bunker 参数版本：'
                +str(value.get('profile_path','')))
            self.write_log('Bunker 控制参数已生成不可变 YAML，新会话才生效；已运行会话不受影响')
            self._api('profile_get','GET','/api/profiles')
            return
        if name=='refresh_maps':
            self.maps_by_id={m['id']:m for m in value}
            self._fill_combo(self.map_box,value,self.current_map_id)
            self._fill_combo(self.nav_map_box,value,self.current_map_id)
            if hasattr(self,'diff_base_box'):
                accepted=[m for m in value if m.get('status')=='ACCEPTED' and m.get('source_pcd')]
                candidates=[m for m in value if m.get('status')=='DRAFT' and m.get('source_pcd')
                            and m.get('update_type')!='AUTOMATIC_DIFF_DRAFT']
                self._fill_combo(self.diff_base_box,accepted,self.diff_base_box.currentData())
                self._fill_combo(self.diff_candidate_box,candidates,self.diff_candidate_box.currentData())
                self._fill_combo(self.diff_review_box,[m for m in value if m.get('diff_report_id')],getattr(self,'pending_review_id',None) or self.diff_review_box.currentData())
                self.pending_review_id=None
            if self.pending_load_map_id:
                selected=self.maps_by_id.get(self.pending_load_map_id)
                self.pending_load_map_id=None
                if selected:self._load_map(selected)
            return
        if name=='refresh_clouds':
            selected=getattr(self,'pending_cloud_id',None) or self.cloud_box.currentData()
            self.clouds_by_id={c['id']:c for c in value}
            self._fill_combo(self.cloud_box,value,selected)
            self.pending_cloud_id=None
            return
        if name=='refresh_routes':
            self.routes_by_id={r['id']:r for r in value}
            self._fill_combo(self.route_box,value,self.current_route_id);return
        if name=='refresh_live':
            self.live_status=value;self.current_session=value.get('session')
            metrics=value.get('global_relocalization')
            signature=json.dumps(metrics,sort_keys=True,ensure_ascii=False) if metrics else ''
            if signature and signature!=getattr(self,'last_relocalization_log',''):
                self.last_relocalization_log=signature;self.write_log('重定位：'+signature)
            self.nav_view.plan=value.get('plan',[]);self.nav_view.update()
            self._update_live();return
        if name=='refresh_jobs':
            # Re-attach after a Qt restart; the Workbench, not this window,
            # owns the live mapping job and its raw rosbag.
            if not self.mapping_job_id:
                mapping_live=next((j for j in value if j.get('kind')=='mapping_live'
                    and j.get('state')=='RUNNING' and j.get('robot')==self._robot()),None)
                if mapping_live:
                    self.mapping_job_id=mapping_live['id']
                    self.current_run=Path(mapping_live['output'])
                    self.glim_btn.setText('结束并保存建图')
                    self.glim_btn.setEnabled(True)
                    self.map_name_edit.setEnabled(False)
                    self.mapping_backend_box.setEnabled(False)
                    self.mapping_state.setText('已恢复后台 FAST-LIO2 建图任务')
                    self.run_label.setText('后台建图任务：'+self.mapping_job_id+
                                           ' · 输出：'+str(self.current_run))
                    self._mapping_backend_changed()
            self.jobs_cache=list(value)
            self.camera_job_state=next((j for j in value if j.get('kind')=='camera_gimbal'),{'state':'STOPPED'})
            if self.camera_job_state.get('state')=='RUNNING' and self.camera_health_state:
                self.camera_job_state.update({k:v for k,v in self.camera_health_state.items()
                                              if k in ('readiness','ready','health','health_error')})
            elif self.camera_job_state.get('state')!='RUNNING':
                self.camera_health_state={}
            captures=[j for j in value if j.get('kind')=='camera_capture' and j.get('id')!=self.last_capture_job_id]
            if captures:
                newest=captures[0]
                if newest.get('state') in ('PASS','FAILED','CANCELLED'):
                    self.last_capture_job_id=newest.get('id')
                    result=newest.get('capture_result') or {}
                    if newest.get('state')=='PASS':
                        self.capture_status_label.setText('拍照成功：'+str(result.get('image_path','未返回路径')))
                    else:
                        self.capture_status_label.setText('拍照失败：'+str(result.get('message',newest.get('state'))))
            self._update_global_status()
            self.camera_status_label.setText('云台：'+self.camera_job_state.get('readiness',self.camera_job_state.get('state','STOPPED')))
            active_capture=next((j for j in value if j.get('kind')=='camera_capture' and j.get('state')=='RUNNING'),None)
            if active_capture and active_capture.get('feedback'):
                feedback=active_capture['feedback'];angles=feedback['actual_angles']
                self.capture_status_label.setText(f"{feedback['stage']} · 实际航向 {angles['heading']:.1f}° 俯仰 {angles['pitch']:.1f}° · 稳定样本 {feedback['stable_count']}")
            self._update_global_status()
            running=next((j for j in value if j.get('kind')=='recorder' and j.get('state')=='RUNNING'),None)
            self.recorder_id=running['id'] if running else None
            if running:self.recorder_output=running.get('output','')
            self._update_recording()
            if self.mapping_job_id:
                mapping=next((j for j in value if j.get('id')==self.mapping_job_id),None)
                if mapping and mapping.get('state')=='RUNNING':
                    if mapping.get('save_requested_at') or self.mapping_saving:
                        self.mapping_saving=True
                        phases={'PCD_IMPORT':'正在保存：登记三维 PCD',
                                'START_CLEARANCE':'正在分析：起点人员自由空间证据（候选不自动替换导航地图）',
                                'GRID_EXPORT':'正在保存：生成二维栅格',
                                'BBS_ASSETS':'正在保存：构建同源 3D-BBS 索引',
                                'COMPLETE':'正在保存：最终校验',
                                'PARTIAL_ASSETS_FAILED':'地图已导出，BBS 构建有问题'}
                        phase=(mapping.get('save_status') or {}).get('stage','LIO_EXPORT')
                        message=phases.get(phase,'正在保存：导出 FAST-LIO2 点云和 rosbag')
                        self.mapping_state.setText(message)
                        self.mapping_save_status.setText('保存状态：'+message)
                        self.glim_btn.setEnabled(False)
                elif mapping and mapping.get('state')!='RUNNING':
                    state=mapping.get('state')
                    result=mapping.get('result') or {}
                    assets=result.get('relocalization_assets') or {}
                    saved=state=='PASS' and bool(result.get('map_version'))
                    self.mapping_job_id=None;self.mapping_saving=False
                    self.current_run=Path(mapping['output']) if mapping.get('output') else self.current_run
                    self.glim_btn.setEnabled(True);self.glim_btn.setText('开始建图')
                    self.map_name_edit.setEnabled(True);self.mapping_backend_box.setEnabled(True)
                    if saved:
                        self.current_map_id=result['map_version']
                        self.pending_load_map_id=self.current_map_id
                        bbs='BBS 索引已生成' if assets.get('state')=='READY' else 'BBS 索引未就绪（查看日志）'
                        label='地图已经保存 · '+bbs
                        self.mapping_state.setText(label)
                        self.mapping_save_status.setText('保存状态：'+label+' · 地图版本 '+self.current_map_id)
                        self.write_log(label+' · '+str(mapping.get('output','')))
                    else:
                        self.mapping_state.setText('保存失败 · '+str(state))
                        self.mapping_save_status.setText('保存状态：失败／未完成；请检查任务日志 '+str(mapping.get('log','')))
                        self.write_log('FAST-LIO2 建图未成功保存：'+str(mapping.get('log','')))
                    self._mapping_backend_changed();self.refresh_library()
            return
        if name=='session_sensors':
            if self.pending_mapping_start:
                self.pending_mapping_start=False;self._start_mapping_processes()
            return
        if name=='mapping_start':
            self.mapping_job_id=value['id'];self.mapping_saving=False
            self.current_run=Path(value['output'])
            self.glim_btn.setEnabled(True);self.glim_btn.setText('结束并保存建图')
            self.map_name_edit.setEnabled(False);self.mapping_backend_box.setEnabled(False);self._mapping_backend_changed()
            self.mapping_state.setText('FAST-LIO2 无 PGO 建图运行中')
            self.mapping_save_status.setText('保存状态：建图中，尚未开始保存')
            self.run_label.setText(f"地图：{self.current_map_name} · Workbench 任务：{self.mapping_job_id} · 输出：{self.current_run}")
            self.write_log('FAST-LIO2 复用当前传感器与 TF 会话；未启动额外雷达驱动或独立录包器')
            self.refresh_jobs();return
        if name=='mapping_finish':
            self.mapping_saving=True
            self.mapping_state.setText('正在保存：导出 FAST-LIO2 地图与原始 rosbag')
            self.mapping_save_status.setText('保存状态：正在保存；尚未验证文件或索引')
            self.write_log('保存请求已接收，等待地图、BBS 索引与 rosbag 完整性结果')
            self.refresh_jobs();return
        if name in ('device_sensors','device_manual','session_localization','session_navigation'):
            self.write_log('会话已启动：'+value.get('task',name));self.refresh_live();return
        if name=='session_stop':
            self.write_log('现场会话已结束');self.refresh_live()
            if self.pending_session:
                task,map_id,request_name=self.pending_session;self.pending_session=None
                self._api(request_name,'POST','/api/session/start',self._session_payload(task,map_id))
            return
        if name=='session_stop_all':
            self.write_log('Studio 与 Workbench 管理的 ROS 进程已全部关闭')
            self.refresh_live();self.refresh_jobs();return
        if name in ('camera_start','camera_stop'):
            self.write_log(name+'：'+str(value.get('id',value.get('state','未知'))))
            self.refresh_jobs();return
        if name=='camera_health':
            health=value.get('health') or {}
            self.camera_health_state=dict(value)
            self.camera_health_seen=time.monotonic()
            self.camera_job_state.update(value)
            if value.get('ready'):
                self.camera_status_label.setText('云台：READY · 相机、串口、反馈与 Action 均在线')
            else:
                self.camera_status_label.setText('云台：'+str(value.get('readiness','UNKNOWN'))+' · '+str(value.get('health_error',health.get('last_error','未就绪')))[:120])
            self.write_log('C1 健康检查：'+str(value.get('readiness','UNKNOWN')))
            self._update_global_status();return
        if name in ('camera_capture','camera_move'):
            self.write_log(('拍照任务已提交：' if name=='camera_capture' else '云台移动任务已提交：')+str(value.get('id','')))
            self.capture_status_label.setText('动作执行中：'+str(value.get('tag','')))
            self.refresh_jobs();return
        if name=='record_start':
            self.recorder_id=value['id'];self.recorder_output=value.get('output','')
            self.write_log('独立录包已启动：'+self.recorder_output);self._update_recording();return
        if name=='record_stop':
            self.write_log(f"录包已保存：{value.get('output',self.recorder_output)} · {value.get('state')}")
            self.recorder_id=None;self._update_recording();self.refresh_jobs();return
        if name=='cloud_import':
            self.pending_cloud_id=value['id']
            self.write_log('三维 PCD 已登记：'+value['id']);self.refresh_library();return
        if name=='cloud_preview':
            self.clean_view.set_cloud({'xyz':value.get('points',[]),'source':'已选地图 · 采样预览'})
            self.cloud_info.setText(f"三维预览：{value.get('sampled_points',0)} / {value.get('total_points',0)} 点；全量删除只按 XYZ 边界执行")
            return
        if name=='cloud_clean':
            self.cloud_clean_btn.setEnabled(True)
            self.pending_cloud_id=value['id']
            self.write_log('全量 PCD 清洗完成，新版本：'+value['id']+'；请重新生成二维地图')
            self.refresh_library()
            return
        if name=='cloud_project':
            self.grid_building=False
            self.map_write_status.setText('二维地图已经保存：'+value['id']+' · '+value.get('asset_summary','BBS 状态未知'))
            self.current_map_id=value['id'];self.write_log('二维地图已生成：'+value['id'])
            self.refresh_library();self._load_map(value);self.pages['地图'].click();return
        if name in ('map_diff','map_diff_preview'):
            report=value.get('report') or {};update=value.get('update_map') or {}
            if name=='map_diff_preview' and update.get('id')!=self.diff_review_box.currentData():return
            self.diff_displayed_id=update.get('id')
            counts=report.get('counts') or {}
            self.diff_status_label.setText(
                f"差异完成 · 新障碍 {counts.get('new_obstacle_cells',0)} 格 · "
                f"清除 {counts.get('cleared_obstacle_cells',0)} 格 · "
                f"新增可观测 {counts.get('newly_observed_cells',0)} 格 · "
                f"覆盖损失 {counts.get('coverage_lost_cells',0)} 格 · "
                f"稳定 {counts.get('stable_free_cells',0)+counts.get('stable_occupied_cells',0)} 格 · "
                f"基准 {report.get('base_map_id')} → 候选 {report.get('candidate_map_id')} · "
                f"更新 {update.get('id','')}（{update.get('status','DRAFT')} / {'REVIEW_REQUIRED' if update.get('review_required') else '已审核'}，未自动激活）\n红：新障碍 蓝：清除 绿：新观测 黄：覆盖丢失")
            image=value.get('_overlay_image') or QImage.fromData(base64.b64decode(value.get('overlay_png_b64','')))
            if not image.isNull():
                self.diff_preview_label.setPixmap(QPixmap.fromImage(image).scaled(
                    self.diff_preview_label.size(),Qt.KeepAspectRatio,Qt.SmoothTransformation))
            self.write_log('自动地图差异已生成待审核版本 '+str(update.get('id',''))+'；正式地图未改变')
            if name=='map_diff':self.pending_review_id=update.get('id');self.refresh_library()
            return
        if name=='map_review':
            self.diff_status_label.setText('审核状态：'+value['status']+' · '+value['id']+' · '+value.get('review_reason','')+'；未自动激活')
            self.refresh_library();return
        if name=='map_revise':
            self.map_saving=False
            self.map_write_status.setText('地图已经保存：'+value['id']+' · '+value['status'])
            self.current_map_id=value['id'];self.write_log(f"地图版本已保存：{value['id']} ({value['status']})")
            self.refresh_library();self._load_map(value)
            message=f"版本：{value['id']}\n状态：{value['status']}\n电子围栏：{len(value.get('keepout_polygons_px',[]))} 个（已烘焙进 PGM）"
            if value['status']=='ACCEPTED':
                message+='\n\n这是新地图版本和新 SHA-256。正式导航仍需与该 SHA 匹配的 PHYSICAL_NAVIGATION 报告；旧路线不会自动跨版本执行。'
            QMessageBox.information(self,'地图已保存',message);return
        if name=='map_import':
            self.current_map_id=value['id'];self.refresh_library();self._load_map(value);return
        if name=='route_save':
            self.current_route_id=value['id'];self.write_log('路线与拍照任务已保存：'+value['id'])
            self.refresh_library();return
        if name=='route_run':
            self.write_log('航点任务已提交；拍照由 V3 mission_runtime 顺序执行');self.refresh_live();return
        if name in ('goal','distance','relocalize','stop_goal'):
            self.write_log(name+' 请求已接受');self.refresh_live()

    @staticmethod
    def _indicator(label,text,state='muted'):
        colors={'good':'#65c7a9','warn':'#e8b178','bad':'#ec9c83','muted':'#90a1ae'}
        label.setText(text);label.setStyleSheet('color:'+colors[state])

    def _update_global_status(self):
        if not hasattr(self,'robot_state_label'):return
        if self.demo:
            self._indicator(self.camera_state_label,'云台：演示未连接')
            self._indicator(self.robot_state_label,'设备：'+self.robot_box.currentText())
            self._indicator(self.mode_state_label,'状态：离线演示','warn')
            self._indicator(self.chassis_state_label,'底盘：演示未连接')
            for label,name,value in ((self.lidar_state_label,'雷达',10.),(self.imu_state_label,'IMU',100.),
                                     (self.odom_state_label,'里程计',50.)):
                self._indicator(label,f'{name}：演示 {value:.1f} Hz','good')
            return
        state=self.camera_job_state.get('state','STOPPED')
        health_name=(self.camera_job_state.get('readiness') or '')
        if state=='RUNNING' and health_name=='READY' and time.monotonic()-self.camera_health_seen<5:
            self._indicator(self.camera_state_label,'云台：硬件 READY','good')
        elif state=='RUNNING':
            self._indicator(self.camera_state_label,'云台：进程运行/'+(health_name or '待验'),'warn')
        elif state in ('FAILED','ORPHANED','INCOMPLETE','STOP_FAILED'):
            self._indicator(self.camera_state_label,'云台：'+state,'bad')
        else:self._indicator(self.camera_state_label,'云台：未启动')
        session=self.live_status.get('session') or {}
        if session:
            robot='FW-mini' if session.get('robot')=='fwmini' else 'Bunker'
            task=session.get('task','')
            mode={'sensors':'传感器在线会话','manual':'底盘手动会话','localization':'定位会话','navigation':'自主导航'}.get(task,task or '未知')
            if task in ('localization','navigation'):
                mode+=' · 已定位' if self.live_status.get('localized') else ' · 未定位'
            self._indicator(self.mode_state_label,'状态：'+mode,'good' if task=='navigation' else 'warn')
        else:
            robot=self.robot_box.currentText()
            self._indicator(self.mode_state_label,'状态：未连接')
        self._indicator(self.robot_state_label,'设备：'+robot,'good' if session else 'muted')
        chassis=self.live_status.get('chassis') or {}
        enabled=session.get('task') in ('manual','navigation')
        if not session:self._indicator(self.chassis_state_label,'底盘：未连接')
        elif not enabled:self._indicator(self.chassis_state_label,'底盘：未启用')
        elif chassis.get('online'):self._indicator(self.chassis_state_label,'底盘：通讯在线','good')
        else:self._indicator(self.chassis_state_label,'底盘：无反馈','bad')
        self.chassis_state_label.setToolTip('通讯在线仅表示底盘反馈新鲜，不代表运动许可；状态：'+str(chassis.get('state','未知')))
        for label,name,key in ((self.lidar_state_label,'雷达','lidar'),(self.imu_state_label,'IMU','imu'),
                               (self.odom_state_label,'里程计','odom')):
            rate=self.topic_rates.get(key)
            online=self.ros_display_connected and rate is not None
            self._indicator(label,f'{name}：{"在线" if online else "离线"} {rate:.1f} Hz' if online else f'{name}：离线 -- Hz',
                            'good' if online else 'bad')

    def _update_live(self):
        session=self.live_status.get('session')
        localized=bool(self.live_status.get('localized'))
        self.localization_label.setText('● 已定位' if localized else '● 未定位 / 重定位中')
        self.localization_label.setStyleSheet('color:#65c7a9' if localized else 'color:#e8b178')
        algorithm=(session or {}).get('global_backend','bbs')
        metrics=self.live_status.get('global_relocalization') or {}
        self.localization_label.setText(('● 已定位' if localized else '● 未定位')+' · '+algorithm+
            f" · 耗时 {metrics.get('total_elapsed_sec','--')} s · Fitness {metrics.get('fitness','--')} · Overlap {metrics.get('overlap','--')} · "+str(metrics.get('detail',''))[:180])
        pose=self.live_status.get('pose')
        self.pose_label.setText(f"base_link：X={pose['x']:.2f}  Y={pose['y']:.2f}  Yaw={math.degrees(pose['yaw']):.1f}°" if pose else 'base_link 当前位置：未知')
        mission=self.live_status.get('mission')
        if mission:
            states={0:'空闲',1:'定位中',2:'导航中',3:'停稳确认',4:'拍照中',5:'已暂停',6:'已完成',7:'已取消',8:'失败'}
            self.mission_label.setText(f"任务：{states.get(mission.get('state'),'未知')} · {mission.get('point_id','')} "
                f"{mission.get('point_index',0)}/{mission.get('point_count',0)} · {mission.get('detail','')}")
        if session:
            livox=session.get('livox') or {}
            identity=(f" · {livox.get('model','MID360S')} {livox.get('sensor_ip','--')} → 主机 {livox.get('host_ip','--')}"
                      f" · 序列号{livox.get('serial_number','NOT_QUERIED')}" if livox else '')
            self.device_label.setText(f"设备会话：{session.get('robot')} · {session.get('task')} · {session.get('id')}"+identity)
            self.device_label.setToolTip('配置文件：'+str(livox.get('config_path','未提供'))+'\n配置身份已校验不代表设备在线；序列号需实际查询。')
            backend=session.get('global_backend','bbs')
            if not self.mapping_job_id:
                self.run_label.setText(f"Workbench 会话：{session.get('task')}  ·  {session.get('id')}  ·  {backend}")
        else:self.device_label.setText('设备会话：未连接')
        self._update_global_status()

    def _update_recording(self):
        recording=bool(self.recorder_id)
        self.record_btn.setText('停止并保存录制' if recording else '开始录制')
        self.record_label.setText(('录制中：'+self.recorder_output) if recording else '录制：未启动')

    def _set_page(self,i):
        self.stack.setCurrentIndex(i)
        for j,b in enumerate(self.pages.values()):b.setChecked(i==j)
        if i==2:self.refresh_library()

    def write_log(self,message):
        if not hasattr(self,'log'):return
        self.log.append(f'[{datetime.now().strftime("%H:%M:%S")}] {message}')
        if self.log.document().blockCount()>350:
            cursor=self.log.textCursor();cursor.movePosition(cursor.Start)
            cursor.select(cursor.BlockUnderCursor);cursor.removeSelectedText();cursor.deleteChar()

    def _on_state(self,s):
        online=s.get('connected',False);self.ros_display_connected=online
        self.topic_rates=dict(s.get('rates') or {})
        angles=s.get('gimbal_angles')
        if angles:self.camera_status_label.setText(f"实际云台：航向 {angles['heading']:.1f}° 俯仰 {angles['pitch']:.1f}° 横滚 {angles['roll']:.1f}°")
        filter_state=s.get('self_filter')
        if filter_state:
            mode=filter_state.get('state','UNKNOWN')
            detail=('正常过滤' if mode=='ACTIVE' else '已旁路，请检查 TF / 过滤比例 / 时间窗')
            self.self_filter_label.setText(
                f"自体回波过滤：{detail} ({mode}) · 屏蔽累计 {filter_state.get('removed_points_total',0)} 点")
            self.self_filter_label.setStyleSheet(
                'color:#65c7a9' if mode=='ACTIVE' else 'color:#ec9c83')
        elif self.mapping_job_id:
            self.self_filter_label.setText('自体回波过滤：未收到在线状态；请检查过滤节点是否启动')
            self.self_filter_label.setStyleSheet('color:#ec9c83')
        else:
            self.self_filter_label.setText('自体回波过滤：等待建图任务')
            self.self_filter_label.setStyleSheet('color:#90a1ae')
        self.status.setText('● ROS 已连接' if online else '● ROS 未连接')
        self.status.setStyleSheet('color:#55c9a9' if online else 'color:#ec9c83')
        od=s.get('odom');self.odom_label.setText('base_link X={:.2f} Y={:.2f} Yaw={:.1f}°'.format(*od[:2],math.degrees(od[2])) if od else 'base_link 局部位姿：未收到')
        self.tri_view.set_robot(s.get('map_robot'))
        self.view_status_label.setText(s.get('display_error') or ('实时视图：map 坐标已对齐' if s.get('map_robot') else '实时视图：机器人 TF 不可用，保留地图'))
        rates=s.get('rates') or {}
        hz=lambda name:('--' if rates.get(name) is None else f'{rates[name]:.1f}')
        self.sensor_rate_label.setText(f"MID360S：{hz('lidar')} Hz    IMU：{hz('imu')} Hz    base_link 里程计：{hz('odom')} Hz")
        self._update_global_status()

    def _on_cloud(self,d):
        if 'projections' in d:self.tri_view.set_cloud(d)
        self.cloud_view.set_cloud(d);self.cloud_count.setText(f'{d.get("source")}  ·  {len(d["xyz"]):,} 点（抽样渲染）')
        if not self.mapping_saving and not self.mapping_save_status.text().startswith('保存状态：地图已经保存'):
            self.mapping_state.setText('正在接收建图点云')

    def _on_grid(self,info):
        if not self.current_map_id:self.map_view.set_grid(info)
        if not self.current_map_id:self.nav_view.set_grid(info)
        self.map_info.setText(f'{info["width"]} × {info["height"]}  |  {info["resolution"]} m/cell')

    def _robot(self):return 'fwmini' if self.robot_box.currentText()=='FW-mini' else 'bunker'

    def _session_payload(self,task,map_id=None):
        p={'robot':self._robot(),'task':task}
        for key in ('calibration_bundle','mount_profile','geometry_file','livox_config','safety_report'):
            if self.cfg.get(key):p[key]=self.cfg[key]
        if task in ('manual','navigation'):
            p['checks']={k:v.isChecked() for k,v in self.motion_checks.items()}
        if task in ('localization','navigation'):
            p['global_backend']=self.global_backend_box.currentData()
            if p['global_backend']=='sc_kiss':
                value=self.sc_database_edit.text().strip()
                p['sc_database']=str(Path(value).expanduser().resolve()) if value else ''
            p['lio_backend']=self.lio_backend_box.currentData()
        if task=='navigation':
            p['enable_camera_gimbal']=False
            p['camera_device_path']=self.cfg.get('camera_device_path','/dev/video0')
            p['gimbal_port_name']=self.cfg.get('gimbal_port_name','')
        if map_id:p['map_id']=map_id
        return p

    def connect_sensors(self):
        self._ensure_ros_worker()
        session=self.live_status.get('session') or {}
        if session.get('robot')==self._robot() and session.get('task') in ('sensors','manual'):
            self.write_log('传感器会话已经在运行');return
        self._start_session('sensors',request_name='device_sensors')

    def connect_chassis(self):
        self._ensure_ros_worker()
        session=self.live_status.get('session') or {}
        if session.get('robot')==self._robot() and session.get('task') in ('manual','navigation'):
            self.write_log('底盘会话已经在运行');return
        self._start_session('manual',request_name='device_manual')

    def start_field_session(self,task):
        map_id=self.nav_map_box.currentData()
        if not map_id:QMessageBox.warning(self,'缺少地图','请先在地图页生成并选择带三维 PCD 的地图版本。');return
        if map_id in self.maps_by_id:self._load_map(self.maps_by_id[map_id])
        self.current_map_id=map_id
        self._start_session(task,map_id)

    def _start_session(self,task,map_id=None,request_name=None):
        self._ensure_ros_worker()
        name=request_name or 'session_'+task
        session=self.live_status.get('session') or {}
        if session:
            same_backend=(task not in ('localization','navigation') or
                (session.get('global_backend','bbs')==self.global_backend_box.currentData()
                 and session.get('lio_backend','batch_lio')==self.lio_backend_box.currentData()
                 and (self.global_backend_box.currentData()!='sc_kiss' or
                      session.get('sc_database')==self._session_payload(task,map_id).get('sc_database'))))
            if (session.get('task')==task and session.get('map_id')==map_id
                    and session.get('robot')==self._robot() and same_backend):
                self.write_log('所需现场会话已经在运行');return
        self._api(name,'POST','/api/session/start',self._session_payload(task,map_id))

    def stop_field_session(self):self._api('session_stop','POST','/api/session/stop')

    def relocalize(self):
        self._api('relocalize','POST','/api/control/relocalize',{'robot':self._robot()})

    def stop_goal(self):self._api('stop_goal','POST','/api/control/stop',{'robot':self._robot()})

    def toggle_recording(self):
        if self.demo:self.write_log('演示模式：不启动录包');return
        if self.recorder_id:
            self._api('record_stop','POST',f'/api/jobs/{self.recorder_id}/stop',{})
            return
        if self.mapping_job_id or (self.glim and self.glim.state()!=QProcess.NotRunning):
            QMessageBox.warning(self,'建图正在录包','请先结束建图；建图会话已经由唯一所有者保存原始 rosbag。');return
        try:custom_topics=parse_topics(self.record_topics_edit.toPlainText())
        except ValueError as ex:QMessageBox.warning(self,'录包话题无效',str(ex));return
        payload={'robot':self._robot(),'template':self.record_template_box.currentText(),
                 'label':'AGT Robot Studio','map_version':self.current_map_id}
        if custom_topics:payload['topics']=custom_topics
        if self.cfg.get('mount_profile'):payload['mount_profile']=self.cfg['mount_profile']
        self._api('record_start','POST','/api/recording/start',payload)

    def close_ros_nodes(self):
        if self.demo:self.write_log('演示模式：没有 ROS 进程需要关闭');return
        self.pending_session=None
        self.pending_mapping_start=False
        for process in (self.bag,self.glim,self.rviz,self.editor):self._interrupt(process)
        self.mapping_job_id=None;self.glim_btn.setEnabled(True);self.glim_btn.setText('开始建图')
        self.map_name_edit.setEnabled(True);self.mapping_backend_box.setEnabled(True);self._mapping_backend_changed()
        self.mapping_state.setText('正在请求停止 ROS 进程')
        if self.worker:
            self.worker.shutdown();self.worker.wait(2000);self.worker=None
        self.status.setText('● ROS 未连接');self.status.setStyleSheet('color:#ec9c83')
        self._api('session_stop_all','POST','/api/session/stop_all',{})

    def send_goal(self,point):
        map_id=self.nav_map_box.currentData()
        pose=self.live_status.get('pose') or {};point['yaw']=float(point.get('yaw',pose.get('yaw',0.)))
        if self.target_mode_box.currentData()=='route':
            point.update(photo=self.photo_box.isChecked(),heading=self.gimbal_heading.value(),
                         pitch=self.gimbal_pitch.value(),roll=0.,photo_required=True)
            self.route_points.append(point);self.current_route_id=None
            self.nav_view.set_waypoints(self.route_points)
            self.mission_label.setText(f'已添加 {len(self.route_points)} 个航点；保存路线后开始任务。')
            return
        self._api('goal','POST','/api/control/goal',{'robot':self._robot(),'map_id':map_id,'pose':point})

    def clear_route(self):
        self.route_points=[];self.current_route_id=None;self.nav_view.set_waypoints([])
        self.mission_label.setText('航点已清空。')

    def save_route(self):
        map_id=self.nav_map_box.currentData()
        if not map_id or not self.route_points:
            QMessageBox.warning(self,'缺少路线','请先选择地图并添加至少一个航点。');return
        self._api('route_save','POST','/api/routes',{'map_id':map_id,'points':self.route_points,
                  'name':f'Studio {len(self.route_points)} 点路线','parent':self.current_route_id})

    def load_route(self):
        route=self.routes_by_id.get(self.route_box.currentData())
        if not route:return
        self.current_route_id=route['id'];self.route_points=[dict(p) for p in route['points']]
        self.target_mode_box.setCurrentIndex(self.target_mode_box.findData('route'))
        index=self.nav_map_box.findData(route['map_id'])
        if index>=0:self.nav_map_box.setCurrentIndex(index);self.select_navigation_map()
        self.nav_view.set_waypoints(self.route_points)
        self.mission_label.setText(f"已加载 {len(self.route_points)} 个航点；地图版本必须与路线绑定版本一致。")

    def run_route(self):
        if not self.current_route_id:
            QMessageBox.warning(self,'请先保存路线','保存会固定地图与航点版本，然后再执行任务。');return
        self._api('route_run','POST','/api/control/route',{'robot':self._robot(),
                  'map_id':self.nav_map_box.currentData(),'route_id':self.current_route_id})

    def move_distance(self,sign):
        map_id=self.nav_map_box.currentData()
        self._api('distance','POST','/api/control/distance',
                  {'robot':self._robot(),'map_id':map_id,'distance_m':sign*self.distance.value()})

    def save_parameters(self):
        try:
            topics={key:editor.text().strip() for key,editor in self.topic_edits.items()}
            for topic in topics.values():parse_topics([topic])
            if any(not topic for topic in topics.values()):raise ValueError('显示话题不能为空')
            record_topics=parse_topics(self.record_topics_edit.toPlainText())
            projection={key:editor.value() for key,editor in self.projection_edits.items()}
            projection['occupied_threshold']=self.min_points_box.value()
            if projection['obstacle_min_height']>=projection['obstacle_max_height']:
                raise ValueError('相对地面障碍高度下限必须小于上限')
            self.cfg.update(topics)
            self.cfg.update(record_topics=record_topics,projection=projection,
                record_template=self.record_template_box.currentText(),
                glim_config_path=self.glim_config_edit.text().strip(),
                bag_root=self.bag_root_edit.text().strip() or '~/agt_studio_runs',
                camera_device_path=self.camera_device_edit.text().strip(),
                gimbal_port_name=self.gimbal_port_edit.text().strip(),
                global_backend=self.global_backend_box.currentData(),
                sc_database=self.sc_database_edit.text().strip(),
                lio_backend=self.lio_backend_box.currentData(),
                mapping_backend=self.mapping_backend_box.currentData(),
                save_mapping_data=self.save_mapping_box.isChecked(),
                enable_camera_gimbal=False,
                map_name=self.map_name_edit.text().strip()[:120])
            import yaml
            USER_CONFIG.parent.mkdir(parents=True,exist_ok=True)
            temporary=USER_CONFIG.with_suffix('.yaml.tmp')
            temporary.write_text(yaml.safe_dump(self.cfg,allow_unicode=True,sort_keys=False),encoding='utf8')
            temporary.replace(USER_CONFIG)
            self.parameter_status.setText('已保存：'+str(USER_CONFIG))
            self.write_log('参数已保存到用户配置；设备参数下次会话生效，显示话题重连后生效')
        except (ValueError,OSError) as ex:
            QMessageBox.warning(self,'参数未保存',str(ex))

    def save_bunker_profile(self):
        if not self.bunker_profile:
            QMessageBox.warning(self,'未读取 Bunker 参数','请先读取当前底盘 YAML。');return
        if self.live_status.get('session'):
            QMessageBox.warning(self,'现场会话仍在运行',
                '请先正常结束传感器／手动／导航会话，再保存新版 Bunker 参数。');return
        profile=dict(self.bunker_profile)
        for key,editor in self.bunker_profile_edits.items():
            profile[key]=editor.value()
        result=QMessageBox.question(self,'确认保存 Bunker 参数版本',
            '将创建新的机器人运动参数 YAML，并用于下次启动的 Bunker 会话。'
            '不会立即向电机发命令，也不会改变急停和人工接管。\n\n'
            '请确认这些限速、加速度及超时参数符合现场底盘能力。',
            QMessageBox.Yes|QMessageBox.No,QMessageBox.No)
        if result!=QMessageBox.Yes:return
        self._api('profile_save','POST','/api/profiles/bunker',{'profile':profile})

    def choose_glim_config(self):
        path=QFileDialog.getExistingDirectory(self,'选择 GLIM 配置目录',str(Path.home()))
        if path:
            self.cfg['glim_config_path']=path;self.glim_config_edit.setText(path)
            self.write_log('GLIM 配置：'+path)

    def _mapping_backend_changed(self):
        fast=self.mapping_backend_box.currentData()=='fastlio'
        active=bool(self.mapping_job_id) or bool(self.glim and self.glim.state()!=QProcess.NotRunning)
        self.glim_cfg_btn.setEnabled(not fast and not active)
        self.editor_btn.setEnabled(not fast)
        self.save_mapping_box.setEnabled(not fast and not active)
        self.run_label.setText(
            'FAST-LIO2 无 PGO：当前 Bunker 验收基线。Workbench 复用现有雷达与 TF 会话，并独占原始 rosbag 录制；不启动第二套驱动或录包器。'
            if fast else
            'GLIM：实验功能，依赖、配置和真实回放尚未验收。保存开关仅控制 GLIM dump 与原始 rosbag；不会重复启动雷达或 TF。')

    def _start_mapping_processes(self):
        if self.mapping_backend_box.currentData()=='fastlio':
            if self.mapping_job_id:return
            payload={'robot':self._robot(),'live':True,'backend':'fastlio','map_name':self.current_map_name}
            for key in ('calibration_bundle','mount_profile','livox_config'):
                if self.cfg.get(key):payload[key]=self.cfg[key]
            self.mapping_state.setText('正在启动 FAST-LIO2 无 PGO')
            self._api('mapping_start','POST','/api/mapping/start',payload)
            return
        self._start_glim_processes()

    def toggle_glim(self):
        if self.demo:self.write_log('演示模式禁止启动建图');return
        self._ensure_ros_worker()
        if self.mapping_job_id:
            if self.mapping_saving:return
            self.mapping_saving=True
            self.mapping_state.setText('正在保存 FAST-LIO2 地图与原始 rosbag')
            self.mapping_save_status.setText('保存状态：正在保存；导出／转换／BBS 索引尚未完成')
            self.glim_btn.setEnabled(False)
            self._api('mapping_finish','POST',f'/api/jobs/{self.mapping_job_id}/finish',{})
            return
        if self.glim and self.glim.state()!=QProcess.NotRunning:
            self._interrupt(self.glim);self._interrupt(self.bag)
            self.glim_btn.setText('开始建图');self.map_name_edit.setEnabled(True);self.mapping_backend_box.setEnabled(True)
            self._mapping_backend_changed()
            self.mapping_state.setText('正在保存 GLIM dump 与 rosbag' if self.mapping_save_data else '建图已结束（未启用数据保存）')
            self.write_log(('建图结束，文件保留在 ' if self.mapping_save_data else '建图结束，配置快照位于 ')+str(self.current_run));return
        if self.recorder_id:
            QMessageBox.warning(self,'独立录制正在运行','请先停止并保存独立录制，再启动会自动录包的建图会话。');return
        self.current_map_name=self.map_name_edit.text().strip()[:120]
        if not self.current_map_name:
            QMessageBox.warning(self,'缺少地图名称','请先填写地图名称，再开始建图。');return
        self.map_name_edit.setText(self.current_map_name)
        session=(self.live_status.get('session') or {})
        if session and session.get('task') not in ('sensors','manual'):
            QMessageBox.warning(self,'模式冲突','请先结束定位／导航会话，再开始建图。');return
        if session and session.get('robot')==self._robot():self._start_mapping_processes()
        else:
            self.mapping_state.setText('正在启动所选车辆的传感器会话')
            self._start_session('sensors',request_name='session_sensors')

    def _start_glim_processes(self):
        if self.glim and self.glim.state()!=QProcess.NotRunning:return
        path=os.path.expanduser(str(self.cfg.get('glim_config_path','')))
        if not path or not Path(path).is_dir():
            self.mapping_state.setText('需要 GLIM 配置')
            QMessageBox.warning(self,'需要 GLIM 配置','请选择包含已标定外参的 GLIM 配置目录。');return
        root=Path(str(self.cfg['bag_root'])).expanduser();root.mkdir(parents=True,exist_ok=True)
        self.current_run=root/('studio_'+datetime.now().strftime('%Y%m%d_%H%M%S'));self.current_run.mkdir()
        dump=self.current_run/'glim_dump';bag=self.current_run/'raw_bag'
        save_data=self.save_mapping_box.isChecked();self.mapping_save_data=save_data
        self.current_map_name=self.map_name_edit.text().strip()[:120]
        if not self.current_map_name:
            QMessageBox.warning(self,'缺少地图名称','请先填写地图名称，再开始建图。');return
        self.save_mapping_box.setEnabled(False);self.map_name_edit.setEnabled(False);self.mapping_backend_box.setEnabled(False)
        snapshot={'created_at':datetime.now(timezone.utc).isoformat(),'robot':self._robot(),
                  'map_name':self.current_map_name,'glim_config_path':path,'save_mapping_data':save_data,
                  'dump_path':str(dump) if save_data else None,'raw_bag':str(bag) if save_data else None,
                  'workbench_session':self.live_status.get('session')}
        (self.current_run/'session.json').write_text(json.dumps(snapshot,ensure_ascii=False,indent=2)+'\n')
        topics=self.cfg.get('record_topics') or [self.cfg['raw_lidar_topic'],self.cfg['imu_topic'],
            self.cfg['odom_topic'],'/agt/livox/points','/wheel/odom','/cmd_vel','/mux/cmd_vel',
            '/agt/safety/motion_permitted','/diagnostics','/tf','/tf_static']
        self.bag=(self._proc('ros2',['bag','record','-o',str(bag),*list(dict.fromkeys(topics))],'rosbag2')
                  if save_data else None)
        glim_args=['--ros-args','-p','config_path:='+path]
        if save_data:glim_args+=['-p','dump_path:='+str(dump)]
        self.glim=self._proc('ros2',['run',self.cfg['glim_package'],self.cfg['glim_node'],*glim_args],'GLIM')
        if self.glim:
            self.glim_btn.setText('结束并保存建图');self.mapping_state.setText('GLIM 建图运行中')
            self.run_label.setText(f"地图：{self.current_map_name}  ·  本次会话：{self.current_run}")
        else:
            self.map_name_edit.setEnabled(True);self.mapping_backend_box.setEnabled(True)
            self._mapping_backend_changed();self._interrupt(self.bag)

    def open_map_editor(self):
        path=str(self.current_run/'glim_dump') if self.current_run else ''
        path=QFileDialog.getExistingDirectory(self,'选择 GLIM dump 目录',path or str(Path.home()),QFileDialog.ShowDirsOnly | QFileDialog.DontUseNativeDialog)
        if path:self.editor=self._proc('ros2',['run',self.cfg['glim_package'],self.cfg['glim_editor'],path],'GLIM Map Editor')

    def open_rviz(self):
        if not self.demo:self.rviz=self._proc('rviz2',[],'RViz')

    def import_pcd(self):
        path,_=QFileDialog.getOpenFileName(self,'导入建图三维 PCD',str(self.current_run or Path.home()),'PCD (*.pcd)',options=QFileDialog.DontUseNativeDialog)
        if not path:return
        calibration={'robot':self._robot(),'glim_config_path':self.cfg.get('glim_config_path')}
        bag=str(self.current_run/'raw_bag') if self.current_run else None
        name=self.map_name_edit.text().strip()[:120] or self.current_map_name
        self._api('cloud_import','POST','/api/clouds/import',{'path':path,'calibration':calibration,'bag':bag,'name':name})

    def preview_cloud(self, *_):
        ident=self.cloud_box.currentData()
        if ident:self._api('cloud_preview','GET',f'/api/clouds/{ident}/preview')

    def clean_cloud_roi(self):
        ident=self.cloud_box.currentData()
        if not ident:
            QMessageBox.warning(self,'尚未选择 PCD','先导入并选择三维点云版本。');return
        low=[self.cloud_roi_edits[(axis,'min')].value() for axis in ('x','y','z')]
        high=[self.cloud_roi_edits[(axis,'max')].value() for axis in ('x','y','z')]
        if any(a>=b for a,b in zip(low,high)):
            QMessageBox.warning(self,'ROI 无效','每个坐标轴的下限必须小于上限。');return
        answer=QMessageBox.question(self,'确认清理全量地图',
            f'将从地图坐标系下的整个 PCD 删除盒内所有点：\\nmin={low}\\nmax={high}\\n'
            '原始 PCD 保留，新版本可预览和再次清洗。务必确认没有删掉树干、墙面等定位结构。',
            QMessageBox.Yes|QMessageBox.No,QMessageBox.No)
        if answer!=QMessageBox.Yes:return
        if self.demo:
            self.write_log('演示模式：不执行 PCD 清洗');return
        self.cloud_clean_btn.setEnabled(False)
        self._api('cloud_clean','POST',f'/api/clouds/{ident}/clean',
                  {'operations':[{'type':'delete','min':low,'max':high}]})

    def generate_grid(self):
        cloud=self.cloud_box.currentData()
        if not cloud:QMessageBox.warning(self,'缺少三维地图','请先导入 FAST-LIO2 或 GLIM 导出的 PCD。');return
        projection={key:editor.value() for key,editor in self.projection_edits.items()}
        projection['occupied_threshold']=self.min_points_box.value()
        if projection['obstacle_min_height']>=projection['obstacle_max_height']:
            QMessageBox.warning(self,'投影参数无效','相对地面障碍下限必须小于上限。');return
        name=(self.clouds_by_id.get(cloud) or {}).get('name','')
        if self.grid_building:return
        self.grid_building=True
        self.map_write_status.setText('正在保存：生成二维地图，并为同源 PCD 构建 BBS 索引')
        self._api('cloud_project','POST',f'/api/clouds/{cloud}/project',{'projection':projection,'name':name})

    def refresh_maps_and_select(self,map_id):self.current_map_id=map_id;self.refresh_library()

    def _load_map(self,manifest,pixels=None):
        if pixels is None and self.api and callable(getattr(self.api,'local',None)):
            self.pending_display_map_id=manifest['id'];self.nav_view.navigation_mode=False
            self.api.local('map_loaded',lambda:(manifest,QImage(manifest['pgm']).convertToFormat(QImage.Format_Grayscale8)))
            return
        try:
            if pixels is None:pixels=QImage(manifest['pgm']).convertToFormat(QImage.Format_Grayscale8)
            if pixels.isNull():raise ValueError('地图 PGM 读取失败')
            for view in (self.map_view,self.nav_view):
                view.frame_id='map';view.received=None
                view.image=pixels.copy();view.res=float(manifest['resolution']);view.origin=tuple(manifest['origin'])
                view.set_keepout(manifest.get('keepout_polygons_px',[]));view.scale=1.;view.pan_x=view.pan_y=0.;view.reset_heading();view.update()
            self.current_map_id=manifest['id']
            summary=manifest.get('asset_summary')
            if summary is None:
                source=manifest.get('source_pcd')
                assets=Path(source).parent/'relocalization' if source else None
                summary=('AGT Polar Context（非 SC）：'+('已生成' if assets and (assets/'polar_context.db').is_file() else '未生成')+
                    '；BBS 体素资产：'+('已生成' if assets and (assets/'voxelmaps_coords').is_dir() else '未生成')) if assets else '未关联三维 PCD 与重定位资产'
            self.asset_label.setText('当前地图资产：'+summary)
            title=(manifest.get('name')+'  |  ') if manifest.get('name') else ''
            self.map_info.setText(f"{title}{manifest['id']}  |  {manifest['width']} × {manifest['height']}  |  {manifest['resolution']} m/cell  |  {manifest['status']}  |  围栏 {len(manifest.get('keepout_polygons_px',[]))} 个")
        except Exception as ex:QMessageBox.warning(self,'地图加载失败',str(ex))

    def load_selected_map(self):
        manifest=self.maps_by_id.get(self.map_box.currentData())
        if manifest:self._load_map(manifest)

    def select_navigation_map(self):
        manifest=self.maps_by_id.get(self.nav_map_box.currentData())
        if manifest:self._load_map(manifest)

    def import_pgm(self):
        path,_=QFileDialog.getOpenFileName(self,'导入二维 PGM（旁边必须有 YAML）',str(Path.home()),'PGM (*.pgm)',options=QFileDialog.DontUseNativeDialog)
        if not path:return
        yaml_path=Path(path).with_suffix('.yaml')
        if not yaml_path.is_file():QMessageBox.warning(self,'缺少 YAML','没有配套 YAML 时拒绝猜测分辨率和原点。');return
        try:
            import yaml
            doc=yaml.safe_load(yaml_path.read_text())
            if 'resolution' not in doc or len(doc.get('origin',[]))!=3:raise ValueError('YAML 必须包含 resolution 和 origin[x,y,yaw]')
            self.map_view.load_pgm(path);self.map_view.res=float(doc['resolution']);self.map_view.origin=tuple(doc['origin'])
            self.current_map_id=None;self.map_info.setText('外部地图已载入；保存后生成新的独立版本')
        except Exception as ex:QMessageBox.warning(self,'地图读取失败',str(ex))

    def export_map(self,status='DRAFT'):
        try:
            if self.map_view.image.isNull():
                QMessageBox.warning(self,'尚未加载地图','请先选择并加载一个二维地图版本。')
                return
            if not self.current_map_id and status=='ACCEPTED':
                QMessageBox.warning(self,'未登记地图',
                    '外部 PGM 尚未登记为 Workbench 地图版本，不能直接标记为已验收导航地图。')
                return
            if self.current_map_id:
                if self.map_saving:return
                pixels=base64.b64encode(self.map_view.pixels().tobytes()).decode('ascii')
                self.map_saving=True
                self.map_write_status.setText('正在保存：写入新版 PGM、YAML、围栏与 SHA')
                self._api('map_revise','POST',f'/api/maps/{self.current_map_id}/revise',
                          {'pixels':pixels,'status':status,'keepout_polygons_px':self.map_view.keepout});return
            directory=QFileDialog.getExistingDirectory(self,'选择新地图的父目录',str(Path.home()),QFileDialog.ShowDirsOnly | QFileDialog.DontUseNativeDialog)
            if not directory:return
            output=Path(directory)/('map_v'+datetime.now().strftime('%Y%m%d_%H%M%S'))
            path=save_nav2_map(output,self.map_view.pixels(),self.map_view.res,self.map_view.origin)
            self.write_log('独立二维地图已保存：'+path);QMessageBox.information(self,'地图已保存',path)
        except Exception as ex:QMessageBox.warning(self,'导出失败',str(ex))

    def _proc(self,command,args,name):
        p=QProcess(self);p.setProgram(command);p.setArguments(args);p.setProcessChannelMode(QProcess.MergedChannels)
        p.readyReadStandardOutput.connect(lambda:self._proc_output(p,name))
        p.errorOccurred.connect(lambda _:self.write_log(f'{name} 启动错误：{p.errorString()}'))
        p.finished.connect(lambda code,_:self.write_log(f'{name} 已退出，状态码 {code}'));p.start()
        if not p.waitForStarted(1600):self.write_log(f'无法启动 {name}: {p.errorString()}');return None
        self.write_log(f'{name} 已启动 (PID {p.processId()})');return p

    def _proc_output(self,p,name):
        if not p:return
        message=bytes(p.readAllStandardOutput()).decode('utf8','replace').strip()
        if message:self.write_log(name+': '+message[-500:])

    @staticmethod
    def _interrupt(process):
        if process and process.state()!=QProcess.NotRunning:
            import signal
            try:os.kill(int(process.processId()),signal.SIGINT)
            except (OSError,ValueError):process.terminate()

    def _poll(self):
        if (not self.mapping_job_id and self.glim and self.glim.state()==QProcess.NotRunning
                and self.glim_btn.text()!='开始建图'):
            self.glim_btn.setText('开始建图');self.map_name_edit.setEnabled(True);self.mapping_backend_box.setEnabled(True)
            self._mapping_backend_changed()
            self.mapping_state.setText('建图已结束，文件已保留' if self.mapping_save_data else '建图已结束（未启用数据保存）')
        if not self.demo:
            self.refresh_live();self.refresh_jobs()
            if not self.components_in_flight:
                self.components_in_flight=True;self._api('refresh_components','GET','/api/components')
            if self.camera_job_state.get('state')=='RUNNING' and not self.camera_health_pending:
                self.camera_health_pending=True;self.check_camera_health()

    def save_logs(self):
        path,_=QFileDialog.getSaveFileName(self,'保存日志',str(Path.home()/'agt_studio.log'),'日志 (*.log *.txt)',options=QFileDialog.DontUseNativeDialog)
        if path:Path(path).write_text(self.log.toPlainText()+'\n',encoding='utf8')

    def _setup_demo(self):
        x=np.linspace(-14,14,150)
        floor=np.column_stack([np.tile(x,40),np.repeat(np.linspace(-7,7,40),len(x)),np.zeros(len(x)*40)])
        z=np.linspace(0,3.2,65);columns=[]
        for xx in np.arange(-12,13,6):
            for yy in (-5.6,5.6):columns.append(np.column_stack([np.full_like(z,xx),np.full_like(z,yy),z]))
        self.cloud_view.set_cloud({'xyz':np.vstack([floor,*columns]),'source':'合成点云 DEMO'})
        self.cloud_count.setText('DEMO 合成点云 · 未连接 MID360')
        h,w=150,200;cells=np.zeros((h,w),dtype=np.int8)
        cells[:8,:]=100;cells[-8:,:]=100;cells[:,20:25]=100;cells[:,172:178]=100;cells[35:115,88:100]=-1
        self.map_view.set_grid({'width':w,'height':h,'data':cells.ravel(),'resolution':.05,'origin':(-5.,-3.75,0.)})
        self.nav_view.set_grid({'width':w,'height':h,'data':cells.ravel(),'resolution':.05,'origin':(-5.,-3.75,0.)})
        self.status.setText('● 演示模式');self.status.setStyleSheet('color:#eac18a')
        self.mapping_state.setText('离线演示：不会启动设备或控制底盘')
        self.sensor_rate_label.setText('MID360S：演示数据    IMU：演示数据    base_link 里程计：演示数据')
        self.asset_label.setText('当前地图资产：演示地图；AGT Polar Context 不是 Scan Context（SC）。')
        self._update_global_status()

    def closeEvent(self,event):
        critical={'mapping_finish','cloud_clean','cloud_project','cloud_import','map_revise'}
        # A queued request also owns a pending write; checking only the
        # worker's currently active HTTP call misses this shutdown race.
        pending=bool(self.api and callable(getattr(self.api,'has_pending',None))
                     and self.api.has_pending(critical))
        if self.mapping_job_id or self.mapping_saving or self.map_saving or self.grid_building or pending or (self.api and getattr(self.api,'active_request',None) in critical):
            QMessageBox.warning(self,'后台保存尚未结束',
                'FAST-LIO2 建图或地图资产保存仍在运行。请先等任务完成，再关闭界面；'
                '当前关闭可能中断 Workbench 并留下不完整地图。')
            event.ignore();return
        if self.worker:self.worker.shutdown();self.worker.wait(2000)
        for p in (self.bag,self.glim,self.rviz,self.editor):
            self._interrupt(p)
            if p and p.state()!=QProcess.NotRunning:
                if not p.waitForFinished(1800):p.kill();p.waitForFinished(700)
        if self.api and callable(getattr(self.api,'shutdown',None)):
            self.api.shutdown()
        if self.control_api:self.control_api.shutdown()
        if self.backend and self.backend.state()!=QProcess.NotRunning:
            self.backend.terminate();self.backend.waitForFinished(1800)
        event.accept()


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--config',default='')
    parser.add_argument('--demo',action='store_true',help='Synthetic UI only; no ROS or motion')
    args=parser.parse_args();cfg=dict(DEFAULT_CONFIG)
    import yaml
    if args.config:
        with open(os.path.expanduser(args.config),encoding='utf8') as f:x=yaml.safe_load(f)
        if not isinstance(x,dict):parser.error('config YAML should be a mapping')
        cfg.update(x)
    if USER_CONFIG.is_file():
        saved=yaml.safe_load(USER_CONFIG.read_text(encoding='utf8'))
        if not isinstance(saved,dict):parser.error('user config YAML should be a mapping')
        cfg.update(saved)
    QApplication.setAttribute(Qt.AA_EnableHighDpiScaling,True)
    QApplication.setAttribute(Qt.AA_UseHighDpiPixmaps,True)
    app=QApplication(sys.argv[:1]);app.setStyle('Fusion')
    window=Studio(cfg,args.demo);window.show();return app.exec_()


if __name__=='__main__':sys.exit(main())
