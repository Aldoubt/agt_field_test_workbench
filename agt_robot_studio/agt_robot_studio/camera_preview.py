"""Read-only camera decoding and bounded, age-aware Qt preview.

``decode_image`` runs in RosWorker's background decoder. It returns an owned
QImage, never a view of ROS-owned memory. Widgets only render completed frames.
No driver, capture Action or robot command is created in this module.
"""
import math
import time

import numpy as np
from PyQt5.QtCore import Qt, QRectF, QTimer
from PyQt5.QtGui import QColor, QImage, QPainter
from PyQt5.QtWidgets import QLabel, QVBoxLayout, QWidget


def decode_image(msg, *, received=None, topic='', max_width=960, max_height=540):
    """Decode common uint8 sensor_msgs/Image layouts, respecting row padding."""
    width, height, step = int(msg.width), int(msg.height), int(msg.step)
    formats = {'rgb8': (3, QImage.Format_RGB888),
               'bgr8': (3, QImage.Format_RGB888),
               'mono8': (1, QImage.Format_Grayscale8),
               'rgba8': (4, QImage.Format_RGBA8888),
               'bgra8': (4, QImage.Format_RGBA8888)}
    encoding = str(msg.encoding).lower()
    if encoding not in formats:
        raise ValueError('不支持的图像编码 ' + encoding + '；需要 rgb8/bgr8/mono8/rgba8/bgra8')
    channels, image_format = formats[encoding]
    if not 0 < width * height <= 16_000_000 or width <= 0 or height <= 0:
        raise ValueError('图像尺寸无效或超过预览上限')
    if step < width * channels or step * height > 64 * 1024 * 1024:
        raise ValueError('图像行跨度 step 无效')
    try:
        raw = memoryview(msg.data)
        if raw.nbytes != step * height:
            raise ValueError('图像数据长度与 height × step 不匹配')
        rows = np.frombuffer(raw, dtype=np.uint8).reshape(height, step)
    except (TypeError, BufferError) as exc:
        raise ValueError('图像数据不是连续字节缓冲区') from exc
    pixels = rows[:, :width * channels].reshape(height, width, channels)
    if encoding in ('bgr8', 'bgra8'):
        pixels = pixels[:, :, [2, 1, 0] + ([3] if channels == 4 else [])]
    pixels = np.ascontiguousarray(pixels)
    image = QImage(pixels.data, width, height, width * channels, image_format).copy()
    if image.isNull():
        raise ValueError('图像转换失败')
    if width > max_width or height > max_height:
        image = image.scaled(max_width, max_height, Qt.KeepAspectRatio, Qt.FastTransformation)
    timestamp = msg.header.stamp
    stamp = float(timestamp.sec) + float(timestamp.nanosec) / 1e9
    if not math.isfinite(stamp) or timestamp.sec < 0 or not 0 <= timestamp.nanosec < 1_000_000_000:
        raise ValueError('图像时间戳无效')
    return dict(image=image, received=time.monotonic() if received is None else float(received),
                stamp_sec=stamp, frame_id=str(msg.header.frame_id), topic=str(topic),
                width=width, height=height, encoding=encoding)


class _ImageCanvas(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.image = None
        self.overlay = '等待相机图像'
        self.colors = {'background': QColor('#172226'), 'text': QColor('#f4f6ef'),
                       'overlay': QColor(12, 20, 22, 145)}
        self.setMinimumSize(180, 120)

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.fillRect(self.rect(), self.colors['background'])
        if self.image is not None and not self.image.isNull():
            size = self.image.size()
            size.scale(self.size(), Qt.KeepAspectRatio)
            target = QRectF((self.width()-size.width())/2, (self.height()-size.height())/2,
                            size.width(), size.height())
            painter.setRenderHint(QPainter.SmoothPixmapTransform)
            painter.drawImage(target, self.image)
        if self.overlay:
            if self.image is not None:
                painter.fillRect(self.rect(), self.colors['overlay'])
            painter.setPen(self.colors['text'])
            painter.drawText(self.rect().adjusted(12, 8, -12, -8),
                             Qt.AlignCenter | Qt.TextWordWrap, self.overlay)
        painter.end()


class CameraPreview(QWidget):
    """Show only verified frames; retain the last frame with a visible stale mask."""
    def __init__(self, parent=None, *, topic='/cv_camera0/image_raw', timeout_sec=2.0):
        super().__init__(parent)
        self.topic = topic
        self.timeout_sec = max(.1, float(timeout_sec))
        self.frame = None
        self.connected = None
        self.error = None
        self.preview_state = 'WAITING'
        self.status_colors = {'good': '#24664e', 'warn': '#8c4b24'}
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(6)
        self.canvas = _ImageCanvas(self)
        layout.addWidget(self.canvas, 1)
        self.status_label = QLabel('尚未收到有效图像')
        self.status_label.setWordWrap(True)
        self.status_label.setObjectName('camera_preview_status')
        layout.addWidget(self.status_label)
        self.detail_label = QLabel(topic)
        self.detail_label.setWordWrap(True)
        self.detail_label.setObjectName('muted')
        layout.addWidget(self.detail_label)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.refresh_staleness)
        self.timer.start(250)

    def set_palette(self, colors):
        """Apply canvas colors without coupling the preview to a UI stylesheet.

        Keys: background, text, overlay; optional good/warn status text colors.
        QColor values retain alpha; opaque overlay colors use a translucent mask.
        """
        for key in self.canvas.colors:
            if key not in colors:
                continue
            color = QColor(colors[key])
            if color.isValid():
                if key == 'overlay' and color.alpha() == 255:
                    color.setAlpha(145)
                self.canvas.colors[key] = color
        for key in self.status_colors:
            if key in colors and QColor(colors[key]).isValid():
                self.status_colors[key] = QColor(colors[key]).name()
        self.refresh_staleness()

    def set_frame(self, payload):
        if not isinstance(payload, dict) or not isinstance(payload.get('image'), QImage):
            self.set_error('预览帧格式无效')
            return
        received = payload.get('received')
        if payload['image'].isNull() or not isinstance(received, (int, float)) or not math.isfinite(received):
            self.set_error('预览帧为空或时间无效')
            return
        source_age = payload.get('source_age_s', 0.)
        if not isinstance(source_age, (int, float)) or not math.isfinite(source_age) or source_age < 0:
            self.set_error('预览帧源时间龄无效')
            return
        self.frame = payload
        self.canvas.image = payload['image']
        self.error = None
        if self.connected is None:
            self.connected = True
        self.topic = payload.get('topic') or self.topic
        self.detail_label.setText(f"{self.topic} · {payload.get('width', payload['image'].width())} × "
                                  f"{payload.get('height', payload['image'].height())} · {payload.get('encoding', '')}")
        self.refresh_staleness()

    def set_connection(self, connected):
        self.connected = bool(connected)
        self.refresh_staleness()

    def set_error(self, error):
        self.error = str(error) if error else None
        self.refresh_staleness()

    def refresh_staleness(self, now=None):
        now = time.monotonic() if now is None else float(now)
        age = (max(0.0, now-self.frame['received']) + max(0.0, self.frame.get('source_age_s', 0.0))) if self.frame else None
        if self.connected is False:
            self.preview_state = 'DISCONNECTED'
            self.canvas.overlay = '显示适配器未连接' + ('\n保留最后有效画面' if self.frame else '')
            text = '图像未连接；请检查 ROS 显示适配器'
            if age is not None:
                text += f' · 最后有效帧 {age:.1f} 秒前'
        elif self.error:
            self.preview_state = 'ERROR'
            self.canvas.overlay = '图像异常' + ('\n保留最后有效画面' if self.frame else '')
            text = self.error
            if age is not None:
                text += f' · 最后有效帧 {age:.1f} 秒前'
        elif self.frame is None:
            self.preview_state = 'WAITING'
            self.canvas.overlay = '等待相机图像\n' + self.topic
            text = '尚未收到有效图像；请启动相机并核对话题'
        elif age > self.timeout_sec:
            self.preview_state = 'STALE'
            self.canvas.overlay = f'画面已过期 {age:.1f} 秒\n保留最后有效画面'
            text = f'图像过期 · 距最后有效帧 {age:.1f} 秒'
        else:
            self.preview_state = 'LIVE'
            self.canvas.overlay = ''
            text = f'实时图像 · 距最后有效帧 {age:.1f} 秒'
        self.status_label.setText(text)
        self.status_label.setStyleSheet('color:'+self.status_colors['good' if self.preview_state == 'LIVE' else 'warn'])
        self.canvas.update()
