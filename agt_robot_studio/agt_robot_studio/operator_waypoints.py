"""Editable route drafts for operators; this widget never sends device commands."""
from copy import deepcopy
import math

from PyQt5.QtCore import Qt, QSize, pyqtSignal
from PyQt5.QtWidgets import (QCheckBox, QDoubleSpinBox, QFormLayout, QHBoxLayout,
                            QLabel, QListWidget, QListWidgetItem, QPushButton,
                            QScrollArea, QVBoxLayout, QWidget)


def _number(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(name + ' must be finite')
    return float(value)


def _angle(value, name):
    number = _number(value, name)
    limit = {'heading': 160.0, 'pitch': 90.0, 'roll': 40.0}[name]
    if abs(number) > limit:
        raise ValueError(name + ' is outside the gimbal range')
    return number


def _display_yaw(radians):
    return (math.degrees(radians) + 180.0) % 360.0 - 180.0


class WaypointEditor(QWidget):
    """Maintain a private route draft and emit copies after user edits.

    ``set_points`` only loads data and never emits ``points_changed``. Field edits,
    reorder/delete actions and ``apply_gimbal_angles`` emit one updated snapshot.
    The owner must invalidate its saved route id after this signal and explicitly
    save/execute through the existing backend. x/y use metres, yaw uses radians in
    snapshots; gimbal heading/pitch/roll use degrees. Extra source fields survive
    edits, but this component never invents fields or waypoint ids.
    """

    points_changed = pyqtSignal(list)
    selection_changed = pyqtSignal(int)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setObjectName('waypointEditor')
        self._points = []
        self._loading = False
        self._editable = True
        self._locked_reason = ''

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(8)
        self.title_label = QLabel('航点草稿 · 0 个')
        self.title_label.setObjectName('h2')
        layout.addWidget(self.title_label)
        self.hint_label = QLabel('在地图上添加航点，然后在这里调整顺序和拍照角度。')
        self.hint_label.setWordWrap(True)
        layout.addWidget(self.hint_label)

        self.point_list = QListWidget()
        self.point_list.setAccessibleName('航点列表')
        self.point_list.setTextElideMode(Qt.ElideNone)
        self.point_list.setWordWrap(True)
        self.point_list.setMinimumHeight(100)
        self.point_list.setMaximumHeight(190)
        self.point_list.currentRowChanged.connect(self._selection_changed)
        layout.addWidget(self.point_list)

        controls = QHBoxLayout()
        self.up_button = QPushButton('上移')
        self.down_button = QPushButton('下移')
        self.delete_button = QPushButton('删除此点')
        self.up_button.clicked.connect(lambda: self.move_selected(-1))
        self.down_button.clicked.connect(lambda: self.move_selected(1))
        self.delete_button.clicked.connect(self.delete_selected)
        for button in (self.up_button, self.down_button, self.delete_button):
            button.setMinimumHeight(34)
            controls.addWidget(button)
        layout.addLayout(controls)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QScrollArea.NoFrame)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.fields_widget = QWidget()
        form = QFormLayout(self.fields_widget)
        form.setContentsMargins(0, 3, 0, 0)
        form.setFieldGrowthPolicy(QFormLayout.AllNonFixedFieldsGrow)
        self.x_spin = self._spin(-1e12, 1e12, 3, .1, ' m', '航点 X，米')
        self.y_spin = self._spin(-1e12, 1e12, 3, .1, ' m', '航点 Y，米')
        self.yaw_spin = self._spin(-180, 180, 1, 5., '°', '车体朝向，度')
        self.heading_spin = self._spin(-160, 160, 1, 1., '°', '此点云台航向，度')
        self.pitch_spin = self._spin(-90, 90, 1, 1., '°', '此点云台俯仰，度')
        self.photo_check = QCheckBox('到点拍照')
        self.required_check = QCheckBox('拍照失败则停止任务')
        for label, widget in (('X', self.x_spin), ('Y', self.y_spin),
                              ('车体朝向', self.yaw_spin), ('', self.photo_check),
                              ('云台航向', self.heading_spin), ('云台俯仰', self.pitch_spin),
                              ('', self.required_check)):
            form.addRow(label, widget)
        self.roll_label = QLabel('横滚保持原值：0.0°')
        self.roll_label.setWordWrap(True)
        form.addRow('', self.roll_label)
        scroll.setWidget(self.fields_widget)
        layout.addWidget(scroll, 1)

        self.draft_label = QLabel('修改只影响路线草稿，不会移动机器人或云台。')
        self.draft_label.setWordWrap(True)
        layout.addWidget(self.draft_label)
        for key, spin in (('x', self.x_spin), ('y', self.y_spin), ('yaw', self.yaw_spin),
                          ('heading', self.heading_spin), ('pitch', self.pitch_spin)):
            spin.valueChanged.connect(lambda value, name=key: self._field_changed(name, value))
        self.photo_check.toggled.connect(lambda value: self._field_changed('photo', value))
        self.required_check.toggled.connect(lambda value: self._field_changed('photo_required', value))
        self._refresh_controls()

    @staticmethod
    def _spin(low, high, decimals, step, suffix, accessible_name):
        spin = QDoubleSpinBox()
        spin.setRange(low, high)
        spin.setDecimals(decimals)
        spin.setSingleStep(step)
        spin.setSuffix(suffix)
        spin.setKeyboardTracking(False)
        spin.setAccessibleName(accessible_name)
        spin.setMinimumHeight(32)
        return spin

    @property
    def selected_index(self):
        return self.point_list.currentRow()

    def points(self):
        """Return an isolated copy so callers cannot silently change this draft."""
        return deepcopy(self._points)

    def set_points(self, points, selected_index=None):
        """Load a route without sending a change event or changing its values."""
        if not isinstance(points, (list, tuple)):
            raise ValueError('waypoints must be a sequence')
        replacement = []
        for point in points:
            if not isinstance(point, dict):
                raise ValueError('waypoint must be an object')
            for name in ('x', 'y', 'yaw'):
                _number(point.get(name), name)
            for name in ('heading', 'pitch', 'roll'):
                _angle(point.get(name, 0.0), name)
            replacement.append(deepcopy(point))
        old_index = self.selected_index
        if selected_index is None:
            selected_index = min(max(old_index, 0), len(replacement) - 1)
        elif not isinstance(selected_index, int) or not -1 <= selected_index < len(replacement):
            if selected_index != -1:
                raise ValueError('selected waypoint index is outside the route')
        self._points = replacement
        self._rebuild_list(selected_index)
        self.draft_label.setText('修改只影响路线草稿，不会移动机器人或云台。')

    def select_point(self, index):
        if not isinstance(index, int) or not -1 <= index < len(self._points):
            raise ValueError('selected waypoint index is outside the route')
        self.point_list.setCurrentRow(index)

    def set_editable(self, editable, reason=''):
        """Keep selection available while preventing edits to an executing draft."""
        self._editable = bool(editable)
        self._locked_reason = str(reason)
        self._refresh_controls()

    def _item_text(self, index):
        point = self._points[index]
        action = '拍照' if point.get('photo', False) else '仅到达'
        return (f'P{index + 1:03d}  ·  {action}\n'
                f"X {point['x']:.2f}  Y {point['y']:.2f}  ·  {_display_yaw(point['yaw']):.1f}°")

    def _rebuild_list(self, selected_index):
        self._loading = True
        self.point_list.blockSignals(True)
        try:
            self.point_list.clear()
            for index in range(len(self._points)):
                item = QListWidgetItem(self._item_text(index))
                item.setSizeHint(QSize(0, 2 * self.point_list.fontMetrics().lineSpacing() + 18))
                self.point_list.addItem(item)
            self.point_list.setCurrentRow(selected_index)
        finally:
            self.point_list.blockSignals(False)
            self._loading = False
        self.title_label.setText(f'航点草稿 · {len(self._points)} 个')
        self._selection_changed(self.selected_index)

    def _selection_changed(self, index):
        self._loading = True
        try:
            point = self._points[index] if 0 <= index < len(self._points) else None
            self.x_spin.setValue(point['x'] if point else 0.0)
            self.y_spin.setValue(point['y'] if point else 0.0)
            self.yaw_spin.setValue(_display_yaw(point['yaw']) if point else 0.0)
            self.heading_spin.setValue(point.get('heading', 0.0) if point else 0.0)
            self.pitch_spin.setValue(point.get('pitch', 0.0) if point else 0.0)
            self.photo_check.setChecked(bool(point.get('photo', False)) if point else False)
            self.required_check.setChecked(bool(point.get('photo_required', True)) if point else True)
            self.roll_label.setText(f"横滚保持原值：{point.get('roll', 0.):.1f}°" if point else '横滚保持原值：--')
        finally:
            self._loading = False
        self._refresh_controls()
        self.selection_changed.emit(index)

    def _refresh_controls(self):
        index = self.selected_index
        selected = 0 <= index < len(self._points)
        can_edit = self._editable and selected
        self.fields_widget.setEnabled(can_edit)
        self.up_button.setEnabled(can_edit and index > 0)
        self.down_button.setEnabled(can_edit and index < len(self._points) - 1)
        self.delete_button.setEnabled(can_edit)
        self.required_check.setEnabled(can_edit and self.photo_check.isChecked())
        if not self._editable:
            self.hint_label.setText(self._locked_reason or '执行中不能修改当前路线；可选择航点查看。')
        elif not self._points:
            self.hint_label.setText('尚无航点：在地图上选择位置和朝向，添加到路线草稿。')
        elif not selected:
            self.hint_label.setText('请在列表中选择要编辑的航点。')
        else:
            self.hint_label.setText(f'正在编辑 P{index + 1:03d}；修改后请重新保存路线。')

    def _field_changed(self, name, value):
        index = self.selected_index
        if self._loading or not self._editable or not 0 <= index < len(self._points):
            return
        if name == 'yaw':
            value = math.radians(value)
        if self._points[index].get(name) == value:
            return
        self._points[index][name] = value
        self.point_list.item(index).setText(self._item_text(index))
        self._refresh_controls()
        self._emit_change()

    def _emit_change(self):
        self.draft_label.setText('草稿已修改 · 保存路线后才可执行。')
        self.points_changed.emit(self.points())

    def move_selected(self, offset):
        index = self.selected_index
        if not self._editable or offset not in (-1, 1) or not 0 <= index < len(self._points):
            return False
        target = index + offset
        if not 0 <= target < len(self._points):
            return False
        self._points[index], self._points[target] = self._points[target], self._points[index]
        self._rebuild_list(target)
        self._emit_change()
        return True

    def delete_selected(self):
        index = self.selected_index
        if not self._editable or not 0 <= index < len(self._points):
            return False
        self._points.pop(index)
        self._rebuild_list(min(index, len(self._points) - 1))
        self._emit_change()
        return True

    def apply_gimbal_angles(self, angles):
        """Copy fresh measured angles into the selected draft; never move hardware.

        The caller owns freshness validation. Missing axes retain their original
        values. Invalid values are rejected atomically and emit no update.
        """
        index = self.selected_index
        if not self._editable or not 0 <= index < len(self._points):
            return False
        if not isinstance(angles, dict) or not any(name in angles for name in ('heading', 'pitch', 'roll')):
            raise ValueError('measured gimbal angles are missing')
        values = {name: _angle(angles[name], name) for name in ('heading', 'pitch', 'roll') if name in angles}
        if all(self._points[index].get(name, 0.0) == value for name, value in values.items()):
            return False
        self._points[index].update(values)
        self._selection_changed(index)
        self._emit_change()
        return True
