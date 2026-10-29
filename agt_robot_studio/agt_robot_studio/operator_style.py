"""Local Qt themes: presentation only, with shared chrome and canvas tokens."""
import math
import re
from string import Template


THEMES = {
    'field': {'name': '田野浅色', 'description': '浅灰绿与白色面板，适合室内调试和日常操作。'},
    'industrial': {'name': '工业深色', 'description': '深蓝灰与青绿强调色，适合实验室和低照度环境。'},
    'daylight': {'name': '户外高对比', 'description': '白底深墨与蓝色操作键，较大点击区域便于现场使用。'},
}

# All themes share one selector template; only presentation tokens vary.
_BASE = {
    'font_size': 13, 'small_font_size': 12, 'title_size': 20, 'heading_size': 16,
    'number_size': 24, 'radius': 8, 'card_radius': 12, 'control_radius': 6,
    'button_height': 18, 'button_pad_y': 8, 'button_pad_x': 14,
    'control_height': 20, 'control_padding': 6, 'indicator_size': 18,
    'tab_padding': 10, 'nav_padding': 12, 'scrollbar_width': 12,
    'font_weight': 500, 'border_width': 1,
}

_TOKENS = {
    'field': {
        'background': '#f3f5ef', 'surface': '#ffffff', 'surface_alt': '#edf1e7',
        'sidebar': '#e8eee2', 'text': '#233029', 'muted': '#53634e',
        'title': '#18261d', 'border': '#bdcbb4', 'border_subtle': '#d4ddcc',
        'input_background': '#ffffff', 'input_text': '#22301e',
        'button_background': '#ffffff', 'button_text': '#2c4128',
        'hover_background': '#eaf3dc', 'hover_border': '#779653',
        'pressed_background': '#dbe9bf', 'primary': '#4b741e',
        'primary_hover': '#3c6116', 'primary_pressed': '#2f4e12',
        'primary_text': '#ffffff', 'focus': '#47791e',
        'selection_background': '#dcebc7', 'selection_text': '#223c16',
        'nav_text': '#485b40', 'nav_selected': '#ffffff', 'nav_accent': '#548521',
        'disabled_background': '#e7ecdf', 'disabled_text': '#65715e',
        'disabled_border': '#ced7c5', 'danger_background': '#fff2ef',
        'danger_hover': '#ffe2da', 'danger_border': '#bc493d',
        'danger_text': '#9b241b', 'good': '#35651a', 'warn': '#865600',
        'bad': '#aa2c24', 'tooltip_background': '#263821',
        'tooltip_text': '#ffffff', 'scrollbar': '#a2b194',
    },
    'industrial': {
        'background': '#111b24', 'surface': '#1c2a36', 'surface_alt': '#243543',
        'sidebar': '#15222d', 'text': '#e9f1f5', 'muted': '#afc0cc',
        'title': '#f6fbff', 'border': '#5b7487', 'border_subtle': '#3d5264',
        'input_background': '#14232e', 'input_text': '#f0f7fb',
        'button_background': '#283d4c', 'button_text': '#eaf5fa',
        'hover_background': '#344e5f', 'hover_border': '#86b4c4',
        'pressed_background': '#3b5f70', 'primary': '#58d1b5',
        'primary_hover': '#7ee3cc', 'primary_pressed': '#38b69b',
        'primary_text': '#092e27', 'focus': '#6ddfc5',
        'selection_background': '#235e5e', 'selection_text': '#f3ffff',
        'nav_text': '#bccbd5', 'nav_selected': '#28413e', 'nav_accent': '#6ddfc5',
        'disabled_background': '#202e39', 'disabled_text': '#8499a8',
        'disabled_border': '#3a4c59', 'danger_background': '#3b2931',
        'danger_hover': '#51323b', 'danger_border': '#e58279',
        'danger_text': '#ffb2a8', 'good': '#8bdeb3', 'warn': '#f8cc77',
        'bad': '#ffaaa0', 'tooltip_background': '#e4f1f5',
        'tooltip_text': '#152934', 'scrollbar': '#597589',
    },
    'daylight': {
        'background': '#ffffff', 'surface': '#ffffff', 'surface_alt': '#eef2f6',
        'sidebar': '#edf2f7', 'text': '#101c2b', 'muted': '#33465e',
        'title': '#071323', 'border': '#63748b', 'border_subtle': '#98a7b8',
        'input_background': '#ffffff', 'input_text': '#091929',
        'button_background': '#ffffff', 'button_text': '#132b48',
        'hover_background': '#e2edff', 'hover_border': '#174e9c',
        'pressed_background': '#c9dcfa', 'primary': '#154f9e',
        'primary_hover': '#103d7d', 'primary_pressed': '#082c61',
        'primary_text': '#ffffff', 'focus': '#074bbb',
        'selection_background': '#c9dfff', 'selection_text': '#092449',
        'nav_text': '#243f5b', 'nav_selected': '#ffffff', 'nav_accent': '#154f9e',
        'disabled_background': '#e8edf2', 'disabled_text': '#58677a',
        'disabled_border': '#a1acba', 'danger_background': '#fff2ef',
        'danger_hover': '#ffddd5', 'danger_border': '#a52d20',
        'danger_text': '#8e1e15', 'good': '#185b2e', 'warn': '#784400',
        'bad': '#a12119', 'tooltip_background': '#102844',
        'tooltip_text': '#ffffff', 'scrollbar': '#61768f',
        'font_size': 14, 'small_font_size': 13, 'title_size': 21,
        'heading_size': 17, 'font_weight': 600, 'border_width': 2,
        'button_height': 22, 'button_pad_y': 10, 'button_pad_x': 16,
        'control_height': 24, 'control_padding': 7, 'indicator_size': 22,
        'tab_padding': 12, 'nav_padding': 13, 'scrollbar_width': 16,
    },
}

_CANVAS = {
    'field': {
        'background': '#102019', 'text': '#ecf2e7', 'muted': '#a9baaa',
        'grid': '#3c5143', 'border': '#5d7560',
        'point_bins': ('#718f80', '#56c69f', '#b1e3cb', '#f0c781', '#d7eae0'),
        'plan': '#134fa5', 'route': '#09613d', 'waypoint': '#09613d',
        'photo_waypoint': '#935200', 'robot': '#ff7c87', 'robot_stale': '#f0bf55',
        'draft': '#974400', 'keepout': '#ff7c87', 'keepout_fill': '#38ff7c87',
        'polygon': '#ffd18a', 'error': '#ffb7b1', 'camera_background': '#14241b',
        'camera_text': '#f4f8ef', 'camera_overlay': '#a0102019',
    },
    'industrial': {
        'background': '#0c1823', 'text': '#e4f4ff', 'muted': '#a8bfce',
        'grid': '#344c60', 'border': '#4f718b',
        'point_bins': ('#718fa8', '#43d6b0', '#89dcf0', '#f4c779', '#d4eaff'),
        'plan': '#134fa5', 'route': '#09613d', 'waypoint': '#09613d',
        'photo_waypoint': '#935200', 'robot': '#ff8f9e', 'robot_stale': '#eac96a',
        'draft': '#974400', 'keepout': '#ff8493', 'keepout_fill': '#38ff8493',
        'polygon': '#ffcf7b', 'error': '#ffb6b6', 'camera_background': '#101c29',
        'camera_text': '#f2f8ff', 'camera_overlay': '#a00c1823',
    },
    'daylight': {
        'background': '#f8fafc', 'text': '#12243b', 'muted': '#41546d',
        'grid': '#bdc9d7', 'border': '#6b8198',
        'point_bins': ('#647b8d', '#006740', '#006b89', '#925100', '#344d7e'),
        'plan': '#134fa5', 'route': '#09613d', 'waypoint': '#09613d',
        'photo_waypoint': '#935200', 'robot': '#a92336', 'robot_stale': '#855100',
        'draft': '#974400', 'keepout': '#b11f31', 'keepout_fill': '#30b11f31',
        'polygon': '#935200', 'error': '#a12119', 'camera_background': '#17283c',
        'camera_text': '#ffffff', 'camera_overlay': '#b0101c2b',
    },
}

_QSS = Template('''
* {font-family: "Noto Sans CJK SC", "Microsoft YaHei", "DejaVu Sans"; font-size: ${font_size}px;}
QMainWindow,QWidget#root,QDialog {background:$background;color:$text}
QWidget {color:$text;background:transparent}
QDialog,QMessageBox,QDialog#navigation_setup {background:$background;color:$text}
QFrame#sidebar {background:$sidebar;border-right:${border_width}px solid $border_subtle}
QFrame#card,QFrame#viewer {background:$surface;border:${border_width}px solid $border_subtle;border-radius:${card_radius}px}
QFrame#global_status {background:$surface;border:${border_width}px solid $border_subtle;border-radius:${radius}px}
QLabel#status_item {font-size:${small_font_size}px;font-weight:600;padding:2px 5px}
QLabel#title {font-size:${title_size}px;font-weight:700;color:$title}
QLabel#h2 {font-size:${heading_size}px;font-weight:600;color:$title}
QLabel#muted {color:$muted;font-size:${small_font_size}px}
QLabel#number {font-size:${number_size}px;font-weight:700;color:$good}
QLabel[state="good"] {color:$good}
QLabel[state="warn"] {color:$warn}
QLabel[state="bad"] {color:$bad}
QLabel[state="muted"] {color:$muted}
QPushButton,QToolButton {background:$button_background;color:$button_text;border:${border_width}px solid $border;border-radius:${radius}px;min-height:${button_height}px;padding:${button_pad_y}px ${button_pad_x}px;font-weight:$font_weight}
QPushButton:hover,QToolButton:hover {background:$hover_background;border-color:$hover_border}
QPushButton:pressed,QToolButton:pressed {background:$pressed_background}
QPushButton:checked,QToolButton:checked {background:$selection_background;color:$selection_text;border-color:$focus}
QPushButton:focus,QToolButton:focus {border:2px solid $focus}
QPushButton#primary {background:$primary;color:$primary_text;border-color:$primary;font-weight:600}
QPushButton#primary:hover {background:$primary_hover;border-color:$primary_hover}
QPushButton#primary:pressed {background:$primary_pressed;border-color:$primary_pressed}
QPushButton#primary:focus {border:2px solid $title}
QPushButton#danger {background:$danger_background;color:$danger_text;border-color:$danger_border;font-weight:600}
QPushButton#danger:hover {background:$danger_hover;border-color:$danger_border}
QPushButton#danger:pressed {background:$danger_hover;border:2px solid $danger_text}
QPushButton#nav {background:transparent;color:$nav_text;border:0;border-left:3px solid transparent;border-radius:5px;text-align:left;padding:${nav_padding}px 12px}
QPushButton#nav:hover {background:$hover_background}
QPushButton#nav:checked {background:$nav_selected;color:$text;border-left:3px solid $nav_accent;font-weight:600}
QPushButton#nav:focus {border:2px solid $focus}
QPushButton:disabled,QToolButton:disabled,QPushButton#primary:disabled,QPushButton#danger:disabled,QPushButton#nav:disabled {background:$disabled_background;color:$disabled_text;border:${border_width}px solid $disabled_border}
QDoubleSpinBox,QSpinBox,QComboBox,QLineEdit {background:$input_background;color:$input_text;border:${border_width}px solid $border;border-radius:${control_radius}px;min-height:${control_height}px;padding:${control_padding}px;selection-background-color:$selection_background;selection-color:$selection_text}
QTextEdit,QPlainTextEdit,QListWidget,QTreeWidget,QTableWidget {background:$input_background;color:$input_text;border:${border_width}px solid $border;border-radius:${control_radius}px;padding:${control_padding}px;selection-background-color:$selection_background;selection-color:$selection_text;alternate-background-color:$surface_alt}
QDoubleSpinBox:focus,QSpinBox:focus,QComboBox:focus,QLineEdit:focus,QTextEdit:focus,QPlainTextEdit:focus,QListWidget:focus,QTreeWidget:focus,QTableWidget:focus {border:2px solid $focus}
QDoubleSpinBox:disabled,QSpinBox:disabled,QComboBox:disabled,QLineEdit:disabled,QTextEdit:disabled,QPlainTextEdit:disabled,QListWidget:disabled,QTreeWidget:disabled,QTableWidget:disabled {background:$disabled_background;color:$disabled_text;border-color:$disabled_border}
QComboBox::drop-down {width:25px;border-left:1px solid $border_subtle}
QDoubleSpinBox::up-button,QDoubleSpinBox::down-button,QSpinBox::up-button,QSpinBox::down-button {width:22px;border-left:1px solid $border_subtle}
QDoubleSpinBox::up-arrow,QSpinBox::up-arrow {image:url(:/agt_operator/${theme}-up.svg);width:12px;height:12px}
QDoubleSpinBox::down-arrow,QSpinBox::down-arrow,QComboBox::down-arrow {image:url(:/agt_operator/${theme}-down.svg);width:12px;height:12px}
QComboBox QAbstractItemView {background:$input_background;color:$input_text;border:${border_width}px solid $border;selection-background-color:$selection_background;selection-color:$selection_text;outline:0}
QListWidget::item,QTreeWidget::item,QTableWidget::item {padding:4px;border-radius:3px}
QListWidget::item:hover,QTreeWidget::item:hover,QTableWidget::item:hover {background:$hover_background;color:$text}
QListWidget::item:selected,QTreeWidget::item:selected,QTableWidget::item:selected {background:$selection_background;color:$selection_text}
QListWidget::item:selected:!active,QTreeWidget::item:selected:!active,QTableWidget::item:selected:!active {background:$selection_background;color:$selection_text}
QCheckBox,QRadioButton {spacing:8px;color:$text;min-height:${control_height}px}
QCheckBox:disabled,QRadioButton:disabled,QLabel:disabled {color:$disabled_text}
QCheckBox::indicator,QRadioButton::indicator {width:${indicator_size}px;height:${indicator_size}px}
QCheckBox::indicator:unchecked {image:url(:/agt_operator/${theme}-unchecked.svg)}
QCheckBox::indicator:checked {image:url(:/agt_operator/${theme}-checked.svg)}
QCheckBox:focus,QRadioButton:focus {outline:1px solid $focus}
QSplitter::handle {background:$border_subtle;width:6px;height:6px}
QSplitter::handle:hover {background:$focus}
QTabWidget::pane {border:${border_width}px solid $border_subtle;background:$surface;border-radius:${radius}px}
QTabBar::tab {background:$surface_alt;color:$muted;padding:${tab_padding}px 14px;border:${border_width}px solid $border_subtle;min-height:${button_height}px}
QTabBar::tab:hover {background:$hover_background;color:$text}
QTabBar::tab:selected {background:$surface;color:$text;border-bottom:3px solid $nav_accent;font-weight:600}
QTabBar::tab:disabled {background:$disabled_background;color:$disabled_text}
QScrollArea {border:0;background:transparent}
QScrollBar:vertical {background:$surface_alt;width:${scrollbar_width}px;margin:0}
QScrollBar:horizontal {background:$surface_alt;height:${scrollbar_width}px;margin:0}
QScrollBar::handle {background:$scrollbar;border-radius:4px;min-width:24px;min-height:24px}
QScrollBar::handle:hover {background:$focus}
QScrollBar::add-line,QScrollBar::sub-line {width:0;height:0;border:0;background:transparent}
QScrollBar::add-page,QScrollBar::sub-page {background:transparent}
QMenuBar,QMenu,QStatusBar {background:$surface;color:$text}
QMenuBar::item {background:transparent;padding:6px 10px}
QMenuBar::item:selected,QMenu::item:selected {background:$selection_background;color:$selection_text}
QMenu {border:${border_width}px solid $border;padding:4px}
QMenu::item {padding:7px 20px}
QMenu::item:disabled {color:$disabled_text}
QMenu::separator {height:1px;background:$border_subtle;margin:4px}
QGroupBox {border:${border_width}px solid $border_subtle;border-radius:${radius}px;margin-top:10px;padding:10px;color:$text}
QGroupBox::title {subcontrol-origin:margin;left:10px;padding:0 4px}
QHeaderView::section {background:$surface_alt;color:$text;padding:6px;border:1px solid $border_subtle}
QProgressBar {background:$surface_alt;color:$text;border:${border_width}px solid $border;border-radius:5px;text-align:center;min-height:${control_height}px}
QProgressBar::chunk {background:$primary;border-radius:3px}
QSlider::groove:horizontal {height:6px;background:$border_subtle;border-radius:3px}
QSlider::handle:horizontal {background:$primary;border:1px solid $focus;width:${indicator_size}px;margin:-7px 0;border-radius:5px}
QSlider::handle:horizontal:disabled {background:$disabled_background;border-color:$disabled_border}
QToolTip {background:$tooltip_background;color:$tooltip_text;border:1px solid $border;padding:7px}
''')


def _theme_name(theme):
    if not isinstance(theme, str) or theme not in _TOKENS:
        raise ValueError('unknown operator theme: ' + repr(theme))
    return theme


def theme_tokens(theme='field'):
    """Return independent tokens, also useful for widget inline styles."""
    return {**_BASE, **_TOKENS[_theme_name(theme)]}


def status_colors(theme='field'):
    """Foregrounds chosen for both root and panel backgrounds."""
    tokens = theme_tokens(theme)
    return {name: tokens[name] for name in ('good', 'warn', 'bad', 'muted')}


def canvas_palette(theme='field'):
    """Return QColor-compatible colours and an immutable point_bins tuple.

    Alpha colours use Qt's #AARRGGBB format. Camera text and overlay stay light
    on dark for all themes so live image readability does not depend on scene.
    """
    return dict(_CANVAS[_theme_name(theme)])


def style_for(theme='field', scale=1.0):
    """Build complete QSS with one token template and a positive pixel scale.

    Pixel dimensions, including negative slider margins, scale together; colour
    and font-weight numbers never change. A one-pixel edge remains visible.
    """
    if isinstance(scale, bool) or not isinstance(scale, (int, float)) or not math.isfinite(scale) or scale <= 0:
        raise ValueError('theme scale must be a positive finite number')
    # Compiled, package-local SVGs make native spin/drop-down/checkbox indicators
    # visible on dark panels too. No temp files, network or QApplication needed.
    try:
        from . import operator_theme_icons  # noqa: F401
    except ImportError:
        # Token-only tooling can still inspect themes without installing Qt.
        pass
    style = _QSS.substitute(theme_tokens(theme), theme=theme)
    if scale == 1:
        return style

    def resize(match):
        value = int(match.group(1))
        if value == 0:
            return '0px'
        dimension = max(1, round(abs(value) * scale))
        return str(-dimension if value < 0 else dimension) + 'px'

    return re.sub(r'(-?\d+)px', resize, style)


# Existing imports keep the light default; runtime switches use style_for.
STYLE = style_for('field')
