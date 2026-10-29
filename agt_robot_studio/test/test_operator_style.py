"""Theme contracts and operator readability, independent of robot services."""
import os
import unittest

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from agt_robot_studio.operator_style import (
    STYLE, THEMES, canvas_palette, status_colors, style_for, theme_tokens,
)

try:
    from PyQt5.QtCore import qInstallMessageHandler
    from PyQt5.QtGui import QImage, QPalette
    from PyQt5.QtWidgets import (
        QApplication, QCheckBox, QComboBox, QDoubleSpinBox, QFrame, QLabel,
        QLineEdit, QListWidget, QMainWindow, QPushButton, QScrollArea, QSpinBox,
        QSplitter, QTabWidget, QTextEdit, QVBoxLayout, QWidget,
    )
except ImportError:
    QApplication = None


def contrast_ratio(foreground, background):
    """WCAG relative luminance for the public opaque RGB tokens."""
    def luminance(color):
        value = color.removeprefix('#')
        if len(value) != 6:
            raise AssertionError('Contrast tokens must be opaque #RRGGBB: ' + color)
        channels = [int(value[index:index + 2], 16) / 255. for index in (0, 2, 4)]
        linear = [c / 12.92 if c <= .04045 else ((c + .055) / 1.055) ** 2.4
                  for c in channels]
        return sum(a * b for a, b in zip(linear, (.2126, .7152, .0722)))
    a, b = sorted((luminance(foreground), luminance(background)))
    return (b + .05) / (a + .05)


class OperatorThemeContracts(unittest.TestCase):
    def test_map_annotations_contrast_with_unchanged_white_occupancy_pixels(self):
        for theme in THEMES:
            for key in ('plan','route','waypoint','photo_waypoint','draft'):
                self.assertGreaterEqual(contrast_ratio(canvas_palette(theme)[key],'#ffffff'),4.5)

    def test_default_style_remains_compatible_and_choices_are_described(self):
        self.assertEqual(list(THEMES), ['field', 'industrial', 'daylight'])
        self.assertEqual(STYLE, style_for('field'))
        self.assertEqual(style_for(), STYLE)
        for ident, info in THEMES.items():
            with self.subTest(theme=ident):
                self.assertTrue(info['name'].strip())
                self.assertTrue(info['description'].strip())

    def test_invalid_theme_and_scale_are_rejected(self):
        for factory in (style_for, theme_tokens, canvas_palette, status_colors):
            with self.subTest(factory=factory.__name__):
                with self.assertRaises(ValueError):
                    factory('missing-theme')
        for scale in (0, -1, float('nan'), float('inf'), -float('inf'), True, False):
            with self.subTest(scale=scale):
                with self.assertRaises(ValueError):
                    style_for('field', scale=scale)

    def test_each_theme_changes_controls_and_canvas_with_isolated_snapshots(self):
        themes = [theme_tokens(ident) for ident in THEMES]
        self.assertEqual(len({t['background'] for t in themes}), 3)
        self.assertEqual(len({t['primary'] for t in themes}), 3)
        self.assertEqual(len({t['input_text'] for t in themes}), 3)
        canvases = [canvas_palette(ident) for ident in THEMES]
        self.assertEqual(len({p['background'] for p in canvases}), 3)
        self.assertTrue(all(isinstance(p['point_bins'], tuple) for p in canvases))
        for factory, key in ((theme_tokens, 'text'), (canvas_palette, 'background'),
                             (status_colors, 'good')):
            before = factory('industrial')
            modified = factory('industrial')
            modified[key] = '#123456'
            self.assertEqual(factory('industrial'), before)

    def test_text_input_primary_and_status_colors_remain_readable(self):
        for ident in THEMES:
            tokens = theme_tokens(ident)
            pairs = [('text', 'background'), ('text', 'surface'),
                     ('muted', 'background'), ('input_text', 'input_background'),
                     ('button_text', 'button_background'), ('primary_text', 'primary')]
            for foreground, background in pairs:
                with self.subTest(theme=ident, foreground=foreground, background=background):
                    ratio = contrast_ratio(tokens[foreground], tokens[background])
                    required = 7. if ident == 'daylight' and foreground == 'text' else 4.5
                    self.assertGreaterEqual(ratio, required,
                        f'{ident} {foreground}/{background}: {ratio:.2f}:1')
            for name, color in status_colors(ident).items():
                for background in ('background', 'surface'):
                    with self.subTest(theme=ident, status=name, background=background):
                        ratio = contrast_ratio(color, tokens[background])
                        self.assertGreaterEqual(ratio, 4.5,
                            f'{ident} {name}/{background}: {ratio:.2f}:1')


@unittest.skipIf(QApplication is None, 'PyQt5 is not installed in this test environment')
class OperatorThemeQt(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def test_packaged_spin_and_checkbox_icons_load_for_every_theme(self):
        for ident in THEMES:
            style_for(ident)
            for symbol in ('up', 'down', 'checked', 'unchecked'):
                with self.subTest(theme=ident, symbol=symbol):
                    image = QImage(f':/agt_operator/{ident}-{symbol}.svg')
                    self.assertFalse(image.isNull(),
                                     f'Missing or unreadable {ident} {symbol} icon')
                    self.assertGreater(image.width(), 0)
                    self.assertGreater(image.height(), 0)

    def sample(self):
        window = QMainWindow()
        window.resize(700, 900)
        self.addCleanup(window.close)
        root = QWidget()
        root.setObjectName('root')
        window.setCentralWidget(root)
        layout = QVBoxLayout(root)
        for name in ('sidebar', 'card', 'viewer', 'global_status'):
            frame = QFrame()
            frame.setObjectName(name)
            row = QVBoxLayout(frame)
            label = QLabel('机器人状态 · 定位有效')
            label.setObjectName('status_item')
            row.addWidget(label)
            layout.addWidget(frame)
        normal = QPushButton('加载路线')
        layout.addWidget(normal)
        for name in ('primary', 'danger', 'nav'):
            button = QPushButton('操作 / ' + name)
            button.setObjectName(name)
            button.setCheckable(name == 'nav')
            button.setChecked(name == 'nav')
            layout.addWidget(button)
        disabled = QPushButton('等待定位后执行')
        disabled.setEnabled(False)
        layout.addWidget(disabled)
        edit = QLineEdit('路线 P001')
        layout.addWidget(edit)
        for widget in (QDoubleSpinBox(), QSpinBox(), QCheckBox('到点拍照')):
            layout.addWidget(widget)
        combo = QComboBox()
        combo.addItems(['地图 A', '地图 B'])
        layout.addWidget(combo)
        tabs = QTabWidget()
        tabs.addTab(QTextEdit('云台角度反馈'), '画面 / 云台')
        route = QListWidget()
        route.addItems(['P001 · 起点', 'P002 · 拍照'])
        route.setCurrentRow(0)
        tabs.addTab(route, '路线点')
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(tabs)
        split = QSplitter()
        split.addWidget(scroll)
        split.addWidget(QLabel('地图画布'))
        layout.addWidget(split)
        return window, normal, edit, disabled

    def test_all_themes_apply_to_real_widgets_without_stylesheet_warnings(self):
        messages = []
        previous = qInstallMessageHandler(lambda _kind, _context, message: messages.append(message))
        self.addCleanup(qInstallMessageHandler, previous)
        for ident in THEMES:
            with self.subTest(theme=ident):
                window, _, edit, disabled = self.sample()
                window.setStyleSheet(style_for(ident))
                window.show()
                self.app.processEvents()
                tokens = theme_tokens(ident)
                self.assertEqual(edit.palette().color(QPalette.Text).name(),
                                 tokens['input_text'].lower())
                self.assertFalse(disabled.isEnabled())
                self.assertEqual(disabled.palette().color(
                    QPalette.Disabled, QPalette.ButtonText).name(),
                    tokens['disabled_text'].lower())
                window.close()
        stylesheet_errors = [m for m in messages
                             if any(needle in m.lower() for needle in (
                                 'could not parse', 'unknown property', 'stylesheet error'))]
        self.assertEqual(stylesheet_errors, [])

    def test_daylight_has_larger_controls_and_scaling_changes_real_size_hints(self):
        sizes = {}
        for ident, scale in (('field', 1.), ('daylight', 1.), ('field', .8)):
            window, button, edit, _ = self.sample()
            window.setStyleSheet(style_for(ident, scale=scale))
            button.ensurePolished()
            edit.ensurePolished()
            sizes[ident, scale] = (button.sizeHint(), edit.sizeHint())
        self.assertGreater(sizes['daylight', 1.][0].height(), sizes['field', 1.][0].height())
        self.assertGreaterEqual(sizes['daylight', 1.][0].height(), 40)
        self.assertGreater(sizes['daylight', 1.][1].height(), sizes['field', 1.][1].height())
        self.assertLess(sizes['field', .8][0].height(), sizes['field', 1.][0].height())


if __name__ == '__main__':
    unittest.main()
