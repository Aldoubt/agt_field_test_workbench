"""Light, high contrast operator chrome; sensor canvases retain their own palette."""
STYLE = '''
* {font-family: "Noto Sans CJK SC", "Microsoft YaHei", "DejaVu Sans"; font-size: 13px;}
QMainWindow,QWidget#root {background:#f3f5ef;color:#202922}
QDialog#navigation_setup {background:#f3f5ef}
QWidget {color:#26372d;background:transparent}
QFrame#sidebar {background:#e9eee4;border-right:1px solid #d4ddcc}
QFrame#card {background:#ffffff;border:1px solid #d8e0d2;border-radius:12px}
QFrame#viewer {background:#ffffff;border:1px solid #d8e0d2;border-radius:12px}
QFrame#global_status {background:#ffffff;border:1px solid #d4ddcc;border-radius:10px}
QLabel#status_item {font-size:12px;font-weight:600;padding:2px 5px}
QLabel#title {font-size:20px;font-weight:700;color:#18251a}
QLabel#h2 {font-size:16px;font-weight:650;color:#203323}
QLabel#muted {color:#556453;font-size:12px}
QLabel#number {font-size:24px;font-weight:700;color:#426a16}
QPushButton {background:#ffffff;border:1px solid #c7d3bd;color:#2c4128;border-radius:8px;padding:10px 15px;font-weight:500}
QPushButton:hover {background:#edf5df;border:1px solid #7e9f55}
QPushButton:pressed {background:#dcebbf}
QPushButton:disabled {color:#788071;background:#e9ede5;border:1px solid #d8dfd0}
QPushButton#primary {background:#507b20;color:#ffffff;border:1px solid #507b20}
QPushButton#primary:hover {background:#3f6518}
QPushButton#primary:disabled {background:#e9ede5;color:#788071;border:1px solid #d8dfd0}
QPushButton#danger {background:#fff3f0;border:1px solid #c75c4c;color:#a12820}
QPushButton#nav {border:0;background:transparent;text-align:left;padding:13px 14px;color:#53604c}
QPushButton#nav:checked {background:#ffffff;color:#2e5018;border-left:3px solid #78a62a}
QPushButton#nav:hover {background:#f6f9f0}
QDoubleSpinBox,QSpinBox,QComboBox,QTextEdit,QLineEdit,QListWidget {background:#ffffff;color:#22301e;border:1px solid #c8d3bd;border-radius:6px;padding:6px}
QComboBox QAbstractItemView {background:#ffffff;color:#26372d;selection-background-color:#dfedc8;selection-color:#213d18}
QListWidget::item:selected {background:#e4efcf;color:#203d16}
QCheckBox {spacing:8px;color:#354a2b}
QSplitter::handle {background:#d7dfcf;width:6px}
QTabWidget::pane {border:1px solid #d4ddcc;background:#ffffff;border-radius:8px}
QTabBar::tab {background:#e7ecdf;color:#4e6046;padding:10px 18px;border:1px solid #d4ddcc}
QTabBar::tab:selected {background:#ffffff;color:#385e1c}
QScrollArea {border:0}
QToolTip {background:#243521;color:#ffffff;padding:6px}
'''
