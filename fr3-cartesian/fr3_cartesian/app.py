import argparse
from collections import deque
from dataclasses import asdict
from pathlib import Path
import sys
import time

from PySide6.QtCore import Qt, QTimer, QEvent, QSettings, QSignalBlocker, QTranslator, QLibraryInfo
from PySide6.QtGui import QKeySequence, QShortcut
from PySide6.QtWidgets import (
    QApplication, QCheckBox, QComboBox, QDoubleSpinBox, QFileDialog, QFormLayout,
    QGridLayout, QGroupBox, QHBoxLayout, QLabel, QLineEdit, QListWidget,
    QMainWindow, QPushButton, QScrollArea, QSlider, QSpinBox, QSplitter, QTabWidget,
    QTableWidget, QTableWidgetItem, QTextEdit, QVBoxLayout, QWidget,
)

from .client import RobotClient
from .geometry import Plane, calibrate_pivot, checked_target, check_current_plane_position
from .i18n import LANGUAGES, translate
from .messages import Message, OperatorValueError, error_message
from .persistence import read_calibration, write_calibration
from .tasks import build_trajectory, validate_array, validate_poses

# Resolve beside this package, not the process working directory: the camera
# workbenches in the parent repository have their own calibration and SDK files.
# Editable installs are the supported deployment layout for this prototype.
ROOT = Path(__file__).resolve().parents[1]


def number(low, high, value, decimals=2, suffix=''):
    box = QDoubleSpinBox()
    box.setRange(low, high)
    box.setDecimals(decimals)
    box.setValue(value)
    box.setSuffix(suffix)
    return box


class MainWindow(QMainWindow):
    def __init__(self, settings=None, language=None, session_path=None):
        super().__init__()
        self.settings = settings if settings is not None else QSettings('FR3Cartesian', 'Studio')
        storage_root = Path(self.settings.fileName()).parent if settings is not None else ROOT
        self.session_path = Path(session_path) if session_path is not None else storage_root/'calibration-session.json'
        self.legacy_path = ROOT/'calibration.json' if settings is None and session_path is None else None
        self.restoring_calibration = True
        self.session_dirty = False
        self.autosave_timer = QTimer(self)
        self.autosave_timer.setSingleShot(True)
        self.autosave_timer.setInterval(300)
        self.autosave_timer.timeout.connect(self.autosave)
        preferred = language or self.settings.value('ui/language', 'en')
        self.language = preferred if preferred in LANGUAGES else 'en'
        self.text_bindings = []
        self.tab_sources = []
        self.log_entries = deque(maxlen=500)
        self.summary_status = 'initial'
        self.plane_name_edited = False
        self.qt_translator = QTranslator(self)
        self.setWindowTitle(self.tr('FR3 / Cartesian Studio'))
        self.resize(1200, 890)
        self.client = RobotClient(self)
        self.client.completed.connect(self.completed)
        self.client.failed.connect(self.failure)
        self.client.rejected.connect(self.move_rejected)
        self.client.diagnostic.connect(self.diagnostic)
        self.client.stopped.connect(lambda ok: self.log('Stop acknowledged' if ok else 'Stop not confirmed — use the physical stop'))
        self.connected = False
        self.state = None
        self.state_time = 0.
        self.tool_samples = []
        self.samples_reference = 'robot_base'
        self.tool_result = None
        self.tool_applied = False
        self.source = None
        self.planes = []
        self.plane_points = []
        self.draft_reference = 'robot_base'
        self.orientation = None
        self.saved_orientation = None
        self.capture_kind = None
        self.demo_pending = None
        self.repeat_direction = None
        self.plane_arrow_held = False
        self.plane_arrow_button = None
        self.plane_repeat_timer = QTimer(self)
        self.plane_repeat_timer.setSingleShot(True)
        self.plane_repeat_timer.timeout.connect(self.repeat_step)
        self.active_jog = None
        self.move_started = 0.
        self.last_plane_application = None
        self.task_path = []
        self.task_index = 0
        self.task_running = False

        central = QWidget()
        self.setCentralWidget(central)
        outer = QVBoxLayout(central)
        language_row = QHBoxLayout()
        title = self.label('FR3  /  CARTESIAN STUDIO')
        title.setObjectName('title')
        language_row.addWidget(title, 1)
        language_row.addWidget(self.label('Language / 语言'))
        self.language_combo = QComboBox()
        for code, name in LANGUAGES.items():
            self.language_combo.addItem(name, code)
        self.language_combo.setCurrentIndex(list(LANGUAGES).index(self.language))
        self.language_combo.currentIndexChanged.connect(self.language_changed)
        language_row.addWidget(self.language_combo)
        outer.addLayout(language_row)
        outer.addWidget(self.paragraph('Teach the tool tip. Define a workplane. Move in XYZ while holding the tool orientation.'))
        split = QSplitter()
        outer.addWidget(split, 1)
        sidebar = QWidget()
        side = QVBoxLayout(sidebar)
        sidebar_scroll = QScrollArea()
        sidebar_scroll.setWidgetResizable(True)
        sidebar_scroll.setFrameShape(QScrollArea.NoFrame)
        sidebar_scroll.setMinimumWidth(320)
        sidebar_scroll.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOn)
        sidebar_scroll.viewport().setStyleSheet('background-color: #101820;')
        sidebar_scroll.setStyleSheet('''
            QScrollArea { background-color: #101820; border: none; }
            QScrollBar:vertical { background: #101820; width: 12px; margin: 0; }
            QScrollBar::handle:vertical { background: #344955; min-height: 28px; border-radius: 4px; }
            QScrollBar::handle:vertical:hover { background: #496674; }
            QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0; }
            QScrollBar::add-page:vertical, QScrollBar::sub-page:vertical { background: #101820; }
        ''')
        sidebar.setStyleSheet('background-color: #101820;')
        sidebar_scroll.setWidget(sidebar)
        split.addWidget(sidebar_scroll)
        self.build_connection(side)
        self.build_teaching(side)
        self.build_base_jog(side)
        side.addStretch()
        self.stop_button = self.button('STOP  ·  Esc', lambda: self.stop(reason=Message('On-screen STOP button')), side)
        self.stop_button.setObjectName('stop')
        side.addWidget(self.paragraph('Software stop requires communication. Keep the controller’s physical stop accessible.'))

        right = QWidget()
        layout = QVBoxLayout(right)
        split.addWidget(right)
        self.tabs = QTabWidget()
        layout.addWidget(self.tabs, 1)
        self.build_tool()
        self.build_planes()
        self.build_motion()
        self.build_pick_place()
        footer = QHBoxLayout()
        self.button('Save calibration…', self.save, footer)
        self.button('Load calibration…', self.load, footer)
        layout.addLayout(footer)
        self.messages = QTextEdit()
        self.messages.setReadOnly(True)
        self.messages.setMaximumHeight(125)
        layout.addWidget(self.messages)
        split.setSizes([310, 870])
        QShortcut(QKeySequence('Escape'), self, activated=lambda: self.stop(reason=Message('Esc key')))
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.poll)
        self.timer.start(250)
        self.setStyleSheet('''
            QMainWindow { background: #101820; }
            QWidget { color: #e5ecf0; font-size: 13px; }
            QLabel#title { font-size: 25px; font-weight: 700; color: #5ee0c1; padding: 8px 0; }
            QGroupBox { border: 1px solid #344955; border-radius: 8px; margin-top: 13px; padding: 14px 8px 8px; }
            QGroupBox::title { subcontrol-origin: margin; left: 12px; color: #89cfeb; }
            QPushButton { background: #263d4a; border: 1px solid #405b68; padding: 8px; border-radius: 5px; }
            QPushButton:hover { background: #365766; }
            QPushButton:disabled { color: #657781; background: #1c2930; }
            QPushButton#stop { background: #b12f43; font-weight: 700; font-size: 20px; padding: 15px; }
            QLineEdit, QSpinBox, QDoubleSpinBox, QComboBox, QListWidget, QTextEdit { background: #182732; border: 1px solid #39515f; padding: 5px; }
            QTableWidget { background: #182732; alternate-background-color: #1e303b; color: #e5ecf0; gridline-color: #344955; border: 1px solid #39515f; selection-background-color: #287968; selection-color: #ffffff; }
            QTableWidget::item { padding: 5px; }
            QHeaderView::section { background: #223440; color: #89cfeb; border: 1px solid #344955; padding: 6px; }
            QTableCornerButton::section { background: #223440; border: 1px solid #344955; }
            QTabWidget::pane { border: 1px solid #344955; }
            QTabBar::tab { background: #223440; padding: 12px; }
            QTabBar::tab:selected { background: #355565; color: #65e6c7; }
        ''')
        self.retranslate()
        self.log('Simulator selected. No robot connection is opened at startup.' if self.mode.currentIndex() == 0
                 else 'Real FR3 selected. No robot connection is opened at startup.')
        self.setup_autosave()
        self.restore_startup_calibration()
        self.restoring_calibration = False
        if self.session_dirty:
            self.autosave()
        QApplication.instance().installEventFilter(self)

    def tr(self, text, **values):
        return translate(self.language, text, **values)

    def bind_text(self, widget, source, setter='setText'):
        self.text_bindings.append((widget, setter, source))
        getattr(widget, setter)(self.tr(source))
        return widget

    def label(self, source):
        return self.bind_text(QLabel(), source)

    def paragraph(self, source):
        widget = self.label(source)
        widget.setWordWrap(True)
        widget.setTextInteractionFlags(Qt.TextSelectableByMouse)
        return widget

    def pushbutton(self, source):
        return self.bind_text(QPushButton(), source)

    def button(self, source, action, layout):
        widget = self.pushbutton(source)
        widget.clicked.connect(action)
        layout.addWidget(widget)
        return widget

    def checkbox(self, source):
        return self.bind_text(QCheckBox(), source)

    def group(self, source):
        return self.bind_text(QGroupBox(), source, 'setTitle')

    def add_tab(self, widget, source):
        self.tab_sources.append(source)
        if not isinstance(widget, QScrollArea):
            scroll = QScrollArea()
            scroll.setWidgetResizable(True)
            scroll.setFrameShape(QScrollArea.NoFrame)
            scroll.setMinimumWidth(400)
            scroll.setStyleSheet('QScrollArea { background-color: #101820; border: none; }')
            scroll.viewport().setStyleSheet('background-color: #101820;')
            widget.setObjectName('calibrationTabContent')
            widget.setStyleSheet('QWidget#calibrationTabContent { background-color: #101820; }')
            scroll.setWidget(widget)
            widget = scroll
        return self.tabs.addTab(widget, self.tr(source))

    def language_changed(self, index):
        code = self.language_combo.itemData(index)
        if code in LANGUAGES and code != self.language:
            previous_default = self.tr('Pick plane')
            self.language = code
            if not self.plane_name_edited and self.plane_name.text() == previous_default:
                self.plane_name.setText(self.tr('Pick plane'))
            self.retranslate()
            self.settings.setValue('ui/language', code)
            self.settings.sync()

    def retranslate(self):
        # Update existing widgets. Do not rebuild forms, reset checkboxes, or send
        # robot commands when an operator changes the display language.
        self.setWindowTitle(self.tr('FR3 / Cartesian Studio'))
        app = QApplication.instance()
        app.removeTranslator(self.qt_translator)
        if self.language == 'zh_CN':
            translations = QLibraryInfo.path(QLibraryInfo.TranslationsPath)
            if self.qt_translator.load('qtbase_zh_CN', translations):
                app.installTranslator(self.qt_translator)
        for widget, setter, source in self.text_bindings:
            getattr(widget, setter)(self.tr(source))
        for index, source in enumerate(self.tab_sources):
            self.tabs.setTabText(index, self.tr(source))
        self.task_points.setHorizontalHeaderLabels([self.tr(text) for text in ('Row', 'Column', 'Plane X / mm', 'Plane Y / mm', 'Plane Z / mm')])
        self.task_waypoints.setHorizontalHeaderLabels([self.tr(text) for text in ('X / mm', 'Y / mm', 'Z / mm', 'RX / °', 'RY / °', 'RZ / °')])
        with QSignalBlocker(self.mode):
            self.mode.setItemText(0, self.tr('Simulator'))
            self.mode.setItemText(1, self.tr('Real FR3'))
        self.mode.setToolTip(self.tr('Changing the backend disconnects the current session.'))
        self.render_details()
        self.render_logs()
        self.refresh()

    def render_details(self):
        selected = self.sample_list.currentRow()
        self.sample_list.clear()
        for i, pose in enumerate(self.tool_samples):
            self.sample_list.addItem(self.tr('{index}   flange: {pose}', index=i+1,
                                            pose=', '.join(f'{v:.2f}' for v in pose)))
        self.sample_list.setCurrentRow(selected)
        if self.tool_result:
            result = self.tool_result
            source = ('Loaded TCP / flange: {xyz}\nContact RMS {rms:.4f} mm. Verify or apply TCP before moving.'
                      if self.summary_status == 'loaded' else
                      'TCP XYZ / flange: {xyz}\nContact RMS {rms:.4f} mm · max {maximum:.4f} mm · condition {condition:.1f}\nTool rotation: 0°, 0°, 0° (flange-aligned axes).')
            self.tool_summary.setText(self.tr(source, xyz=', '.join(f'{v:.3f} mm' for v in result['tool'][:3]),
                                             rms=result['rms'], maximum=result['maximum'], condition=result['condition']))
        else:
            source = ('Contacts changed. Recalculate and apply the tool; workplanes must be retaught.'
                      if self.summary_status == 'changed' else 'Capture at least four contacts to calculate the TCP.')
            self.tool_summary.setText(self.tr(source))
        if self.orientation is None and self.saved_orientation is not None:
            self.orientation_label.setText(self.tr('Saved orientation: {angles}. Lock current orientation again.',
                                                  angles=' / '.join(f'{v:.2f}°' for v in self.saved_orientation)))
        elif self.orientation is None:
            self.orientation_label.setText(self.tr('Orientation not locked'))
        else:
            self.orientation_label.setText(' / '.join(f'{v:.2f}°' for v in self.orientation))
        self.render_plane_summary()

    def render_plane_summary(self):
        index = self.plane_list.currentRow()
        if 0 <= index < len(self.planes):
            plane = self.planes[index]
            if plane.reference != 'robot_base':
                self.plane_summary.setText(self.tr('This plane was captured with the old coordinate-frame bug. Load a recovered base-frame calibration or reteach the plane before using it.'))
                return
            self.plane_summary.setText(self.tr('{name}\nOrigin / base: {origin}\n+Z normal / base: {normal}',
                                               name=plane.name, origin=str(plane.origin),
                                               normal=', '.join(f'{row[2]:.4f}' for row in plane.basis)))
        else:
            self.plane_summary.setText(self.tr('No workplanes taught yet.'))

    def build_connection(self, layout):
        group = self.group('CONNECTION')
        form = QFormLayout(group)
        self.mode = QComboBox()
        self.mode.addItems(['Simulator', 'Real FR3'])
        saved_backend = str(self.settings.value('connection/backend', 'robot')).lower()
        self.mode.setCurrentIndex(0 if saved_backend in ('simulator', 'simulation', '0') else 1)
        self.mode.currentIndexChanged.connect(self.backend_changed)
        self.ip = QLineEdit('192.168.58.2')
        self.sdk = QLineEdit(str(ROOT / 'vendor'))
        form.addRow(self.label('Backend'), self.mode)
        self.mode.setToolTip(self.tr('Changing the backend disconnects the current session.'))
        form.addRow(self.label('Robot IP'), self.ip)
        form.addRow(self.label('SDK folder'), self.sdk)
        self.connect_button = self.pushbutton('Connect')
        self.connect_button.clicked.connect(self.connect_robot)
        form.addRow(self.connect_button)
        self.status = self.paragraph('Disconnected')
        self.status.setFixedHeight(40)
        form.addRow(self.status)
        self.readout = self.paragraph('TCP in robot base: —')
        self.readout.setMinimumHeight(110)
        form.addRow(self.readout)
        form.addRow(self.paragraph('V3.8.25-QX · use manual mode\nSet payload/limits in WebApp.\nPlane moves require base workpiece 0.'))
        layout.addWidget(group)

    def build_teaching(self, layout):
        group = self.group('TEACHING / JOINT JOG')
        box = QVBoxLayout(group)
        box.addWidget(self.paragraph('Use the pendant/drag teaching or bounded joint jog below. Exit drag mode before capturing a point.'))
        self.teach_enabled = self.checkbox('Enable teaching jog controls')
        self.teach_enabled.toggled.connect(self.teaching_changed)
        box.addWidget(self.teach_enabled)
        row = QHBoxLayout()
        self.joint_limit = number(.1, 10, .5, 1, ' °')
        row.addWidget(self.label('Joint travel limit'))
        row.addWidget(self.joint_limit)
        box.addLayout(row)
        speed_row = QHBoxLayout()
        speed_row.addWidget(self.label('Jog speed'))
        self.jog_speed = QSpinBox()
        self.jog_speed.setRange(1, 10)
        self.jog_speed.setValue(3)
        self.jog_speed.setSuffix(' %')
        self.jog_speed_slider = QSlider(Qt.Horizontal)
        self.jog_speed_slider.setRange(1, 10)
        self.jog_speed_slider.setValue(3)
        self.jog_speed.valueChanged.connect(self.jog_speed_slider.setValue)
        self.jog_speed_slider.valueChanged.connect(self.jog_speed.setValue)
        speed_row.addWidget(self.jog_speed_slider, 1)
        speed_row.addWidget(self.jog_speed)
        box.addLayout(speed_row)
        grid = QGridLayout()
        self.joint_buttons = []
        self.joint_labels = []
        for joint in range(1, 7):
            joint_label = self.label(f'J{joint}')
            self.joint_labels.append(joint_label)
            grid.addWidget(joint_label, joint-1, 0)
            for direction in (0, 1):
                b = self.pushbutton('−' if direction == 0 else '+')
                b.setMinimumHeight(30)
                b.setFocusPolicy(Qt.NoFocus)
                b.pressed.connect(lambda j=joint, d=direction, button=b: self.begin_jog(0, j, d, self.joint_limit.value(), button))
                b.released.connect(lambda: self.end_jog(1))
                grid.addWidget(b, joint-1, direction+1)
                self.joint_buttons.append(b)
        box.addLayout(grid)
        self.jog_status = self.paragraph('Connect to a robot or simulator to jog.')
        self.jog_status.setFixedHeight(48)
        box.addWidget(self.jog_status)
        box.addWidget(self.paragraph('Hold a joint button to jog. Release to stop; the travel limit also stops the jog. Joint motion may change tool orientation.'))
        layout.addWidget(group)

    def build_base_jog(self, layout):
        group = self.group('BASE FRAME CARTESIAN JOG')
        box = QVBoxLayout(group)
        box.addWidget(self.paragraph('Hold an axis button to jog in the robot base frame. Release stops; travel limits cap each jog.'))
        form = QFormLayout()
        self.base_linear_limit = number(.1, 50, 5, 1, ' mm')
        self.base_rotation_limit = number(.1, 10, 1, 1, ' °')
        form.addRow(self.label('Linear travel limit'), self.base_linear_limit)
        form.addRow(self.label('Rotation travel limit'), self.base_rotation_limit)
        box.addLayout(form)
        speed_row = QHBoxLayout()
        speed_row.addWidget(self.label('TCP jog speed'))
        self.base_jog_speed = QSpinBox()
        self.base_jog_speed.setRange(1, 10)
        self.base_jog_speed.setValue(3)
        self.base_jog_speed.setSuffix(' %')
        self.base_jog_speed_slider = QSlider(Qt.Horizontal)
        self.base_jog_speed_slider.setRange(1, 10)
        self.base_jog_speed_slider.setValue(3)
        self.base_jog_speed.valueChanged.connect(self.base_jog_speed_slider.setValue)
        self.base_jog_speed_slider.valueChanged.connect(self.base_jog_speed.setValue)
        speed_row.addWidget(self.base_jog_speed_slider, 1)
        speed_row.addWidget(self.base_jog_speed)
        box.addLayout(speed_row)
        box.addWidget(self.paragraph('Controller speed percentage for base XYZ and rotation jogs. Joint jog speed is set separately.'))
        grid = QGridLayout()
        self.base_jog_buttons = []
        for row, (axis_name, axis) in enumerate((('X', 1), ('Y', 2), ('Z', 3),
                                                 ('RX', 4), ('RY', 5), ('RZ', 6))):
            grid.addWidget(self.label(axis_name), row, 0)
            for col, direction in enumerate((0, 1), start=1):
                sign = '−' if direction == 0 else '+'
                button = self.pushbutton(f'{sign}{axis_name}')
                button.setMinimumHeight(30)
                button.setFocusPolicy(Qt.NoFocus)
                button.pressed.connect(lambda a=axis, d=direction, b=button: self.begin_base_jog(a, d, b))
                button.released.connect(lambda: self.end_jog(3))
                grid.addWidget(button, row, col)
                self.base_jog_buttons.append(button)
        box.addLayout(grid)
        layout.addWidget(group)

    def build_tool(self):
        tab = QWidget()
        box = QVBoxLayout(tab)
        box.addWidget(self.paragraph('1  /  TOOL CENTER POINT'))
        box.addWidget(self.paragraph('Touch the same rigid reference point with the tool tip from at least four different orientations. Tilt about two axes, keeping the tip on the reference. Capture only while stationary. This pivot method calibrates XYZ; tool axes remain aligned with the flange.'))
        self.sample_list = QListWidget()
        self.sample_list.setMinimumHeight(140)
        box.addWidget(self.sample_list, 1)
        row = QHBoxLayout()
        self.capture_tool_button = self.button('Capture contact', lambda: self.capture('tool'), row)
        self.demo_tool_button = self.button('Simulator: next contact', self.demo_tool, row)
        self.button('Remove selected', self.remove_tool_sample, row)
        self.button('Clear contacts', self.clear_tool_samples, row)
        box.addLayout(row)
        row = QHBoxLayout()
        row.addWidget(self.label('Maximum contact residual'))
        self.residual = number(.05, 2, .5, 2, ' mm')
        row.addWidget(self.residual)
        self.solve_button = self.button('Calculate TCP', self.solve_tool, row)
        box.addLayout(row)
        self.tool_summary = self.paragraph('Capture at least four contacts to calculate the TCP.')
        box.addWidget(self.tool_summary)
        group = self.group('APPLY TOOL TO CONTROLLER')
        form = QFormLayout(group)
        self.slot = QSpinBox()
        self.slot.setRange(1, 14)
        self.physical_id = QSpinBox()
        self.physical_id.setRange(0, 14)
        self.physical_id.setValue(1)
        self.load_number = QSpinBox()
        self.load_number.setRange(0, 99)
        form.addRow(self.label('Tool coordinate slot'), self.slot)
        form.addRow(self.label('Physical tool ID'), self.physical_id)
        form.addRow(self.label('Existing payload profile number'), self.load_number)
        self.apply_button = self.pushbutton('Apply TCP to this slot')
        self.apply_button.clicked.connect(self.apply_tool)
        form.addRow(self.apply_button)
        form.addRow(self.paragraph('Applying updates this controller slot. Configure the correct payload mass and center of gravity in the controller first.'))
        box.addWidget(group)
        self.add_tab(tab, '1  Tool calibration')
        for field in (self.slot, self.physical_id, self.load_number):
            field.valueChanged.connect(self.invalidate_application)

    def build_planes(self):
        tab = QWidget()
        box = QVBoxLayout(tab)
        box.addWidget(self.paragraph('2  /  WORKPIECE PLANES'))
        box.addWidget(self.paragraph('Calculate a plane here, then explicitly apply it to a controller Wobj slot below to use it for WebUI Wobj jogging. Saving a local plane alone does not update the controller.'))
        box.addWidget(self.paragraph('With the calibrated tool applied, touch three points on the workpiece: O (origin), X (along +X), and Y (on the +Y side of the plane). Space points widely. +Z follows X × Y; verify it points away from the surface. You can teach separate pick and place planes.'))
        self.plane_name = QLineEdit(self.tr('Pick plane'))
        self.plane_name.textEdited.connect(self.mark_plane_name_edited)
        form = QFormLayout()
        form.addRow(self.label('New plane name'), self.plane_name)
        box.addLayout(form)
        self.point_labels = []
        self.plane_capture_buttons = []
        for i, name in enumerate(['O — origin', 'X — point along +X', 'Y — point toward +Y']):
            row = QHBoxLayout()
            lab = QLabel()
            lab.setWordWrap(True)
            lab.setTextInteractionFlags(Qt.TextSelectableByMouse)
            row.addWidget(lab, 1)
            b = self.button('Capture', lambda checked=False, n=i: self.capture(f'plane{n}'), row)
            self.plane_capture_buttons.append(b)
            self.point_labels.append(lab)
            box.addLayout(row)
        row = QHBoxLayout()
        self.demo_plane_button = self.button('Simulator: next plane point', self.demo_plane, row)
        self.button('Clear points', self.clear_plane_points, row)
        self.add_plane_button = self.button('Calculate & add plane', self.add_plane, row)
        box.addLayout(row)
        self.plane_list = QListWidget()
        self.plane_list.currentRowChanged.connect(self.plane_selected)
        box.addWidget(self.plane_list, 1)
        self.plane_summary = self.paragraph('No workplanes taught yet.')
        box.addWidget(self.plane_summary)
        upload = self.group('APPLY PLANE FOR WEBUI WOBJ JOGGING')
        upload_box = QVBoxLayout(upload)
        upload_row = QHBoxLayout()
        upload_row.addWidget(self.label('Target controller Wobj slot'))
        self.workpiece_slot = QSpinBox()
        self.workpiece_slot.setRange(1, 14)
        upload_row.addWidget(self.workpiece_slot)
        upload_box.addLayout(upload_row)
        self.plane_upload_preview = self.paragraph('Select a saved plane to preview its controller coordinates.')
        upload_box.addWidget(self.plane_upload_preview)
        self.plane_replace_confirm = self.checkbox('I reviewed this plane and agree to replace and select the target controller Wobj slot.')
        self.plane_replace_confirm.toggled.connect(self.refresh)
        upload_box.addWidget(self.plane_replace_confirm)
        self.plane_apply_guidance = self.paragraph('Connect to the robot before applying a workpiece plane.')
        upload_box.addWidget(self.plane_apply_guidance)
        self.apply_plane_button = self.button('Apply plane to controller', self.apply_plane, upload_box)
        self.plane_upload_status = self.paragraph('No plane has been applied by this app in this connection.')
        upload_box.addWidget(self.plane_upload_status)
        upload_box.addWidget(self.paragraph('Applying sends no motion. After verification, select Wobj jogging in WebUI. For this app’s XYZ arrows, use base workpiece 0; the app still converts the selected plane to base coordinates.'))
        self.workpiece_slot.valueChanged.connect(self.invalidate_plane_upload)
        box.addWidget(upload)
        row = QHBoxLayout()
        self.button('Remove plane', self.remove_plane, row)
        self.button('Continue to XYZ controls →', lambda: self.tabs.setCurrentIndex(2), row)
        box.addLayout(row)
        self.plane_list.setMinimumHeight(70)
        self.plane_scroll = QScrollArea()
        self.plane_scroll.setWidgetResizable(True)
        self.plane_scroll.setFrameShape(QScrollArea.NoFrame)
        self.plane_scroll.setStyleSheet('QScrollArea { background-color: #101820; border: none; }')
        self.plane_scroll.viewport().setStyleSheet('background-color: #101820;')
        tab.setObjectName('workpieceContent')
        tab.setStyleSheet('QWidget#workpieceContent { background-color: #101820; }')
        self.plane_scroll.setWidget(tab)
        self.add_tab(self.plane_scroll, '2  Workpiece planes')

    def mark_plane_name_edited(self, _text):
        self.plane_name_edited = True

    def build_motion(self):
        tab = QWidget()
        box = QVBoxLayout(tab)
        box.addWidget(self.paragraph('3  /  CARTESIAN MOTION'))
        self.plane_combo = QComboBox()
        self.plane_combo.currentIndexChanged.connect(self.motion_plane_changed)
        box.addWidget(self.plane_combo)
        self.plane_readout = self.paragraph('Plane XYZ: —')
        box.addWidget(self.plane_readout)
        row = QHBoxLayout()
        self.lock_button = self.button('Lock current tool orientation', self.lock_orientation, row)
        self.orientation_label = self.label('Orientation not locked')
        self.orientation_label.setWordWrap(True)
        row.addWidget(self.orientation_label)
        box.addLayout(row)
        self.orientation_guidance = self.paragraph('Connect to the robot before locking tool orientation.')
        box.addWidget(self.orientation_guidance)
        tcp_row = QHBoxLayout()
        self.apply_saved_tcp_button = self.button('Apply saved TCP', self.apply_tool, tcp_row)
        self.tcp_status = self.paragraph('Saved TCP is not verified on the controller.')
        tcp_row.addWidget(self.tcp_status, 1)
        box.addLayout(tcp_row)
        frame_row = QHBoxLayout()
        self.base_frame_button = self.button('Use base workpiece 0', self.select_base_frame, frame_row)
        self.frame_status = self.paragraph('Connect to verify the controller workpiece.')
        frame_row.addWidget(self.frame_status, 1)
        box.addLayout(frame_row)
        box.addWidget(self.paragraph('This button applies base workpiece 0 on the controller without moving the robot. Your saved planes remain available in this app.'))
        limits = self.group('WORKPLANE LIMITS / mm')
        grid = QGridLayout(limits)
        grid.addWidget(self.label('Axis'), 0, 0)
        grid.addWidget(self.label('Minimum'), 0, 1)
        grid.addWidget(self.label('Maximum'), 0, 2)
        self.bounds = []
        for i, (axis, low, high) in enumerate([('X', -200, 200), ('Y', -200, 200), ('Z', 0, 200)]):
            pair = [number(-2000, 2000, low), number(-2000, 2000, high)]
            self.bounds.append(pair)
            grid.addWidget(self.label(axis), i+1, 0)
            for j, widget in enumerate(pair):
                grid.addWidget(widget, i+1, j+1)
                widget.valueChanged.connect(self.disarm)
        box.addWidget(limits)
        self.position_limit_status = self.paragraph('Connect and select a plane to check the current position against its limits.')
        box.addWidget(self.position_limit_status)
        self.axes_verified = self.checkbox('I verified this plane’s +X, +Y, +Z directions and travel limits')
        self.axes_verified.toggled.connect(self.disarm)
        box.addWidget(self.axes_verified)
        self.arm = self.checkbox('Enable Cartesian motion')
        self.arm.toggled.connect(self.arm_changed)
        box.addWidget(self.arm)
        row = QHBoxLayout()
        self.step = number(.1, 10, 1, 2, ' mm')
        self.speed = number(.5, 10, 3, 1, ' %')
        self.hold = self.checkbox('Hold arrows to repeat 1 mm steps')
        row.addWidget(self.label('Step'))
        row.addWidget(self.step)
        row.addWidget(self.label('Speed'))
        row.addWidget(self.speed)
        row.addWidget(self.hold)
        box.addLayout(row)
        pad = QGridLayout()
        self.arrows = []
        for text, axis, direction, r, c in [('↑  +Y',1,1,0,1), ('←  −X',0,-1,1,0),
                                           ('+X  →',0,1,1,2), ('−Y  ↓',1,-1,2,1),
                                           ('+Z  ↑',2,1,0,3), ('−Z  ↓',2,-1,2,3)]:
            b = self.pushbutton(text)
            b.setMinimumHeight(50)
            b.setFocusPolicy(Qt.NoFocus)
            b.pressed.connect(lambda a=axis, d=direction, button=b: self.arrow_pressed(a, d, button))
            b.released.connect(self.arrow_released)
            pad.addWidget(b, r, c)
            self.arrows.append(b)
        pad.addWidget(self.label('XY plane'), 1, 1, alignment=Qt.AlignCenter)
        box.addLayout(pad)
        box.addWidget(self.paragraph('Hold an arrow to move up to the step limit. Release to request an immediate stop, including during the current step. Hold mode repeats 1 mm steps only while pressed. Each move preserves the locked orientation.'))
        row = QHBoxLayout()
        self.targets = [number(-2000, 2000, 0) for _ in range(3)]
        for name, target in zip('XYZ', self.targets):
            row.addWidget(self.label(name))
            row.addWidget(target)
        self.move_button = self.button('Move to XYZ', self.move_target, row)
        box.addLayout(row)
        box.addWidget(self.paragraph('Absolute moves are straight lines, limited to 100 mm per command. Travel limits cover the TCP only; check the arm, tool and fixtures for clearance. This prototype provides positioning, not an automatic gripper sequence.'))
        box.addStretch()
        self.add_tab(tab, '3  XYZ controls')

    def build_pick_place(self):
        """Build an explicit static-array recipe and inspectable base-TCP route."""
        tab = QWidget()
        tab.setObjectName('pickPlaceContent')
        tab.setStyleSheet('QWidget#pickPlaceContent { background-color: #101820; }')
        box = QVBoxLayout(tab)
        box.addWidget(self.paragraph('4  /  STATIC PICK AND PLACE'))
        box.addWidget(self.paragraph('Teach every part position in the selected workpiece plane. Set one fixed destination in robot-base TCP coordinates, then add any intermediate base TCP waypoints needed to keep each straight MoveL segment clear and within 100 mm. No vision, gripper, collision planner, or automatic approach/retract is included.'))
        row = QHBoxLayout()
        self.task_rows = QSpinBox(); self.task_rows.setRange(1, 10); self.task_rows.setValue(3)
        self.task_columns = QSpinBox(); self.task_columns.setRange(1, 10); self.task_columns.setValue(2)
        row.addWidget(self.label('Array rows'))
        row.addWidget(self.task_rows)
        row.addWidget(self.label('Array columns'))
        row.addWidget(self.task_columns)
        box.addLayout(row)
        self.task_points = QTableWidget(6, 5)
        self.task_points.setHorizontalHeaderLabels([self.tr(text) for text in ('Row', 'Column', 'Plane X / mm', 'Plane Y / mm', 'Plane Z / mm')])
        self.task_points.horizontalHeader().setStretchLastSection(True)
        self.task_points.setAlternatingRowColors(True)
        self.task_points.setMaximumHeight(255)
        box.addWidget(self.task_points)
        box.addWidget(self.paragraph('Edit each cell directly in plane coordinates. Row-major order is the pick order. Changing the dimensions regenerates a regular 30 mm grid while preserving overlapping edited cells.'))
        self.task_destination = [number(-2000, 2000, value, 2) for value in (300, 0, 200, 180, 0, 0)]
        destination_box = self.group('FIXED DESTINATION / ROBOT BASE TCP · mm, °')
        destination_grid = QGridLayout(destination_box)
        for index, (name, widget) in enumerate(zip(('X', 'Y', 'Z', 'RX', 'RY', 'RZ'), self.task_destination)):
            destination_grid.addWidget(self.label(name), index//3, (index%3)*2)
            destination_grid.addWidget(widget, index//3, (index%3)*2+1)
        box.addWidget(destination_box)
        route_box = self.group('MANUAL TRANSIT WAYPOINTS / ROBOT BASE TCP · mm, °')
        route_layout = QVBoxLayout(route_box)
        self.task_waypoints = QTableWidget(0, 6)
        self.task_waypoints.setHorizontalHeaderLabels([self.tr(text) for text in ('X / mm', 'Y / mm', 'Z / mm', 'RX / °', 'RY / °', 'RZ / °')])
        self.task_waypoints.horizontalHeader().setStretchLastSection(True)
        self.task_waypoints.setAlternatingRowColors(True)
        self.task_waypoints.setMaximumHeight(170)
        route_layout.addWidget(self.task_waypoints)
        route_buttons = QHBoxLayout()
        self.button('Add waypoint', self.add_task_waypoint, route_buttons)
        self.button('Remove selected waypoint', self.remove_task_waypoint, route_buttons)
        route_layout.addLayout(route_buttons)
        box.addWidget(route_box)
        box.addWidget(self.paragraph('For each part, the route is: source → listed waypoints → fixed destination. For the next part, it retraces the waypoint list in reverse → next source. Each arrow is a separate straight MoveL and must pass the selected plane limits and 100 mm segment cap. The destination is visited once per source point. Gripper/I/O is not controlled.'))
        self.task_review = QTextEdit()
        self.task_review.setReadOnly(True)
        self.task_review.setMaximumHeight(190)
        self.task_review.setPlainText(self.tr('Select a calibrated base-reference workpiece plane to preview the route.'))
        box.addWidget(self.task_review)
        self.task_ack = self.checkbox('I reviewed every base TCP waypoint, straight segment, travel bound and destination')
        box.addWidget(self.task_ack)
        controls = QHBoxLayout()
        self.task_start = self.button('Start pick/place sequence', self.start_task, controls)
        self.task_cancel = self.button('Cancel sequence', self.cancel_task, controls)
        box.addLayout(controls)
        box.addStretch()
        self.task_rows.valueChanged.connect(self.resize_task_array)
        self.task_columns.valueChanged.connect(self.resize_task_array)
        self.task_points.itemChanged.connect(self.task_changed)
        for widget in self.task_destination:
            widget.valueChanged.connect(self.task_changed)
        self.task_waypoints.itemChanged.connect(self.task_changed)
        self.task_ack.toggled.connect(self.refresh)
        self.resize_task_array()
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setStyleSheet('QScrollArea { background-color: #101820; border: none; }')
        scroll.viewport().setStyleSheet('background-color: #101820;')
        scroll.setWidget(tab)
        self.add_tab(scroll, '4  Pick/place')

    def log(self, text, **values):
        if text:
            message = text if isinstance(text, Message) else Message(text, values)
            self.log_entries.append((time.strftime('%H:%M:%S'), message))
            self.render_logs()

    def resize_task_array(self, *_):
        """Resize a row-major array, keeping the overlapping operator edits."""
        old = getattr(self, 'task_points', None)
        previous = {}
        if old is not None:
            for row in range(old.rowCount()):
                items = [old.item(row, column) for column in range(5)]
                if all(item is not None for item in items):
                    try:
                        key = (int(items[0].text())-1, int(items[1].text())-1)
                        previous[key] = [float(items[column].text()) for column in (2, 3, 4)]
                    except ValueError:
                        # Ignore a partially edited row while preserving the
                        # remaining valid cell coordinates during resize.
                        continue
        rows, columns = self.task_rows.value(), self.task_columns.value()
        self.task_points.blockSignals(True)
        self.task_points.setRowCount(rows*columns)
        for row in range(rows):
            for column in range(columns):
                index = row*columns+column
                values = previous.get((row, column), [column*30., row*30., 0.])
                cells = [str(row+1), str(column+1), *(f'{value:.3f}' for value in values)]
                for cell_column, value in enumerate(cells):
                    item = QTableWidgetItem(value)
                    if cell_column < 2:
                        item.setFlags(item.flags() & ~Qt.ItemIsEditable)
                    self.task_points.setItem(index, cell_column, item)
        self.task_points.blockSignals(False)
        self.task_changed()

    def task_point_values(self):
        return [[float(self.task_points.item(row, column).text()) for column in (2, 3, 4)]
                for row in range(self.task_points.rowCount())]

    def task_waypoint_values(self):
        return [[float(self.task_waypoints.item(row, column).text()) for column in range(6)]
                for row in range(self.task_waypoints.rowCount())]

    def task_data(self):
        return dict(rows=self.task_rows.value(), columns=self.task_columns.value(),
                    points=validate_array(self.task_rows.value(), self.task_columns.value(), self.task_point_values()),
                    destination=[widget.value() for widget in self.task_destination],
                    waypoints=validate_poses(self.task_waypoint_values()))

    def add_task_waypoint(self):
        row = self.task_waypoints.rowCount()
        self.task_waypoints.blockSignals(True)
        self.task_waypoints.insertRow(row)
        for column, value in enumerate(self.task_destination):
            self.task_waypoints.setItem(row, column, QTableWidgetItem(f'{value.value():.2f}'))
        self.task_waypoints.blockSignals(False)
        self.task_changed()

    def remove_task_waypoint(self):
        row = self.task_waypoints.currentRow()
        if row >= 0:
            self.task_waypoints.removeRow(row)
            self.task_changed()

    def task_changed(self, *_):
        if not hasattr(self, 'task_review'):
            return
        self.task_ack.setChecked(False)
        self.update_task_review()
        if hasattr(self, 'session_dirty'):
            self.schedule_autosave()

    def update_task_review(self):
        try:
            plane = self.current_plane()
            if plane is None or plane.reference != 'robot_base':
                self.task_review.setText(self.tr('Select a calibrated base-reference workpiece plane to preview the route.'))
                return
            data = self.task_data()
            path = build_trajectory(plane, data['points'], data['destination'], data['waypoints'],
                                    self.orientation or [0., 0., 0.])
            lines = [self.tr('{count} source points · {moves} straight MoveL segments · row-major order',
                             count=data['rows']*data['columns'], moves=len(path))]
            lines.extend(f'{index+1:03d}. {self.task_display_name(name)}: '+', '.join(f'{value:.2f}' for value in pose)
                         for index, (name, pose) in enumerate(path))
            self.task_review.setText('\n'.join(lines))
        except (ValueError, TypeError, IndexError) as exc:
            self.task_review.setText(self.tr('The array or route contains an invalid coordinate: {detail}', detail=str(exc)))

    def task_display_name(self, name):
        for prefix in ('Pick', 'Place'):
            if name.startswith(prefix+' '):
                return self.tr('{kind} {index}', kind=self.tr(prefix), index=name[len(prefix)+1:])
        for prefix in ('Route', 'Return'):
            if name.startswith(prefix+' '):
                return self.tr('{kind} {index}', kind=self.tr(prefix), index=name[len(prefix)+1:])
        return name

    def task_block_reason(self):
        fresh = self.state is not None and time.monotonic()-self.state_time < 1.2
        if not self.connected:
            return 'Connect to the robot or simulator before starting the sequence.'
        if not fresh:
            return 'Waiting for fresh robot feedback.'
        if not self.tool_applied:
            return 'Verify or apply the saved TCP before starting the sequence.'
        if self.current_plane() is None or self.current_plane().reference != 'robot_base':
            return 'Select a valid base-reference workpiece plane before starting the sequence.'
        if self.orientation is None:
            return 'Lock the current tool orientation in XYZ controls before starting the sequence.'
        if not self.arm.isChecked():
            return 'Enable Cartesian motion and verify the plane directions and limits first.'
        if self.client.busy or not self.state['stopped'] or not self.state['program_stopped']:
            return 'Wait until the robot and controller program are stationary.'
        if not self.state['safe'] or any(self.state['errors']):
            return 'Robot reports a stop, collision, or fault'
        if not self.state['enabled']:
            return 'Enable the robot using the controller first'
        if self.state['mode'] != 1:
            return 'Use controller manual mode for teaching jog'
        if self.state['user'] != 0:
            return 'Select base workpiece 0 in the controller before Cartesian motion.'
        if not self.task_ack.isChecked():
            return 'Review every route pose and check the sequence acknowledgment before starting.'
        try:
            data = self.task_data()
            path = build_trajectory(self.current_plane(), data['points'], data['destination'],
                                    data['waypoints'], self.orientation)
            previous = self.state['tcp']
            for _, pose in path:
                target = self.current_plane().from_base(pose[:3])
                checked_target(self.current_plane(), previous, target, pose[3:], self.limits())
                previous = pose
        except (ValueError, TypeError, IndexError) as exc:
            return 'A route target violates plane limits or the 100 mm segment cap.'
        return None

    def start_task(self):
        reason = self.task_block_reason()
        if reason:
            self.log('Pick/place sequence blocked: {reason}', reason=reason)
            self.refresh()
            return
        data = self.task_data()
        self.task_path = build_trajectory(self.current_plane(), data['points'], data['destination'],
                                          data['waypoints'], self.orientation)
        self.task_index = 0
        self.task_running = True
        self.teach_enabled.setChecked(False)
        self.log('Pick/place sequence started: {count} source points.', count=data['rows']*data['columns'])
        self.send_task_segment()

    def send_task_segment(self):
        if not self.task_running:
            return
        if self.task_index >= len(self.task_path):
            self.task_running = False
            self.task_path = []
            self.log('Pick/place sequence completed. No gripper I/O was performed.')
            self.refresh()
            return
        name, pose = self.task_path[self.task_index]
        self.log('Sequence move {index}/{count}: {name}', index=self.task_index+1,
                 count=len(self.task_path), name=self.task_display_name(name))
        plane_target = self.current_plane().from_base(pose[:3])
        if not self.send_move(plane_target, speed=self.speed.value(), orientation=pose[3:]):
            self.task_running = False
            self.refresh()

    def cancel_task(self):
        if self.task_running or self.client.pending and self.client.pending[1] == 'move':
            self.task_running = False
            self.task_path = []
            self.stop(reason=Message('Pick/place sequence canceled by operator'))

    def diagnostic(self, text):
        if isinstance(text, Message):
            self.log(text)
        elif text:
            self.log('SDK diagnostic (original): {detail}', detail=text)

    def render_logs(self):
        self.messages.setPlainText('\n'.join(f'{stamp}  {self.tr(message)}' for stamp, message in self.log_entries))
        bar = self.messages.verticalScrollBar()
        bar.setValue(bar.maximum())

    def failure(self, text):
        was_connected = self.connected
        self.task_running = False
        self.task_path = []
        self.repeat_direction = None
        self.disarm()
        self.teach_enabled.setChecked(False)
        message = text if isinstance(text, Message) else Message('Operation failed: {detail}', {'detail': text})
        self.log('ERROR: {reason}', reason=message)
        # Never continue on a command or telemetry failure; drop pending SDK work.
        self.connected = False
        self.tool_applied = False
        self.client.shutdown()
        if was_connected:
            self.client.stop()
        self.refresh()

    def move_rejected(self, message):
        self.task_running = False
        self.task_path = []
        self.disarm()
        self.log('Move not sent: {reason}. Connection remains open.', reason=message)
        self.refresh()

    def disarm(self, *_):
        self.arrow_released()
        self.arm.setChecked(False)
        self.refresh()

    def teaching_changed(self, enabled):
        if enabled:
            self.disarm()
            self.orientation = None
            self.saved_orientation = None
            self.schedule_autosave()
        elif self.active_jog:
            self.end_jog(self.active_jog[0] + 1)
        self.refresh()

    def invalidate_application(self, *_):
        self.tool_applied = False
        self.disarm()

    def backend_changed(self, index):
        if not hasattr(self, 'arm'):
            return
        self.settings.setValue('connection/backend', 'simulation' if index == 0 else 'robot')
        self.settings.sync()
        if self.connected or self.client.pending:
            self.stop(reason=Message('Backend changed'))
            self.log('Backend changed. Click Connect to open the selected connection.')
        self.state = None
        self.state_time = 0.
        self.refresh()

    def connect_robot(self):
        if self.connected:
            self.stop(reason=Message('Disconnect button'))
            return
        simulation = self.mode.currentIndex() == 0
        if self.source is not None and self.source != ('simulation' if simulation else 'robot'):
            self.log('Clear/load calibration from the matching backend before applying it; simulator and robot data cannot be mixed.')
        self.tool_applied = False
        self.last_plane_application = None
        self.plane_replace_confirm.setChecked(False)
        self.disarm()
        self.client.connect_robot(simulation=simulation, ip=self.ip.text().strip(), sdk_path=self.sdk.text().strip())
        self.refresh()

    def completed(self, method, state):
        previously_applied = self.tool_applied
        self.state, self.state_time = state, time.monotonic()
        self.tool_applied = bool(state.get('tcp_verified') and self.tool_result
                                 and self.source == self.backend_name()
                                 and state['tool'] == self.slot.value()
                                 and all(abs(a-b) <= .001 for a, b in zip(state['tool_offset'], self.tool_result['tool'])))
        if method == 'connect':
            self.connected = True
            self.log('Connected to {destination}', destination=Message('simulator') if self.client.simulation else self.client.ip)
            self.verify_saved_tcp()
        elif method == 'apply_tool':
            self.log('Calibrated TCP applied. Continue to workpiece planes.')
        elif method == 'select_base_frame':
            self.log('Base workpiece 0 selected and verified. Lock orientation and verify plane directions/limits before enabling motion.')
        elif method == 'apply_plane':
            self.last_plane_application = state['applied_workpiece']
            self.plane_replace_confirm.setChecked(False)
            self.log('Plane {name} applied and verified in controller Wobj{slot}. Use WebUI Wobj jogging with this slot.',
                     name=self.last_plane_application['name'], slot=self.last_plane_application['slot'])
        elif method == 'verify_tool':
            if self.tool_applied:
                self.log('Controller TCP matches saved calibration in tool slot {slot}; no controller settings were changed.', slot=self.slot.value())
            else:
                self.log('Controller TCP does not match saved calibration. Active tool {actual}, expected tool {expected}; active offset {offset}. Click Apply saved TCP.',
                         actual=state['tool'], expected=self.slot.value(), offset=str(state['tool_offset']))
        elif method == 'capture':
            kind, self.capture_kind = self.capture_kind, None
            if kind == 'tool':
                if self.source != self.backend_name() and self.tool_samples:
                    self.log('Clear contacts before switching calibration backend')
                else:
                    self.source = self.backend_name()
                    self.samples_reference = 'robot_base'
                    self.tool_samples.append(state['flange'])
                    self.invalidate_tool()
            elif kind and kind.startswith('plane'):
                i = int(kind[-1])
                if i <= len(self.plane_points):
                    self.plane_points = self.plane_points[:i] + [state['tcp'][:3]]
                    self.draft_reference = 'robot_base'
            elif kind == 'orientation':
                self.orientation = state['tcp'][3:]
                self.saved_orientation = list(self.orientation)
                self.orientation_label.setText(' / '.join(f'{v:.2f}°' for v in self.orientation))
                self.log('Current orientation locked')
        elif method == 'demo_pose' and self.demo_pending:
            kind, self.demo_pending = self.demo_pending, None
            self.capture(kind)
        elif method == 'move':
            self.log('Plane jog stopped on release' if state.get('plane_jog_stopped') else 'Linear move completed')
            if self.task_running:
                self.task_index += 1
                QTimer.singleShot(80, self.send_task_segment)
            elif self.plane_arrow_held and self.repeat_direction and self.arm.isChecked():
                self.plane_repeat_timer.start(80)
        if not state['safe'] or any(state['errors']):
            self.stop(reason=Message('Controller stop/fault: safe={safe}, errors={errors}',
                                     dict(safe=state['safe'], errors=str(state['errors']))))
            if state['errors'] == [1, 2]:
                self.log('Controller fault 1/2: linear target point error (including tool mismatch). Review and reset the fault in WebUI, then reconnect with the calibrated tool and base workpiece 0 selected.')
            else:
                self.log('Controller reports stop/fault; motion disabled')
        if previously_applied and not self.tool_applied:
            self.orientation = None
            self.axes_verified.setChecked(False)
            self.disarm()
        self.refresh()
        if method not in ('read', 'servo_state'):
            self.schedule_autosave()

    def backend_name(self):
        return 'simulation' if self.client.simulation else 'robot'

    def poll(self):
        if self.connected and not self.client.busy and not self.repeat_direction:
            self.client.call('read')
        self.refresh()

    def refresh(self):
        if not hasattr(self, 'arm'):
            return
        idle = self.connected and not self.client.busy
        fresh = self.state is not None and time.monotonic()-self.state_time < 1.2
        safe = fresh and self.state['safe'] and not any(self.state['errors'])
        self.connect_button.setText(self.tr('Disconnect' if self.connected else 'Connect'))
        self.connect_button.setEnabled(self.connected or not self.client.busy)
        self.mode.setEnabled(not self.client.busy)
        for w in (self.ip, self.sdk):
            w.setEnabled(not self.connected and not self.client.busy)
        simulation = self.client.simulation if self.connected or self.client.pending else self.mode.currentIndex() == 0
        self.status.setText(self.tr('SIMULATOR' if simulation else 'REAL ROBOT') +
                            self.tr(' · connected' if self.connected else ' · disconnected') +
                            (self.tr(' · command running') if self.client.busy and not self.jog_request_active() else ''))
        if self.state:
            p = self.state['tcp']
            self.readout.setText(self.tr('TCP / base (mm, °)\n{pose}\nTool {tool} · Workpiece {user}\n{enabled} · faults {errors}',
                                        pose=', '.join(f'{v:.2f}' for v in p), tool=self.state['tool'], user=self.state['user'],
                                        enabled=Message('Enabled' if self.state['enabled'] else 'Disabled'), errors=str(self.state['errors'])))
            plane = self.current_plane()
            if plane and plane.reference == 'robot_base':
                self.plane_readout.setText(self.tr('Plane XYZ / mm: {xyz}', xyz=', '.join(f'{v:.3f}' for v in plane.from_base(p[:3]))))
            else:
                self.plane_readout.setText(self.tr('Plane XYZ: —'))
            for index, label in enumerate(self.joint_labels):
                label.setText(f"J{index+1}  {self.state['joints'][index]:.2f}°")
        else:
            self.readout.setText(self.tr('TCP in robot base: —'))
            self.plane_readout.setText(self.tr('Plane XYZ: —'))
            for index, label in enumerate(self.joint_labels):
                label.setText(f'J{index+1}')
        jog_reason = self.jog_block_reason(fresh)
        self.jog_status.setText(self.tr('Jog active; release the button to stop.' if self.active_jog else
                                        jog_reason or 'Ready to jog. Hold a button to jog; release to stop.'))
        active_button = self.active_jog[1] if self.active_jog else None
        for b in self.joint_buttons + self.base_jog_buttons:
            b.setEnabled((idle and not jog_reason) or (b is active_button and b.isDown()))
            b.setToolTip(self.tr(jog_reason) if jog_reason else '')
        for b in (self.capture_tool_button, self.demo_tool_button):
            b.setEnabled(idle)
        self.demo_tool_button.setVisible(simulation)
        self.demo_plane_button.setVisible(simulation)
        self.demo_plane_button.setEnabled(idle and self.tool_applied and len(self.plane_points) < 3)
        for i, b in enumerate(self.plane_capture_buttons):
            b.setEnabled(idle and self.tool_applied and i <= len(self.plane_points))
        for i, label in enumerate(self.point_labels):
            name = self.tr(['O — origin', 'X — point along +X', 'Y — point toward +Y'][i])
            label.setText(name+': '+(', '.join(f'{v:.3f}' for v in self.plane_points[i]) if i < len(self.plane_points) else '—'))
        self.add_plane_button.setEnabled(len(self.plane_points) == 3 and self.tool_applied)
        self.solve_button.setEnabled(len(self.tool_samples) >= 4)
        self.apply_button.setEnabled(idle and self.tool_result is not None and self.source == self.backend_name())
        self.apply_saved_tcp_button.setEnabled(self.apply_button.isEnabled())
        self.tcp_status.setText(self.tr('TCP verified on controller · tool {slot}', slot=self.slot.value()) if self.tool_applied
                                else self.tr('Saved TCP is not verified on the controller.'))
        plane = self.current_plane()
        if hasattr(self, 'task_review'):
            self.update_task_review()
        reteach_reason = self.plane_reteach_reason()
        self.workpiece_slot.setEnabled(not self.client.busy and not self.arm.isChecked())
        self.plane_replace_confirm.setEnabled(not self.client.busy and plane is not None and reteach_reason is None)
        self.plane_replace_confirm.setToolTip(self.tr(reteach_reason) if reteach_reason else '')
        plane_apply_reason = self.plane_apply_block_reason()
        self.apply_plane_button.setText(self.tr('Reteach plane before applying' if reteach_reason else 'Apply plane to controller'))
        self.apply_plane_button.setEnabled(plane_apply_reason is None)
        self.apply_plane_button.setToolTip(self.tr(plane_apply_reason) if plane_apply_reason else '')
        self.plane_apply_guidance.setText(self.tr(plane_apply_reason or 'Ready to apply the reviewed plane to controller Wobj{slot}.',
                                                slot=self.workpiece_slot.value()))
        if plane is None:
            self.plane_upload_preview.setText(self.tr('Select a saved plane to preview its controller coordinates.'))
        elif plane.reference != 'robot_base':
            self.plane_upload_preview.setText(self.tr(reteach_reason))
        else:
            pose = plane.pose()
            self.plane_upload_preview.setText(self.tr('Plane {name} → Wobj{slot} · reference: robot base\nXYZ / mm: {xyz}\nRX RY RZ / °: {angles}',
                                                    name=plane.name, slot=self.workpiece_slot.value(),
                                                    xyz=', '.join(f'{v:.3f}' for v in pose[:3]), angles=', '.join(f'{v:.3f}' for v in pose[3:])))
        self.plane_upload_status.setText(self.tr('Last apply verified: {name} → Wobj{slot}.',
                                              name=self.last_plane_application['name'], slot=self.last_plane_application['slot'])
                                         if self.connected and self.last_plane_application
                                         else self.tr('No plane has been applied by this app in this connection.'))
        self.base_frame_button.setEnabled(bool(idle and safe and self.state['stopped'] and not self.active_jog))
        self.frame_status.setText(self.tr('Controller workpiece 0 (base) selected.') if self.connected and self.state and self.state['user'] == 0
                                  else self.tr('Controller workpiece {slot} selected. Click “Use base workpiece 0”.', slot=self.state['user'])
                                  if self.connected and self.state else self.tr('Connect to verify the controller workpiece.'))
        lock_reason = self.orientation_lock_block_reason()
        self.lock_button.setEnabled(lock_reason is None)
        self.lock_button.setToolTip(self.tr(lock_reason) if lock_reason else '')
        self.orientation_guidance.setText(self.tr(lock_reason or 'Ready to lock the current tool orientation for the selected plane.'))
        plane = self.current_plane()
        if plane is not None and plane.reference != 'robot_base':
            self.position_limit_status.setText(self.tr('This plane was captured with the old coordinate-frame bug. Load a recovered base-frame calibration or reteach the plane before using it.'))
        elif self.connected and fresh and plane is not None:
            try:
                xyz = check_current_plane_position(plane, self.state['tcp'], self.limits())
                self.position_limit_status.setText(self.tr('Current TCP is inside plane limits: X={x:.3f}, Y={y:.3f}, Z={z:.3f} mm.',
                                                          x=xyz[0], y=xyz[1], z=xyz[2]))
            except OperatorValueError as exc:
                self.position_limit_status.setText(self.tr(exc.message))
        else:
            self.position_limit_status.setText(self.tr('Waiting for fresh robot feedback.' if self.connected and not fresh
                                                      else 'Connect and select a plane to check the current position against its limits.'))
        enabled = bool(idle and safe and self.arm.isChecked() and self.tool_applied)
        for b in self.arrows + [self.move_button]:
            # Leave pressed arrows enabled during an in-flight increment so release is delivered.
            if not (b.isDown() and self.connected and self.arm.isChecked()):
                b.setEnabled(enabled)
        for w in (self.plane_combo, self.step, self.speed, self.hold, self.slot, self.physical_id, self.load_number):
            w.setEnabled(not self.arm.isChecked() and not self.client.busy)
        for w in (self.joint_limit, self.base_linear_limit, self.base_rotation_limit):
            w.setEnabled(not self.client.busy and self.active_jog is None)
        for w in (self.jog_speed, self.jog_speed_slider, self.base_jog_speed, self.base_jog_speed_slider):
            w.setEnabled(not self.client.busy and self.active_jog is None)
        reason = self.task_block_reason()
        self.task_start.setEnabled(reason is None and not self.task_running)
        self.task_start.setToolTip(self.tr(reason) if reason else '')
        self.task_cancel.setEnabled(self.task_running)
        for widget in (self.task_rows, self.task_columns, self.task_points, *self.task_destination,
                       self.task_waypoints, self.task_ack):
            widget.setEnabled(not self.client.busy and not self.task_running)
        self.task_review.setToolTip(self.tr(reason) if reason else '')

    def jog_block_reason(self, fresh):
        if not self.connected:
            return 'Connect to a robot or simulator to jog.'
        if not fresh:
            return 'Waiting for fresh robot feedback.'
        if not self.state['safe'] or any(self.state['errors']):
            return 'Robot reports a stop, collision, or fault'
        if not self.state['enabled']:
            return 'Enable the robot using the controller first'
        if self.state['mode'] != 1:
            return 'Use controller manual mode for teaching jog'
        if not self.state['program_stopped']:
            return "Stop the controller's Lua program before using this application"
        if not self.state['stopped']:
            return 'Robot must be stationary (exit drag mode before capturing)'
        if not self.teach_enabled.isChecked():
            return 'Enable teaching jog controls'
        return None

    def jog_request_active(self):
        jog_methods = {'start_jog', 'stop_jog'}
        pending_method = self.client.pending[1] if self.client.pending else None
        queued_method = self.client.queued_command[0] if self.client.queued_command else None
        return bool(self.active_jog or self.client.stream_token is not None or pending_method in jog_methods or queued_method in jog_methods)

    def capture(self, kind):
        if kind == 'tool' and self.tool_samples and self.samples_reference != 'robot_base':
            self.log('Clear the old contacts before adding base-frame TCP calibration samples. The existing TCP offset can still be verified and used.')
            return
        if kind.startswith('plane') and self.plane_points and self.draft_reference != 'robot_base':
            self.log('Clear the old draft plane points before capturing new base-frame points.')
            return
        if not self.connected or self.client.busy:
            return
        if kind.startswith('plane') and not self.tool_applied:
            return
        self.capture_kind = kind
        self.client.call('capture')

    def invalidate_tool(self):
        self.tool_result = None
        self.tool_applied = False
        self.planes.clear()
        self.plane_points.clear()
        self.draft_reference = 'robot_base'
        self.orientation = None
        self.saved_orientation = None
        self.summary_status = 'changed'
        self.sync_planes()
        self.render_details()
        self.disarm()
        self.schedule_autosave()

    def remove_tool_sample(self):
        i = self.sample_list.currentRow()
        if i >= 0:
            self.tool_samples.pop(i)
            self.invalidate_tool()

    def clear_tool_samples(self):
        self.tool_samples.clear()
        self.samples_reference = 'robot_base'
        self.source = None
        self.invalidate_tool()

    def solve_tool(self):
        try:
            result = calibrate_pivot(self.tool_samples, self.residual.value())
            self.tool_result = asdict(result)
            self.summary_status = 'calculated'
            self.render_details()
            self.log('TCP calculated')
        except ValueError as exc:
            self.log(error_message(exc))
        self.refresh()
        self.schedule_autosave()

    def apply_tool(self):
        if self.tool_result and self.source == self.backend_name():
            self.disarm()
            self.client.call('apply_tool', dict(tool=self.tool_result['tool'], slot=self.slot.value(),
                                               tool_id=self.physical_id.value(), load=self.load_number.value()))

    def verify_saved_tcp(self):
        if self.connected and self.tool_result and self.source == self.backend_name():
            self.client.call('verify_tool', dict(tool=self.tool_result['tool'], slot=self.slot.value()))

    def invalidate_plane_upload(self, *_):
        self.plane_replace_confirm.setChecked(False)
        self.refresh()

    def plane_reteach_reason(self):
        plane = self.current_plane()
        if plane is not None and plane.reference != 'robot_base':
            return Message('Plane “{name}” needs reteaching: its original capture reference is unknown. Keep your TCP. Select this plane and click Remove plane, click Clear points, then capture O/X/Y again and Calculate & add plane. Checking the replacement acknowledgment cannot repair these old points.',
                           dict(name=plane.name))
        return None

    def plane_apply_block_reason(self):
        reteach_reason = self.plane_reteach_reason()
        if reteach_reason is not None:
            return reteach_reason
        if not self.connected:
            return Message('Connect to the robot before applying a workpiece plane.')
        if self.client.busy:
            return Message('Wait for the current controller command to finish before applying a workpiece plane.')
        if self.current_plane() is None:
            return Message('Select a saved plane before applying it to the controller.')
        if self.state is None or time.monotonic() - self.state_time >= 1.2:
            return Message('Waiting for fresh robot feedback.')
        if any(self.state['errors']):
            return Message('Controller fault {main}/{sub}: review and reset it in WebUI before applying a plane.',
                           dict(main=self.state['errors'][0], sub=self.state['errors'][1]))
        if not self.state['safe']:
            return Message('Robot reports a stop, collision, or fault')
        if self.active_jog or not self.state['stopped']:
            return Message('Release jogging, exit drag mode, and wait for the robot to stop before applying a plane.')
        if not self.state['program_stopped']:
            return Message("Stop the controller's Lua program before using this application")
        if not self.tool_applied:
            return Message('Verify or apply the saved TCP in Tool calibration or XYZ controls before applying a plane.')
        if not self.plane_replace_confirm.isChecked():
            return Message('Review the plane preview and target Wobj slot, then check the replacement acknowledgment above this button.')
        return None

    def apply_plane(self):
        reason = self.plane_apply_block_reason()
        if reason is not None:
            self.log('Cannot apply plane: {reason}', reason=reason)
            self.refresh()
            return
        plane = self.current_plane()
        self.disarm()
        self.teach_enabled.setChecked(False)
        self.orientation = None
        self.saved_orientation = None
        self.orientation_label.setText(self.tr('Orientation not locked'))
        self.axes_verified.setChecked(False)
        self.client.call('apply_plane', dict(plane=asdict(plane), slot=self.workpiece_slot.value()))
        self.refresh()

    def select_base_frame(self):
        if not self.connected or self.client.busy or self.active_jog:
            return
        self.disarm()
        self.teach_enabled.setChecked(False)
        self.orientation = None
        self.saved_orientation = None
        self.orientation_label.setText(self.tr('Orientation not locked'))
        self.axes_verified.setChecked(False)
        self.client.call('select_base_frame')
        self.schedule_autosave()
        self.refresh()

    def demo_tool(self):
        self.demo_pending = 'tool'
        self.client.call('demo_pose', dict(kind='pivot', index=len(self.tool_samples)))

    def demo_plane(self):
        i = len(self.plane_points)
        if i < 3:
            self.demo_pending = f'plane{i}'
            self.client.call('demo_pose', dict(kind='plane', index=i))

    def clear_plane_points(self):
        self.plane_points.clear()
        self.draft_reference = 'robot_base'
        self.refresh()
        self.schedule_autosave()

    def add_plane(self):
        try:
            plane = Plane.teach(self.plane_name.text(), self.plane_points, self.draft_reference)
            if any(p.name == plane.name for p in self.planes):
                raise OperatorValueError('Choose a unique plane name')
            self.planes.append(plane)
            self.plane_points.clear()
            self.sync_planes()
            self.plane_list.setCurrentRow(len(self.planes)-1)
            self.log('Added workplane: {name}', name=plane.name)
        except ValueError as exc:
            self.log(error_message(exc))
        self.refresh()

    def sync_planes(self):
        self.plane_list.clear()
        self.plane_combo.clear()
        for plane in self.planes:
            self.plane_list.addItem(plane.name)
            self.plane_combo.addItem(plane.name)
        self.schedule_autosave()

    def plane_selected(self, i):
        if 0 <= i < len(self.planes):
            self.plane_combo.setCurrentIndex(i)
        self.render_plane_summary()

    def remove_plane(self):
        i = self.plane_list.currentRow()
        if 0 <= i < len(self.planes):
            self.planes.pop(i)
            self.sync_planes()
            self.disarm()

    def current_plane(self):
        i = self.plane_combo.currentIndex()
        return self.planes[i] if 0 <= i < len(self.planes) else None

    def motion_plane_changed(self, *_):
        self.orientation = None
        self.saved_orientation = None
        self.orientation_label.setText(self.tr('Orientation not locked'))
        self.axes_verified.setChecked(False)
        self.plane_replace_confirm.setChecked(False)
        self.disarm()

    def lock_orientation(self):
        reason = self.orientation_lock_block_reason()
        if reason:
            self.log(reason)
            return
        self.teach_enabled.setChecked(False)
        self.disarm()
        self.capture('orientation')

    def orientation_lock_block_reason(self):
        if not self.connected:
            return 'Connect to the robot before locking tool orientation.'
        if self.client.busy:
            return 'Wait for the current controller command to finish before locking tool orientation.'
        if self.active_jog:
            return 'Release the jog button and wait for the robot to stop before locking tool orientation.'
        if not self.tool_applied:
            return 'Click “Apply saved TCP” below, or apply TCP in Tool calibration. The controller must confirm the saved tool slot and offset.'
        if self.current_plane() is None:
            return 'Select a workplane before locking tool orientation.'
        if self.current_plane().reference != 'robot_base':
            return 'This plane was captured with the old coordinate-frame bug. Load a recovered base-frame calibration or reteach the plane before using it.'
        return None

    def arm_changed(self, checked):
        if not checked:
            self.arrow_released()
            if self.task_running:
                self.cancel_task()
        if checked:
            missing = next((reason for ready, reason in (
                (self.connected, 'Connect to the robot before enabling Cartesian motion.'),
                (self.tool_applied, 'Click “Apply saved TCP” in XYZ controls and wait for controller confirmation.'),
                (self.state is not None and self.state.get('user') == 0, 'Click “Use base workpiece 0” before enabling Cartesian motion.'),
                (self.current_plane() is not None, 'Select a workplane before enabling Cartesian motion.'),
                (self.current_plane() is not None and self.current_plane().reference == 'robot_base', 'This plane was captured with the old coordinate-frame bug. Load a recovered base-frame calibration or reteach the plane before using it.'),
                (self.orientation is not None, 'Lock the current tool orientation after selecting the workplane.'),
                (self.axes_verified.isChecked(), 'Review the plane directions and travel limits, then check “I verified this plane’s +X, +Y, +Z directions and travel limits”.'),
            ) if not ready), None)
            if missing:
                self.log('Cannot enable Cartesian motion: {reason}', reason=Message(missing))
                self.arm.setChecked(False)
            else:
                self.teach_enabled.setChecked(False)
        self.refresh()

    def begin_base_jog(self, axis, direction, button):
        # Native StartJOG takes controller percent, including rotational axes.
        # Keep this ceiling independent of joint jog speed; it is not TCP mm/s.
        limit = self.base_rotation_limit.value() if axis >= 4 else self.base_linear_limit.value()
        self.begin_jog(2, axis, direction, limit, button, speed=self.base_jog_speed.value())

    def begin_jog(self, ref, axis, direction, distance, button, speed=None):
        if not self.teach_enabled.isChecked() or self.active_jog or not self.connected:
            return
        if ref == 0 or axis >= 4:
            self.orientation = None
            self.saved_orientation = None
            self.disarm()
        self.active_jog = (ref, button)
        accepted = self.client.call('start_jog', dict(ref=ref, axis=axis, direction=direction,
                                                       distance=distance, speed=self.jog_speed.value() if speed is None else speed))
        if not accepted:
            self.active_jog = None
            self.log('Jog could not start; wait for the current controller command to finish.')
        self.refresh()

    def end_jog(self, stop_ref):
        if not self.active_jog or self.active_jog[0] != stop_ref - 1:
            return
        self.active_jog[1].setDown(False)
        self.active_jog = None
        if not self.client.call('stop_jog', dict(stop_ref=stop_ref)):
            self.log('Jog stop could not be queued; use STOP or Esc.')
        self.refresh()

    def limits(self):
        return [[w.value() for w in pair] for pair in self.bounds]

    def arrow_pressed(self, axis, direction, button=None):
        if self.plane_arrow_held or not self.arm.isChecked() or self.client.busy:
            return
        self.plane_arrow_held = True
        self.plane_arrow_button = button
        self.repeat_direction = (axis, direction) if self.hold.isChecked() else None
        if not self.send_step(axis, direction):
            self.arrow_released()

    def arrow_released(self):
        self.repeat_direction = None
        self.plane_arrow_held = False
        self.plane_repeat_timer.stop()
        if self.plane_arrow_button is not None:
            self.plane_arrow_button.setDown(False)
            self.plane_arrow_button = None
        self.client.end_plane_jog()

    def repeat_step(self):
        if self.plane_arrow_held and self.repeat_direction and self.arm.isChecked():
            self.send_step(*self.repeat_direction)

    def send_step(self, axis, direction):
        plane = self.current_plane()
        if plane and self.state:
            xyz = plane.from_base(self.state['tcp'][:3])
            xyz[axis] += direction * (1 if self.hold.isChecked() else self.step.value())
            return self.send_move(xyz, jog=self.plane_arrow_held)
        return False

    def move_target(self):
        self.send_move([w.value() for w in self.targets])

    def send_move(self, target, speed=None, jog=False, orientation=None):
        if not self.arm.isChecked() or self.client.busy:
            return False
        try:
            if time.monotonic()-self.state_time > 1:
                raise OperatorValueError('Position is stale; wait for live feedback')
            plane = self.current_plane()
            orientation = self.orientation if orientation is None else orientation
            checked_target(plane, self.state['tcp'], target, orientation, self.limits())
            args = dict(plane=asdict(plane), target=target, orientation=orientation,
                        bounds=self.limits(), speed=self.speed.value() if speed is None else speed)
            accepted = self.client.start_plane_jog(args) if jog else self.client.call('move', args)
        except ValueError as exc:
            self.log(error_message(exc))
            self.disarm()
            accepted = False
        self.refresh()
        return accepted

    def stop(self, *, reason=None):
        self.log('Stop requested: {reason}', reason=reason or Message('Manual stop request'))
        self.task_running = False
        self.task_path = []
        self.repeat_direction = None
        self.active_jog = None
        self.disarm()
        self.teach_enabled.setChecked(False)
        self.connected = False
        self.tool_applied = False
        self.orientation = None
        # Kill SDK work before sending Stop: no late request can move after the stop.
        self.client.shutdown()
        self.client.stop()
        self.log('Motion disabled. Reconnect and verify the saved TCP to continue.')
        self.refresh()

    def setup_autosave(self):
        for widget in (self.residual, self.slot, self.physical_id, self.load_number,
                       self.workpiece_slot,
                       self.step, self.speed, *self.targets, self.jog_speed, self.base_jog_speed,
                       self.joint_limit, self.base_linear_limit, self.base_rotation_limit,
                       *(widget for pair in self.bounds for widget in pair)):
            widget.valueChanged.connect(self.schedule_autosave)
        self.plane_name.textChanged.connect(self.schedule_autosave)
        self.plane_combo.currentIndexChanged.connect(self.schedule_autosave)
        self.hold.toggled.connect(self.schedule_autosave)
        self.tabs.currentChanged.connect(self.schedule_autosave)
        self.task_rows.valueChanged.connect(self.schedule_autosave)
        self.task_columns.valueChanged.connect(self.schedule_autosave)

    def calibration_document(self):
        plane = self.current_plane()
        return dict(schema=3, controller='V3.8.25-QX', source=self.source,
                    samples=self.tool_samples, tolerance=self.residual.value(), tool=self.tool_result,
                    samples_reference=self.samples_reference,
                    slot=self.slot.value(), tool_id=self.physical_id.value(), load=self.load_number.value(),
                    workpiece_slot=self.workpiece_slot.value(),
                    planes=[asdict(p) for p in self.planes], bounds=self.limits(),
                    selected_plane=plane.name if plane else None,
                    plane_draft=dict(name=self.plane_name.text(), points=self.plane_points, reference=self.draft_reference),
                    xyz=dict(step=self.step.value(), speed=self.speed.value(), hold=self.hold.isChecked(),
                             targets=[w.value() for w in self.targets],
                             orientation_reference=self.orientation if self.orientation is not None else self.saved_orientation),
                    jog=dict(joint_speed=self.jog_speed.value(), tcp_speed=self.base_jog_speed.value(),
                             joint_limit=self.joint_limit.value(), linear_limit=self.base_linear_limit.value(),
                             rotation_limit=self.base_rotation_limit.value()),
                    task=self.task_data(),
                    active_tab=self.tabs.currentIndex())

    def schedule_autosave(self, *_):
        if not self.restoring_calibration:
            self.session_dirty = True
            self.autosave_timer.start()

    def autosave(self):
        self.autosave_timer.stop()
        if not self.session_dirty or self.restoring_calibration:
            return
        try:
            write_calibration(self.session_path, self.calibration_document())
            self.session_dirty = False
        except (OSError, ValueError) as exc:
            self.log('Cannot autosave calibration to {path}: {reason}', path=str(self.session_path), reason=error_message(exc))

    def restore_calibration(self, data):
        was_restoring = self.restoring_calibration
        self.restoring_calibration = True
        try:
            self.disarm()
            self.teach_enabled.setChecked(False)
            self.tool_applied = False
            self.tool_samples, self.source = data['samples'], data['source']
            self.samples_reference = data['samples_reference']
            self.tool_result, self.planes = data['tool'], data['planes']
            self.residual.setValue(data['tolerance'])
            self.slot.setValue(data['slot'])
            self.physical_id.setValue(data['tool_id'])
            self.load_number.setValue(data['load'])
            self.workpiece_slot.setValue(data['workpiece_slot'])
            self.last_plane_application = None
            self.plane_replace_confirm.setChecked(False)
            for pair, values in zip(self.bounds, data['bounds']):
                for widget, value in zip(pair, values):
                    widget.setValue(value)
            self.sync_planes()
            index = self.plane_combo.findText(data['selected_plane']) if data['selected_plane'] is not None else -1
            self.plane_combo.setCurrentIndex(index)
            self.plane_list.setCurrentRow(index)
            draft = data['plane_draft']
            self.plane_points = draft['points']
            self.draft_reference = draft['reference']
            self.plane_name.setText(draft['name'] if draft['name'] is not None else self.tr('Pick plane'))
            self.plane_name_edited = draft['name'] is not None
            xyz = data['xyz']
            self.step.setValue(xyz['step'])
            self.speed.setValue(xyz['speed'])
            self.hold.setChecked(xyz['hold'])
            jog = data['jog']
            self.jog_speed.setValue(jog['joint_speed'])
            self.base_jog_speed.setValue(jog['tcp_speed'])
            self.joint_limit.setValue(jog['joint_limit'])
            self.base_linear_limit.setValue(jog['linear_limit'])
            self.base_rotation_limit.setValue(jog['rotation_limit'])
            task = data['task']
            self.task_rows.setValue(task['rows'])
            self.task_columns.setValue(task['columns'])
            self.resize_task_array()
            self.task_points.blockSignals(True)
            for row, point in enumerate(task['points']):
                for column, value in zip((2, 3, 4), point):
                    self.task_points.item(row, column).setText(f'{value:.3f}')
            self.task_points.blockSignals(False)
            for widget, value in zip(self.task_destination, task['destination']):
                widget.setValue(value)
            self.task_waypoints.blockSignals(True)
            self.task_waypoints.setRowCount(0)
            for pose in task['waypoints']:
                row = self.task_waypoints.rowCount()
                self.task_waypoints.insertRow(row)
                for column, value in enumerate(pose):
                    self.task_waypoints.setItem(row, column, QTableWidgetItem(f'{value:.2f}'))
            self.task_waypoints.blockSignals(False)
            self.task_ack.setChecked(False)
            for widget, value in zip(self.targets, xyz['targets']):
                widget.setValue(value)
            self.orientation = None
            self.saved_orientation = xyz['orientation_reference']
            self.axes_verified.setChecked(False)
            self.tabs.setCurrentIndex(min(data['active_tab'], self.tabs.count()-1))
            self.summary_status = 'loaded' if self.tool_result else 'initial'
            self.render_details()
            self.refresh()
            if any(p.reference != 'robot_base' for p in self.planes):
                self.log('Saved plane reference is unknown because older versions recorded Wobj-relative poses as base. Keep the TCP, clear the plane draft, and reteach three plane points with this version.')
        finally:
            self.restoring_calibration = was_restoring

    def restore_startup_calibration(self):
        path = self.session_path if self.session_path.exists() else self.legacy_path
        if path is None or not path.exists():
            return
        try:
            data = read_calibration(path)
            self.restore_calibration(data)
            if path != self.session_path:
                self.session_dirty = True
            self.log('Restored calibration from {path}. Connect to verify TCP, then lock orientation and verify limits before motion.', path=str(path))
        except (OSError, ValueError, KeyError, TypeError, OverflowError) as exc:
            self.log('Cannot restore saved calibration: {reason}', reason=error_message(exc))

    def save(self):
        path, _ = QFileDialog.getSaveFileName(self, self.tr('Save calibration'), str(ROOT/'calibration.json'),
                                             'JSON (*.json)', options=QFileDialog.DontUseNativeDialog)
        if path:
            try:
                write_calibration(path, self.calibration_document())
                self.log('Saved {path}', path=path)
            except (OSError, ValueError) as exc:
                self.log(error_message(exc))

    def load(self):
        if self.client.busy or self.arm.isChecked():
            self.log('Finish motion and disable Cartesian controls before loading')
            return
        path, _ = QFileDialog.getOpenFileName(self, self.tr('Load calibration'), str(ROOT),
                                             'JSON (*.json)', options=QFileDialog.DontUseNativeDialog)
        if not path:
            return
        try:
            self.restore_calibration(read_calibration(path))
            self.schedule_autosave()
            self.verify_saved_tcp()
            self.log('Loaded {path}; motion stays disabled until TCP is verified and orientation locked.', path=path)
        except (OSError, ValueError, KeyError, TypeError, OverflowError) as exc:
            self.log('Cannot load calibration: {reason}', reason=error_message(exc))
        self.refresh()

    def event(self, event):
        if event.type() == QEvent.WindowDeactivate and hasattr(self, 'arm'):
            if self.active_jog:
                self.end_jog(self.active_jog[0] + 1)
            elif self.plane_arrow_held or self.client.plane_jog_token is not None:
                self.disarm()  # Release the arrow; keep the verified connection.
            elif ((self.client.pending and self.client.pending[1] == 'move') or self.repeat_direction):
                self.stop(reason=Message('Application focus lost during a plane move'))
            elif self.arm.isChecked():
                self.disarm()
        return super().event(event)

    def eventFilter(self, watched, event):
        if (event.type() == QEvent.MouseButtonRelease and event.button() == Qt.LeftButton
                and self.plane_arrow_held and watched is not self.plane_arrow_button):
            self.arrow_released()  # Release redirected by scrolling/reparenting.
        if (event.type() == QEvent.MouseButtonRelease and event.button() == Qt.LeftButton
                and self.active_jog and watched is not self.active_jog[1]):
            # Normally the button's own released signal stops the jog. This
            # catches release events redirected by scrolling or focus changes.
            self.active_jog[1].setDown(False)
            self.end_jog(self.active_jog[0] + 1)
        return super().eventFilter(watched, event)

    def closeEvent(self, event):
        self.autosave()
        if self.connected or self.client.busy:
            self.stop(reason=Message('Application closing'))
            if self.client.stop_process:
                # Keep the stop subprocess alive until its result is received.
                self.client.stop_process.waitForFinished(5500)
        self.client.shutdown()
        self.autosave_timer.stop()
        event.accept()


def main():
    parser = argparse.ArgumentParser(description='FR3 Cartesian calibration studio')
    parser.add_argument('--smoke-test', action='store_true', help='Create the UI and exit without connecting')
    parser.add_argument('--screenshot', type=Path, help='Save a screenshot during the smoke test')
    parser.add_argument('--language', choices=LANGUAGES, help='Initial display language (en or zh_CN)')
    args = parser.parse_args()
    app = QApplication(sys.argv[:1])
    app.setStyle('Fusion')
    window = MainWindow(language=args.language)
    window.show()
    if args.smoke_test:
        def finish():
            if args.screenshot:
                window.grab().save(str(args.screenshot))
            window.close()
            app.quit()
        QTimer.singleShot(500, finish)
    sys.exit(app.exec())
