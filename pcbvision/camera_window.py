"""Industrial camera live/capture window; all SDK access stays in one thread."""
from __future__ import annotations

import queue
import math
import threading
import time

import cv2
import numpy as np
from PySide6.QtCore import QThread, QTimer, Qt, Signal, Slot
from PySide6.QtGui import QColor, QImage, QPainter, QPen, QPixmap
from PySide6.QtWidgets import (
    QCheckBox, QComboBox, QDoubleSpinBox, QFormLayout, QGroupBox,
    QHBoxLayout, QInputDialog, QLabel, QLineEdit, QPushButton, QScrollArea, QSizePolicy, QSlider, QVBoxLayout,
    QWidget,
)


def _make_camera():
    from .hikvision import HikvisionCamera
    return HikvisionCamera()


class _ParameterSlider(QWidget):
    """A Dear ImGui-style drag control with a precise numeric editor.

    The camera SDK ranges are often floating point and can span several orders
    of magnitude, while QSlider works with integers.  Keep the slider normalized
    to 0..1000 and map it to the parameter range; exposure uses logarithmic
    mapping so a wide microsecond range remains useful at both ends.
    """

    valueEdited = Signal(float)

    def __init__(self, key, spin, parent=None):
        super().__init__(parent)
        self.key = key
        self.spin = spin
        self.slider = QSlider(Qt.Horizontal)
        self.slider.setRange(0, 1000)
        self.slider.setSingleStep(1)
        self.slider.setPageStep(50)
        self.slider.setMinimumWidth(145)
        self.slider.setToolTip('Drag to tune this camera parameter; type an exact value at right')
        self.slider.valueChanged.connect(self._slider_changed)
        self.spin.valueChanged.connect(self._spin_changed)
        self.spin.editingFinished.connect(lambda: self.valueEdited.emit(self.spin.value()))
        row = QHBoxLayout(self)
        row.setContentsMargins(0, 0, 0, 0)
        row.setSpacing(6)
        row.addWidget(self.slider, 1)
        row.addWidget(self.spin)

    def set_range(self, minimum, maximum):
        self.spin.setRange(minimum, maximum)
        self._sync_slider(self.spin.value())

    def set_value(self, value):
        self.spin.setValue(value)
        self._sync_slider(value)

    def _normalized(self, value):
        lo, hi = self.spin.minimum(), self.spin.maximum()
        if hi <= lo:
            return 0
        if self.key == 'exposure' and lo > 0 and hi / lo >= 20:
            return round(1000 * (math.log(max(lo, value)) - math.log(lo)) /
                         (math.log(hi) - math.log(lo)))
        return round(1000 * (value - lo) / (hi - lo))

    def _value(self, position):
        lo, hi = self.spin.minimum(), self.spin.maximum()
        if self.key == 'exposure' and lo > 0 and hi / lo >= 20:
            return math.exp(math.log(lo) + position / 1000 * (math.log(hi) - math.log(lo)))
        return lo + position / 1000 * (hi - lo)

    def _sync_slider(self, value):
        self.slider.blockSignals(True)
        self.slider.setValue(max(0, min(1000, self._normalized(value))))
        self.slider.blockSignals(False)

    def _slider_changed(self, position):
        self.spin.blockSignals(True)
        self.spin.setValue(self._value(position))
        self.spin.blockSignals(False)
        self.valueEdited.emit(self.spin.value())

    def _spin_changed(self, value):
        self._sync_slider(value)


class CameraWorker(QThread):
    devices_found = Signal(object)
    state_changed = Signal(bool, bool)
    parameters_ready = Signal(object)
    preview_ready = Signal(object)
    snapshot_ready = Signal(object)
    calibration_frame_ready = Signal(object)
    tuning_done = Signal(object)
    profile_finished = Signal(object)
    message = Signal(str)
    command_done = Signal()

    def __init__(self, camera_factory=_make_camera, parent=None):
        super().__init__(parent)
        self._factory = camera_factory
        self._commands = queue.Queue()
        self._stopping = threading.Event()

    def submit(self, command, value=None):
        self._commands.put((command, value))

    def shutdown(self):
        self._stopping.set()
        self._commands.put(("shutdown", None))

    def run(self):
        camera = None
        live = False
        next_preview = 0.
        try:
            while not self._stopping.is_set():
                try:
                    command, value = self._commands.get(timeout=.02 if live else .1)
                except queue.Empty:
                    command = None
                if command == "shutdown":
                    break
                if command == 'tune':
                    # Scalar SDK controls are writable during acquisition.
                    # Do not stop/start the stream for each drag event, and
                    # report completion separately from modal camera commands.
                    result = {'requested': value, 'parameters': None, 'error': ''}
                    try:
                        if camera is None or not camera.is_open:
                            raise RuntimeError('Open a camera before tuning')
                        camera.apply_parameters(value)
                        result['parameters'] = camera.parameters()
                    except Exception as exc:
                        result['error'] = str(exc)
                        self.message.emit(f'Camera tuning failed: {exc}')
                    self.tuning_done.emit(result)
                    continue
                if command is not None:
                    try:
                        if camera is None:
                            camera = self._factory()
                        if command == "scan":
                            devices = camera.enumerate_devices()
                            self.devices_found.emit(devices)
                            library = getattr(camera, "sdk_library", "")
                            detail = f" SDK: {library}." if library else ""
                            self.message.emit(f"Found {len(devices)} camera(s). Select a device and click Open.{detail}")
                        elif command == "open":
                            camera.open(value)
                            self.parameters_ready.emit(camera.parameters())
                            self.message.emit(f"Camera open: {camera.resolution[0]} × {camera.resolution[1]}. Start live view or take a snapshot.")
                        elif command == "start":
                            camera.start_live()
                            live = True
                            self.message.emit("Live view running")
                        elif command == "stop":
                            camera.stop_live()
                            live = False
                            self.message.emit("Live view stopped")
                        elif command == "snapshot":
                            frame = camera.snapshot()
                            if frame is None:
                                raise RuntimeError("Snapshot returned no image")
                            self.snapshot_ready.emit(frame.copy())
                            self.message.emit(f"Snapshot sent to registration: {frame.shape[1]} × {frame.shape[0]}")
                        elif command == 'calibration_frame':
                            # Always acquire a fresh native image on the SDK
                            # thread, never a resized/painted GUI preview.
                            frame = camera.snapshot()
                            if frame is None:
                                raise RuntimeError('Calibration snapshot returned no image')
                            metadata = dict(value, parameters=camera.parameters())
                            self.calibration_frame_ready.emit({'frame': frame.copy(), 'metadata': metadata})
                            self.message.emit(f'Intrinsic sample captured: {frame.shape[1]} × {frame.shape[0]} (native pixels)')
                        elif command == "parameters":
                            self.parameters_ready.emit(camera.parameters())
                            self.message.emit("Read current camera parameters")
                        elif command in ('profile_save', 'profile_load'):
                            from .camera_profiles import ProfileStore
                            if not camera.is_open:
                                raise RuntimeError('Open a camera before using profiles')
                            store = ProfileStore(value['backend'], value['serial'])
                            parameters = camera.parameters()
                            if command == 'profile_save':
                                data = store.save(value['name'], parameters, camera.resolution,
                                                  overwrite=value.get('overwrite', False))
                            else:
                                data = store.load(value['name'])
                                values = store.values_for_camera(data, parameters, camera.resolution)
                                resume = live
                                try:
                                    camera.stop_live()
                                    camera.apply_parameters(values)
                                finally:
                                    # A rejected/partial apply must leave the UI
                                    # showing actual camera values, and restore
                                    # the previous live state when possible.
                                    try:
                                        parameters = camera.parameters()
                                        self.parameters_ready.emit(parameters)
                                    finally:
                                        if resume:
                                            camera.start_live()
                                store.verify_applied(data, parameters, camera.resolution)
                            self.profile_finished.emit({'action': command, 'name': data['name'],
                                                        'path': str(store.path(data['name']))})
                            verb = 'saved / 已保存' if command == 'profile_save' else 'loaded / 已加载'
                            self.message.emit(f'Camera profile {verb}: {data["name"]} · {store.path(data["name"])}')
                        elif command == "apply":
                            # Some camera nodes are only writable while acquisition is stopped.
                            resume = live
                            try:
                                camera.stop_live()
                                camera.apply_parameters(value)
                                self.message.emit("Camera parameters applied")
                            finally:
                                self.parameters_ready.emit(camera.parameters())
                                if resume:
                                    camera.start_live()
                        elif command == "disconnect":
                            live = False
                            camera.close()
                            self.message.emit("Camera closed")
                    except Exception as exc:
                        if command == "open" and camera is not None:
                            try:
                                camera.close()
                            except Exception:
                                pass
                        live = bool(camera and camera.is_streaming)
                        self.message.emit(f"{command.capitalize()} failed: {exc}")
                    finally:
                        self.state_changed.emit(bool(camera and camera.is_open), live)
                        self.command_done.emit()

                if live and time.monotonic() >= next_preview and not self._stopping.is_set():
                    try:
                        frame = camera.capture_frame(timeout_ms=200)
                        if frame is not None:
                            h, w = frame.shape[:2]
                            if max(h, w) > 1280:
                                scale = 1280 / max(h, w)
                                frame = cv2.resize(frame, (round(w * scale), round(h * scale)), interpolation=cv2.INTER_AREA)
                            self.preview_ready.emit(frame)
                        next_preview = time.monotonic() + 1 / 18
                    except Exception as exc:
                        live = False
                        try:
                            camera.stop_live()
                        except Exception:
                            pass
                        self.message.emit(f"Live view failed: {exc}. Close and reopen the camera to reconnect.")
                        self.state_changed.emit(bool(camera and camera.is_open), False)
        finally:
            if camera is not None:
                try:
                    camera.close()
                except Exception as exc:
                    self.message.emit(f"Camera cleanup: {exc}")


class CameraWindow(QWidget):
    snapshot_ready = Signal(object)
    closed = Signal()

    def __init__(self, parent=None, camera_factory=_make_camera, *,
                 camera_name="Hikvision", preview_only=False, device_serial="",
                 auto_preview=False, log_stdout=False):
        super().__init__(parent, Qt.Window)
        self.device_serial = device_serial
        self.log_stdout = log_stdout
        self._startup_stage = 'scan' if auto_preview else None
        self._devices_received = False
        self.setWindowTitle(f"{camera_name} camera — " + ("fiducial aiming preview" if preview_only else "live view and snapshot"))
        self.resize(1100, 800 if preview_only else 720)
        self._devices = []
        self._is_open = False
        self._is_live = False
        self._busy = False
        self._closing = False
        self._preview_image = QImage()
        self.camera_name = camera_name
        self.intrinsic_window = None
        self._opened_device = None
        self._pending_tunes = {}
        self._tuning = False
        self._profile_pending = None
        self._tune_timer = QTimer(self)
        self._tune_timer.setSingleShot(True)
        self._tune_timer.setInterval(100)
        self._tune_timer.timeout.connect(self._flush_tuning)

        self.preview = QLabel("Open a camera and start live view")
        self.preview.setAlignment(Qt.AlignCenter)
        self.preview.setMinimumSize(500, 320)
        self.preview.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Ignored)
        self.preview.setStyleSheet("background: #171b20; color: #aeb7c2")
        self.status = QLabel("Finding cameras…")
        self.status.setWordWrap(True)
        self.device_combo = QComboBox()
        self.refresh_button = QPushButton("Find cameras")
        self.open_button = QPushButton("Open camera")
        self.disconnect_button = QPushButton("Close camera")
        self.live_button = QPushButton("Start live view")
        self.snapshot_button = QPushButton("Snapshot → registration")
        self.snapshot_button.setVisible(not preview_only)
        self.intrinsic_button = QPushButton('Intrinsic calibration / 相机内参标定…')
        self.intrinsic_button.clicked.connect(self.open_intrinsics)
        self.crosshair = QCheckBox("Center crosshair / 中心十字线")
        self.crosshair.setChecked(preview_only)
        self.crosshair.toggled.connect(self._paint_preview)
        self.read_button = QPushButton("Read parameters")
        self.apply_button = QPushButton("Apply parameters")
        self.parameters_group = QGroupBox()
        self.parameters_group.setStyleSheet('''
          QGroupBox { background: #252d38; color: #edf1f7; border: 1px solid #45586c;
            border-radius: 5px; padding: 8px; }
          QLabel, QCheckBox { color: #edf1f7; background: transparent; }
          QDoubleSpinBox, QComboBox { color: #edf1f7; background: #151d27;
            border: 1px solid #45586c; padding: 4px; }
          QDoubleSpinBox:disabled, QComboBox:disabled, QCheckBox:disabled { color: #8492a3; }
          QPushButton { color: #edf1f7; background: #34485b; padding: 6px;
            border: 1px solid #526c83; border-radius: 3px; }
          QPushButton:disabled { color: #8492a3; background: #263441; }
          QSlider::groove:horizontal { height: 12px; background: #151d27; border-radius: 4px; }
          QSlider::sub-page:horizontal { background: #285e86; border-radius: 4px; }
          QSlider::handle:horizontal { background: #69b5ed; width: 12px; margin: -3px 0; border-radius: 3px; }
          QSlider::handle:horizontal:hover { background: #a4d9ff; }
          QSlider::handle:horizontal:disabled { background: #526170; }
        ''')
        form = QFormLayout(self.parameters_group)
        form.addRow(QLabel('<b>Camera parameters / 相机参数</b>'))
        self.profile_combo = QComboBox()
        self.profile_combo.setPlaceholderText('No profiles yet / 暂无配置')
        self.profile_combo.setToolTip('Choose a named profile, then click Load / 选择配置后点击加载')
        self.profile_load_button = QPushButton('Load / 加载')
        self.profile_save_button = QPushButton('Save / 保存')
        self.profile_save_as_button = QPushButton('Save as… / 另存为…')
        self.profile_load_button.clicked.connect(self._load_profile)
        self.profile_save_button.clicked.connect(self._save_profile)
        self.profile_save_as_button.clicked.connect(self._save_profile_as)
        self.profile_combo.currentIndexChanged.connect(self._update_profile_buttons)
        form.addRow(QLabel('Camera profile / 相机配置'))
        form.addRow(self.profile_combo)
        profile_buttons = QHBoxLayout()
        for button in (self.profile_load_button, self.profile_save_button, self.profile_save_as_button):
            profile_buttons.addWidget(button)
        form.addRow(profile_buttons)
        self.profile_info = QLabel('Open a camera to save/load profiles / 打开相机后保存或加载配置')
        self.profile_info.setWordWrap(True)
        self.profile_info.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Preferred)
        self.profile_info.setTextInteractionFlags(Qt.TextSelectableByMouse)
        form.addRow(self.profile_info)
        self.auto_exposure = QCheckBox("Auto exposure")
        self.auto_exposure.toggled.connect(self._auto_exposure_changed)
        form.addRow(self.auto_exposure)
        self.manual_exposure_button = QPushButton("Manual exposure / 手动曝光")
        self.manual_exposure_button.setToolTip("Turn off auto exposure, then drag Exposure to tune live / 关闭自动曝光后拖动实时调节")
        self.manual_exposure_button.clicked.connect(self._choose_manual_exposure)
        form.addRow(self.manual_exposure_button)
        self.resolution_combo = QComboBox()
        self.resolution_combo.setToolTip("Camera acquisition preset, not display scaling; click Apply parameters")
        form.addRow(QLabel("Resolution / 分辨率"))
        form.addRow(self.resolution_combo)
        self.float_controls = {}
        self.parameter_sliders = {}
        for key, label, suffix in (("exposure", "Exposure / 曝光", " µs"),
                                   ("gain", "Gain / 增益", " dB"), ("gamma", "Gamma / 伽马", ""),
                                   ("contrast", "Contrast / 对比度", "")):
            spin = QDoubleSpinBox()
            spin.setDecimals(3)
            spin.setSuffix(suffix)
            spin.setKeyboardTracking(False)
            if camera_name == "MindVision" and key in ("gain", "gamma", "contrast"):
                spin.setDecimals(0)
                spin.setSuffix(" SDK units" if key == "gain" else "")
            if camera_name == "MindVision" and key in ("gamma", "contrast"):
                spin.setToolTip("Native SDK scale (100 is neutral); edits select the gamma/contrast LUT")
            self.float_controls[key] = spin
            control = _ParameterSlider(key, spin)
            control.valueEdited.connect(lambda value, name=key: self._queue_tuning(name, value))
            self.parameter_sliders[key] = control
            form.addRow(QLabel(label))
            form.addRow(control)
        tuning_hint = QLabel('Drag to tune live; type for precision. / 拖动实时调节，也可输入精确值。')
        tuning_hint.setWordWrap(True)
        form.addRow(tuning_hint)
        self.reverse_x = QCheckBox("Mirror X")
        self.reverse_y = QCheckBox("Mirror Y")
        form.addRow(self.reverse_x)
        form.addRow(self.reverse_y)
        form.addRow(self.read_button)
        form.addRow(self.apply_button)
        self._capabilities = {}

        sidebar = QVBoxLayout()
        sidebar.addWidget(self.device_combo)
        for button in (self.refresh_button, self.open_button, self.disconnect_button,
                       self.live_button, self.snapshot_button):
            sidebar.addWidget(button)
        sidebar.addWidget(self.intrinsic_button)
        sidebar.addWidget(self.parameters_group)
        sidebar.addWidget(self.crosshair)
        hint = QLabel(
            "Align the fiducial with the crosshair / 十字线对准标记。"
            "Image center has an offset from TCP; use for rough aiming."
            if preview_only else
            "Snapshots use a fresh full-resolution frame and go directly to the main image area. Click Register there to apply the CAD overlay.")
        hint.setWordWrap(True)
        hint.setMaximumWidth(300)
        sidebar.addWidget(hint)
        sidebar.addStretch()
        row = QHBoxLayout()
        row.addWidget(self.preview, 1)
        sidebar_widget = QWidget()
        sidebar_widget.setLayout(sidebar)
        sidebar_scroll = QScrollArea()
        sidebar_scroll.setWidgetResizable(True)
        sidebar_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        sidebar_scroll.setMinimumWidth(380)
        sidebar_scroll.setMaximumWidth(430)
        sidebar_scroll.setWidget(sidebar_widget)
        row.addWidget(sidebar_scroll)
        layout = QVBoxLayout(self)
        layout.addLayout(row, 1)
        layout.addWidget(self.status)

        self.worker = CameraWorker(camera_factory, self)
        self.worker.devices_found.connect(self._set_devices)
        self.worker.parameters_ready.connect(self._set_parameters)
        self.worker.state_changed.connect(self._set_state)
        self.worker.preview_ready.connect(self._show_preview)
        self.worker.snapshot_ready.connect(self._deliver_snapshot)
        self.worker.calibration_frame_ready.connect(self._deliver_calibration_frame)
        self.worker.tuning_done.connect(self._tuning_done)
        self.worker.profile_finished.connect(self._profile_finished)
        self.worker.message.connect(self._message)
        self.worker.command_done.connect(self._command_done)
        self.worker.finished.connect(self._finish_close)
        self.refresh_button.clicked.connect(lambda: self._submit("scan"))
        self.open_button.clicked.connect(self._open_camera)
        self.disconnect_button.clicked.connect(lambda: self._submit("disconnect"))
        self.live_button.clicked.connect(lambda: self._submit("stop" if self._is_live else "start"))
        self.snapshot_button.clicked.connect(lambda: self._submit("snapshot"))
        self.read_button.clicked.connect(lambda: self._submit("parameters"))
        self.apply_button.clicked.connect(self._apply_parameters)
        self.worker.start()
        self._submit("scan")

    def _submit(self, command, value=None):
        if self._busy or self._closing:
            return
        if command in ('disconnect', 'open', 'scan', 'apply', 'parameters'):
            self._pending_tunes.clear()
            self._tune_timer.stop()
        self._busy = True
        self.status.setText(f"{command.capitalize()}…")
        self._update_controls()
        self.worker.submit(command, value)

    @Slot()
    def _command_done(self):
        self._busy = False
        self._update_controls()
        if self._closing:
            return
        if self._profile_pending is not None:
            self._advance_profile_operation()
            return
        if self._is_open and self._pending_tunes:
            self._tune_timer.start()
        if self._startup_stage == 'scan':
            self._startup_stage = None
            if not self._devices_received:
                return  # Preserve the SDK scan error instead of hiding it.
            if 0 <= self.device_combo.currentIndex() < len(self._devices):
                self._startup_stage = 'open'
                self._open_camera()
            else:
                self._message(f"Camera {self.device_serial or '(unique device)'} not selected. Find cameras and select one manually. / 未找到指定相机，请手动选择。")
        elif self._startup_stage == 'open':
            self._startup_stage = None
            if self._is_open:
                self._submit('start')

    @Slot(str)
    def _message(self, text):
        self.status.setText(text)
        if self.log_stdout:
            print(f'{time.strftime("%H:%M:%S")}  {text}', flush=True)

    @Slot(object)
    def _set_devices(self, devices):
        self._devices_received = True
        self._devices = devices
        self.device_combo.clear()
        self.device_combo.addItems([item["name"] for item in devices])
        if self.device_serial:
            index = next((i for i, d in enumerate(devices) if d.get('sn') == self.device_serial), -1)
            self.device_combo.setCurrentIndex(index)
        elif self._startup_stage == 'scan' and len(devices) != 1:
            # Never choose an arbitrary device when several cameras are present.
            self.device_combo.setCurrentIndex(-1)

    def _open_camera(self):
        index = self.device_combo.currentIndex()
        if 0 <= index < len(self._devices):
            self._opened_device = self._devices[index]
            self._submit("open", self._devices[index]["dev_info"])

    def open_intrinsics(self):
        if self.intrinsic_window is None:
            from .intrinsic_window import IntrinsicWindow
            self.intrinsic_window = IntrinsicWindow(self)
            self.intrinsic_window.closed.connect(self._intrinsics_closed)
        self.intrinsic_window.show()
        self.intrinsic_window.raise_()
        self.intrinsic_window.activateWindow()
        self._update_controls()

    def _intrinsics_closed(self):
        self.intrinsic_window = None
        self._update_controls()

    def capture_intrinsic_frame(self):
        if self._busy or not self._is_open or not self._opened_device:
            raise RuntimeError('Open the camera and wait for it to become idle / 请先打开相机并等待')
        self._submit('calibration_frame', {'backend': self.camera_name.lower(),
                                         'serial': self._opened_device.get('sn', ''),
                                         'camera_name': self._opened_device['name']})

    @Slot(object)
    def _deliver_calibration_frame(self, payload):
        if self.intrinsic_window is not None and not self._closing:
            self.intrinsic_window.accept_capture(payload)

    @Slot(bool, bool)
    def _set_state(self, opened, live):
        changed = opened != self._is_open
        self._is_open, self._is_live = opened, live
        if not opened:
            self._preview_image = QImage()
            self.preview.clear()
            self.preview.setText("Camera closed")
        if changed:
            self._refresh_profiles()
        self._update_controls()

    def _update_controls(self):
        enabled = not self._busy and not self._closing and self._profile_pending is None
        self.device_combo.setEnabled(enabled and not self._is_open)
        self.refresh_button.setEnabled(enabled and not self._is_open)
        self.open_button.setEnabled(enabled and not self._is_open and bool(self._devices))
        for button in (self.disconnect_button, self.live_button, self.snapshot_button):
            button.setEnabled(enabled and self._is_open)
        self.parameters_group.setEnabled(enabled and self._is_open)
        self.intrinsic_button.setEnabled(enabled and self._is_open)
        if self.intrinsic_window is not None:
            self.intrinsic_window.update_capture_available(enabled and self._is_open)
        self.live_button.setText("Stop live view" if self._is_live else "Start live view")
        self._update_profile_buttons()

    def _profile_store(self):
        from .camera_profiles import ProfileStore
        return ProfileStore(self.camera_name.lower(), (self._opened_device or {}).get('sn', ''))

    def _refresh_profiles(self, selected=None):
        selected = selected or self.profile_combo.currentText()
        self.profile_combo.clear()
        if not self._is_open:
            self.profile_info.setText('Open a camera to save/load profiles / 打开相机后保存或加载配置')
            return
        try:
            store = self._profile_store()
            names, errors = store.list_profiles()
            self.profile_combo.addItems(names)
            index = self.profile_combo.findText(selected)
            if index >= 0:
                self.profile_combo.setCurrentIndex(index)
            text = 'Profiles / 配置目录: '+str(store.directory)
            if errors:
                text += '\nUnreadable profile(s) / 无法读取的配置:\n'+'\n'.join(errors)
            self.profile_info.setText(text)
            self.profile_info.setToolTip(text)
        except Exception as exc:
            self.profile_info.setText(str(exc))
        self._update_profile_buttons()

    def _update_profile_buttons(self, *_):
        if not hasattr(self, 'profile_info'):
            return
        ready = self._is_open and not self._busy and not self._closing and self._profile_pending is None
        ready = ready and bool((self._opened_device or {}).get('sn'))
        selected = self.profile_combo.currentIndex() >= 0
        self.profile_combo.setEnabled(ready)
        self.profile_load_button.setEnabled(ready and selected)
        self.profile_save_button.setEnabled(ready and selected)
        self.profile_save_as_button.setEnabled(ready)

    def _save_profile_as(self):
        dialog = QInputDialog(self)
        dialog.setWindowTitle('Save camera profile / 保存相机配置')
        dialog.setLabelText('Profile name / 配置名称')
        dialog.setInputMode(QInputDialog.TextInput)
        dialog.setTextEchoMode(QLineEdit.Normal)
        dialog.setStyleSheet('QDialog, QLabel { background: #252d38; color: #edf1f7; }'
                            'QLineEdit { background: #151d27; color: #edf1f7; padding: 6px; }'
                            'QPushButton { background: #34485b; color: #edf1f7; padding: 6px; }')
        if dialog.exec() == QInputDialog.Accepted:
            self._start_profile_operation('profile_save', dialog.textValue(), overwrite=False)

    def _save_profile(self):
        self._start_profile_operation('profile_save', self.profile_combo.currentText(), overwrite=True)

    def _load_profile(self):
        self._start_profile_operation('profile_load', self.profile_combo.currentText())

    def _start_profile_operation(self, command, name, *, overwrite=False):
        if self._closing or self._busy or not self._is_open or self._profile_pending is not None:
            return
        try:
            from .camera_profiles import profile_name
            store = self._profile_store()
            name = profile_name(name)
            self._profile_pending = (command, dict(store.identity, name=name, overwrite=overwrite))
            if command == 'profile_load':
                # Load replaces pending edits. Any already-running write is
                # allowed to finish first, then the profile wins.
                self._pending_tunes.clear()
            self._tune_timer.stop()
            self._update_controls()
            self.status.setText('Waiting for camera tuning before profile operation… / 等待相机参数写入后处理配置…')
            self._advance_profile_operation()
        except Exception as exc:
            self._profile_pending = None
            self._update_controls()
            self._message(f'Camera profile failed / 配置操作失败: {exc}')

    def _advance_profile_operation(self):
        if self._profile_pending is None or self._closing or self._busy or self._tuning:
            return
        if not self._is_open:
            self._profile_pending = None
            self._update_controls()
            self._message('Camera profile cancelled: camera closed / 配置操作取消：相机已关闭')
            return
        if self._pending_tunes:
            self._flush_tuning()
            return
        command, value = self._profile_pending
        self._profile_pending = None
        self._submit(command, value)

    def _profile_finished(self, result):
        if self._closing:
            return
        self._refresh_profiles(result['name'])
        self.profile_info.setText(f'{result["name"]}\n{result["path"]}')
        self.profile_info.setToolTip(result['path'])

    @Slot(object)
    def _set_parameters(self, values):
        self._capabilities = values
        self.resolution_combo.clear()
        for preset in values.get("resolutions", []):
            self.resolution_combo.addItem(preset["label"], preset["id"])
        self.resolution_combo.setCurrentIndex(self.resolution_combo.findData(values.get("resolution_id")))
        self.resolution_combo.setEnabled(bool(values.get("resolutions")))
        self.manual_exposure_button.setEnabled(values.get("auto_exposure") is not None and values.get("exposure") is not None)
        for key in self.float_controls:
            node = values.get(key)
            control = self.parameter_sliders[key]
            control.setEnabled(node is not None)
            tooltip = 'Unsupported by this camera / 此相机不支持' if node is None else 'Drag to tune live; type an exact value / 拖动实时调节，或输入精确值'
            if node is not None and self.camera_name == 'MindVision' and key in ('gamma', 'contrast'):
                tooltip += '; native SDK scale, 100 neutral / SDK原生刻度，100为中性'
            control.setToolTip(tooltip)
            if node is not None:
                control.set_range(node["min"], node["max"])
                control.set_value(node["value"])
        for key, check in (("auto_exposure", self.auto_exposure),
                           ("reverse_x", self.reverse_x), ("reverse_y", self.reverse_y)):
            check.setEnabled(values.get(key) is not None)
            check.blockSignals(True)
            check.setChecked(bool(values.get(key)))
            check.blockSignals(False)
        self._update_exposure_enabled()

    def _choose_manual_exposure(self):
        self.auto_exposure.setChecked(False)
        self._pending_tunes['auto_exposure'] = False
        self._tune_timer.start()
        self.float_controls["exposure"].setFocus()
        self.float_controls["exposure"].selectAll()
        self.status.setText("Manual exposure selected. Drag Exposure or type a value in µs. / 已选择手动曝光，拖动或输入微秒值。")

    def _update_exposure_enabled(self):
        self.parameter_sliders["exposure"].setEnabled(
            self._capabilities.get("exposure") is not None and not self.auto_exposure.isChecked()
        )

    def _auto_exposure_changed(self, checked):
        self._update_exposure_enabled()
        if self._is_open and not self._closing and self._capabilities.get('auto_exposure') is not None:
            self._pending_tunes['auto_exposure'] = checked
            if checked:
                self._pending_tunes.pop('exposure', None)
            self._tune_timer.start()

    def _queue_tuning(self, key, value):
        if self._closing or not self._is_open or self._capabilities.get(key) is None:
            return
        if key == 'exposure':
            if self.auto_exposure.isChecked():
                return
            self._pending_tunes['auto_exposure'] = False
        self._pending_tunes[key] = value
        # Coalesce fast mouse events without postponing every write until the
        # drag ends. Only one write/readback can be in flight at a time.
        if not self._tune_timer.isActive():
            self._tune_timer.start()

    def _flush_tuning(self):
        if self._closing or not self._is_open:
            self._pending_tunes.clear()
            return
        if self._busy or self._tuning or not self._pending_tunes:
            return
        values, self._pending_tunes = self._pending_tunes, {}
        self._tuning = True
        self.worker.submit('tune', values)

    def _tuning_done(self, result):
        self._tuning = False
        if self._closing:
            return
        parameters = result['parameters']
        if parameters is not None:
            # Never let an older SDK readback jump the handle back underneath
            # an ongoing drag or overwrite the next coalesced user edit.
            for key in result['requested']:
                if key not in self.parameter_sliders or key in self._pending_tunes or self._busy:
                    continue
                control = self.parameter_sliders[key]
                node = parameters.get(key)
                if node is not None and not control.slider.isSliderDown() and not control.spin.hasFocus():
                    control.set_value(node['value'])
        if result['error']:
            self._pending_tunes.clear()
            if self._profile_pending is not None:
                self._profile_pending = None
                self._update_controls()
                self._message('Profile operation cancelled: camera tuning failed / 配置操作取消：相机参数写入失败')
        elif self._profile_pending is not None:
            self._advance_profile_operation()
        elif self._pending_tunes:
            self._tune_timer.start()

    def _apply_parameters(self):
        values = {key: spin.value() for key, spin in self.float_controls.items()
                  if self._capabilities.get(key) is not None}
        for key, check in (("auto_exposure", self.auto_exposure),
                           ("reverse_x", self.reverse_x), ("reverse_y", self.reverse_y)):
            if self._capabilities.get(key) is not None:
                values[key] = check.isChecked()
        selected = self.resolution_combo.currentData()
        if selected is not None and selected != self._capabilities.get("resolution_id"):
            values["resolution_id"] = selected
        self._submit("apply", values)

    @Slot(object)
    def _show_preview(self, frame):
        if self._closing:
            return
        if frame.ndim == 2:
            frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        self._preview_image = QImage(rgb.data, rgb.shape[1], rgb.shape[0], rgb.strides[0], QImage.Format_RGB888).copy()
        self._paint_preview()

    def _paint_preview(self):
        if not self._preview_image.isNull():
            pixmap = QPixmap.fromImage(self._preview_image).scaled(
                self.preview.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
            if self.crosshair.isChecked():
                # Paint on the display pixmap, never on image/snapshot pixels.
                painter = QPainter(pixmap)
                x, y = pixmap.width() // 2, pixmap.height() // 2
                for color, width in ((QColor("black"), 3), (QColor("#00ff70"), 1)):
                    painter.setPen(QPen(color, width))
                    painter.drawLine(x - 35, y, x + 35, y)
                    painter.drawLine(x, y - 35, x, y + 35)
                    painter.drawEllipse(x - 10, y - 10, 20, 20)
                painter.end()
            self.preview.setPixmap(pixmap)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        if hasattr(self, "preview"):
            self._paint_preview()

    @Slot(object)
    def _deliver_snapshot(self, frame):
        if not self._closing:
            self.snapshot_ready.emit(frame.copy())

    @Slot()
    def _finish_close(self):
        self.close()

    def closeEvent(self, event):
        self._tune_timer.stop()
        self._pending_tunes.clear()
        self._profile_pending = None
        if self.intrinsic_window is not None:
            self.intrinsic_window.close()
            if self.intrinsic_window is not None:
                # Detection/fitting must finish before its QThread/UI is freed.
                from PySide6.QtCore import QTimer
                QTimer.singleShot(100, self.close)
                event.ignore()
                return
        if self.worker.isRunning():
            self._closing = True
            self.status.setText("Closing camera…")
            self._update_controls()
            self.worker.shutdown()
            event.ignore()
            return
        self.closed.emit()
        event.accept()
