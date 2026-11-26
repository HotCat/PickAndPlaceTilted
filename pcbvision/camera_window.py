"""Industrial camera live/capture window; all SDK access stays in one thread."""
from __future__ import annotations

import queue
import threading
import time

import cv2
import numpy as np
from PySide6.QtCore import QThread, Qt, Signal, Slot
from PySide6.QtGui import QColor, QImage, QPainter, QPen, QPixmap
from PySide6.QtWidgets import (
    QCheckBox, QComboBox, QDoubleSpinBox, QFormLayout, QGroupBox,
    QHBoxLayout, QLabel, QPushButton, QSizePolicy, QVBoxLayout, QWidget,
)


def _make_camera():
    from .hikvision import HikvisionCamera
    return HikvisionCamera()


class CameraWorker(QThread):
    devices_found = Signal(object)
    state_changed = Signal(bool, bool)
    parameters_ready = Signal(object)
    preview_ready = Signal(object)
    snapshot_ready = Signal(object)
    calibration_frame_ready = Signal(object)
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
        self.parameters_group = QGroupBox("Camera parameters")
        form = QFormLayout(self.parameters_group)
        self.auto_exposure = QCheckBox("Auto exposure")
        self.auto_exposure.toggled.connect(self._update_exposure_enabled)
        form.addRow(self.auto_exposure)
        self.manual_exposure_button = QPushButton("Manual exposure / 手动曝光")
        self.manual_exposure_button.setToolTip("Turn off auto exposure, then enter Exposure and click Apply parameters")
        self.manual_exposure_button.clicked.connect(self._choose_manual_exposure)
        form.addRow(self.manual_exposure_button)
        self.resolution_combo = QComboBox()
        self.resolution_combo.setToolTip("Camera acquisition preset, not display scaling; click Apply parameters")
        form.addRow(QLabel("Resolution / 分辨率"))
        form.addRow(self.resolution_combo)
        self.float_controls = {}
        for key, label, suffix in (("exposure", "Exposure", " µs"),
                                   ("gain", "Gain", " dB"), ("gamma", "Gamma", ""),
                                   ("contrast", "Contrast / 对比度", "")):
            spin = QDoubleSpinBox()
            spin.setDecimals(3)
            spin.setSuffix(suffix)
            spin.setKeyboardTracking(False)
            if camera_name == "MindVision" and key in ("gain", "gamma", "contrast"):
                spin.setDecimals(0)
                spin.setSuffix(" SDK units" if key == "gain" else "")
            if camera_name == "MindVision" and key in ("gamma", "contrast"):
                spin.setToolTip("Native SDK scale (100 is neutral); Apply selects the gamma/contrast LUT")
            self.float_controls[key] = spin
            form.addRow(label, spin)
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
        row.addLayout(sidebar)
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
        self._is_open, self._is_live = opened, live
        if not opened:
            self._preview_image = QImage()
            self.preview.clear()
            self.preview.setText("Camera closed")
        self._update_controls()

    def _update_controls(self):
        enabled = not self._busy and not self._closing
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

    @Slot(object)
    def _set_parameters(self, values):
        self._capabilities = values
        self.resolution_combo.clear()
        for preset in values.get("resolutions", []):
            self.resolution_combo.addItem(preset["label"], preset["id"])
        self.resolution_combo.setCurrentIndex(self.resolution_combo.findData(values.get("resolution_id")))
        self.resolution_combo.setEnabled(bool(values.get("resolutions")))
        self.manual_exposure_button.setEnabled(values.get("auto_exposure") is not None and values.get("exposure") is not None)
        for key, spin in self.float_controls.items():
            node = values.get(key)
            spin.setEnabled(node is not None)
            spin.setToolTip("Unsupported by this camera" if node is None else "")
            if node is not None:
                spin.setRange(node["min"], node["max"])
                spin.setValue(node["value"])
        for key, check in (("auto_exposure", self.auto_exposure),
                           ("reverse_x", self.reverse_x), ("reverse_y", self.reverse_y)):
            check.setEnabled(values.get(key) is not None)
            check.setChecked(bool(values.get(key)))
        self._update_exposure_enabled()

    def _choose_manual_exposure(self):
        self.auto_exposure.setChecked(False)
        self.float_controls["exposure"].setFocus()
        self.float_controls["exposure"].selectAll()
        self.status.setText("Manual exposure selected. Enter Exposure (µs), then click Apply parameters. / 输入曝光时间后点击应用。")

    def _update_exposure_enabled(self):
        self.float_controls["exposure"].setEnabled(
            self._capabilities.get("exposure") is not None and not self.auto_exposure.isChecked()
        )

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
