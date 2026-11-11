"""Dedicated Hikvision live/capture window; all SDK access stays in one thread."""
from __future__ import annotations

import queue
import threading
import time

import cv2
import numpy as np
from PySide6.QtCore import QThread, Qt, Signal, Slot
from PySide6.QtGui import QImage, QPixmap
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
                            self.message.emit(f"Found {len(devices)} camera(s). Select a device and click Open.")
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

    def __init__(self, parent=None, camera_factory=_make_camera):
        super().__init__(parent, Qt.Window)
        self.setWindowTitle("Hikvision camera — live view and snapshot")
        self.resize(1100, 720)
        self._devices = []
        self._is_open = False
        self._is_live = False
        self._busy = False
        self._closing = False
        self._preview_image = QImage()

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
        self.read_button = QPushButton("Read parameters")
        self.apply_button = QPushButton("Apply parameters")
        self.parameters_group = QGroupBox("Camera parameters")
        form = QFormLayout(self.parameters_group)
        self.auto_exposure = QCheckBox("Auto exposure")
        self.auto_exposure.toggled.connect(self._update_exposure_enabled)
        form.addRow(self.auto_exposure)
        self.float_controls = {}
        for key, label, suffix in (("exposure", "Exposure", " µs"),
                                   ("gain", "Gain", " dB"), ("gamma", "Gamma", "")):
            spin = QDoubleSpinBox()
            spin.setDecimals(3)
            spin.setSuffix(suffix)
            spin.setKeyboardTracking(False)
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
        sidebar.addWidget(self.parameters_group)
        hint = QLabel("Snapshots use a fresh full-resolution frame and go directly to the main image area. Click Register there to apply the CAD overlay.")
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
        self.worker.message.connect(self.status.setText)
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

    @Slot(object)
    def _set_devices(self, devices):
        self._devices = devices
        self.device_combo.clear()
        self.device_combo.addItems([item["name"] for item in devices])

    def _open_camera(self):
        index = self.device_combo.currentIndex()
        if 0 <= index < len(self._devices):
            self._submit("open", self._devices[index]["dev_info"])

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
        self.live_button.setText("Stop live view" if self._is_live else "Start live view")

    @Slot(object)
    def _set_parameters(self, values):
        self._capabilities = values
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
            self.preview.setPixmap(QPixmap.fromImage(self._preview_image).scaled(
                self.preview.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))

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
        if self.worker.isRunning():
            self._closing = True
            self.status.setText("Closing camera…")
            self._update_controls()
            self.worker.shutdown()
            event.ignore()
            return
        self.closed.emit()
        event.accept()
