"""
Pick & Place Vision Guide — PySide6 UI with industrial camera + chessboard calibration + rectification.

Modes:
  - Live View: continuous camera capture, adjustable settings
  - Single Frame: software trigger for single frame grab
"""

import sys
import os

import yaml
import cv2
import numpy as np

from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QLabel, QComboBox,
    QPushButton, QToolBar, QStatusBar, QSlider, QSpinBox,
    QCheckBox, QFormLayout, QHBoxLayout, QVBoxLayout, QFileDialog,
    QMessageBox, QListWidget, QListWidgetItem, QGroupBox, QMenuBar, QMenu,
)
from PySide6.QtCore import Qt, Signal, Slot, QSize
from PySide6.QtGui import QImage, QPixmap, QIcon

from camera import MindVisionCamera, CameraSettings, CameraSettingRanges
from calibration import (
    detect_chessboard,
    calibrate_camera,
    estimate_pose,
    save_calibration,
    load_calibration,
    Rectifier,
)


def _app_dir() -> str:
    """Resolve application directory for config/calib files."""
    if getattr(sys, 'frozen', False):
        return os.path.dirname(sys.executable)
    return os.path.dirname(os.path.abspath(__file__))


# ════════════════════════════════════════════════════════════════════════
#  Camera Settings Window
# ════════════════════════════════════════════════════════════════════════

class CameraSettingsWindow(QWidget):
    """Floating window for camera parameter adjustments."""

    settings_changed = Signal(CameraSettings)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Camera Settings")
        self.setWindowFlags(Qt.Tool | Qt.WindowStaysOnTopHint)
        self.setMinimumWidth(320)
        self._block_signals = False
        self._build_ui()

    def _build_ui(self):
        layout = QFormLayout(self)

        self._ae_check = QCheckBox("Auto Exposure")
        self._ae_check.stateChanged.connect(self._on_setting_changed)
        layout.addRow(self._ae_check)

        self._exposure_slider = QSlider(Qt.Horizontal)
        self._exposure_spin = QSpinBox()
        self._exposure_spin.setSuffix(" us")
        self._exposure_spin.setMinimum(100)
        self._exposure_spin.setMaximum(1000000)
        self._exposure_spin.setSingleStep(100)
        self._exposure_slider.valueChanged.connect(self._exposure_spin.setValue)
        self._exposure_spin.valueChanged.connect(self._exposure_slider.setValue)
        self._exposure_spin.valueChanged.connect(self._on_setting_changed)
        exp_row = QHBoxLayout()
        exp_row.addWidget(self._exposure_slider, 1)
        exp_row.addWidget(self._exposure_spin)
        layout.addRow("Exposure:", exp_row)

        self._gamma_slider = QSlider(Qt.Horizontal)
        self._gamma_spin = QSpinBox()
        self._gamma_slider.valueChanged.connect(self._gamma_spin.setValue)
        self._gamma_spin.valueChanged.connect(self._gamma_slider.setValue)
        self._gamma_spin.valueChanged.connect(self._on_setting_changed)
        gamma_row = QHBoxLayout()
        gamma_row.addWidget(self._gamma_slider, 1)
        gamma_row.addWidget(self._gamma_spin)
        layout.addRow("Gamma:", gamma_row)

        self._contrast_slider = QSlider(Qt.Horizontal)
        self._contrast_spin = QSpinBox()
        self._contrast_slider.valueChanged.connect(self._contrast_spin.setValue)
        self._contrast_spin.valueChanged.connect(self._contrast_slider.setValue)
        self._contrast_spin.valueChanged.connect(self._on_setting_changed)
        contrast_row = QHBoxLayout()
        contrast_row.addWidget(self._contrast_slider, 1)
        contrast_row.addWidget(self._contrast_spin)
        layout.addRow("Contrast:", contrast_row)

        self._gain_slider = QSlider(Qt.Horizontal)
        self._gain_spin = QSpinBox()
        self._gain_slider.valueChanged.connect(self._gain_spin.setValue)
        self._gain_spin.valueChanged.connect(self._gain_slider.setValue)
        self._gain_spin.valueChanged.connect(self._on_setting_changed)
        gain_row = QHBoxLayout()
        gain_row.addWidget(self._gain_slider, 1)
        gain_row.addWidget(self._gain_spin)
        layout.addRow("Analog Gain:", gain_row)

        self._reverse_x_check = QCheckBox("Reverse X (Horizontal Mirror)")
        self._reverse_x_check.stateChanged.connect(self._on_setting_changed)
        layout.addRow(self._reverse_x_check)

        self._reverse_y_check = QCheckBox("Reverse Y (Vertical Mirror)")
        self._reverse_y_check.stateChanged.connect(self._on_setting_changed)
        layout.addRow(self._reverse_y_check)

    def set_ranges(self, ranges: CameraSettingRanges):
        self._block_signals = True
        self._exposure_slider.setRange(ranges.exposure_min_us, ranges.exposure_max_us)
        self._exposure_slider.setSingleStep(ranges.exposure_step_us)
        self._exposure_spin.setRange(ranges.exposure_min_us, ranges.exposure_max_us)
        self._exposure_spin.setSingleStep(ranges.exposure_step_us)
        self._gamma_slider.setRange(ranges.gamma_min, ranges.gamma_max)
        self._gamma_spin.setRange(ranges.gamma_min, ranges.gamma_max)
        self._contrast_slider.setRange(ranges.contrast_min, ranges.contrast_max)
        self._contrast_spin.setRange(ranges.contrast_min, ranges.contrast_max)
        self._gain_slider.setRange(ranges.analog_gain_min, ranges.analog_gain_max)
        self._gain_spin.setRange(ranges.analog_gain_min, ranges.analog_gain_max)
        self._block_signals = False

    def set_values(self, settings: CameraSettings):
        self._block_signals = True
        self._ae_check.setChecked(settings.ae_enabled)
        self._exposure_slider.setValue(settings.exposure_us)
        self._exposure_spin.setValue(settings.exposure_us)
        self._exposure_slider.setEnabled(not settings.ae_enabled)
        self._exposure_spin.setEnabled(not settings.ae_enabled)
        self._gamma_slider.setValue(settings.gamma)
        self._gamma_spin.setValue(settings.gamma)
        self._contrast_slider.setValue(settings.contrast)
        self._contrast_spin.setValue(settings.contrast)
        self._gain_slider.setValue(settings.analog_gain)
        self._gain_spin.setValue(settings.analog_gain)
        self._reverse_x_check.setChecked(settings.reverse_x)
        self._reverse_y_check.setChecked(settings.reverse_y)
        self._block_signals = False

    def _on_setting_changed(self):
        if self._block_signals:
            return
        settings = CameraSettings(
            exposure_us=self._exposure_spin.value(),
            gamma=self._gamma_spin.value(),
            contrast=self._contrast_spin.value(),
            analog_gain=self._gain_spin.value(),
            ae_enabled=self._ae_check.isChecked(),
            reverse_x=self._reverse_x_check.isChecked(),
            reverse_y=self._reverse_y_check.isChecked(),
        )
        self._exposure_slider.setEnabled(not settings.ae_enabled)
        self._exposure_spin.setEnabled(not settings.ae_enabled)
        self.settings_changed.emit(settings)


# ════════════════════════════════════════════════════════════════════════
#  Calibration Window
# ════════════════════════════════════════════════════════════════════════

class CalibrationWindow(QWidget):
    """Popup window for collecting chessboard samples and running calibration.

    No live preview - grabs frames from MainWindow on demand.
    Shows collected image previews when user clicks thumbnails.
    """

    calibration_done = Signal()

    def __init__(self, camera: MindVisionCamera, config: dict, parent=None):
        super().__init__(parent)
        self._camera = camera
        self._config = config
        self._samples_dir = os.path.join(_app_dir(), "calib_samples")
        os.makedirs(self._samples_dir, exist_ok=True)

        calib_cfg = config.get("calibration", {})
        self._board_size = tuple(calib_cfg.get("board_size", [11, 8]))
        self._square_size = calib_cfg.get("square_size", 5.0)

        self._valid_samples = 0
        self._K = None
        self._D = None

        self.setWindowTitle("Camera Calibration")
        self.setMinimumSize(800, 550)
        self.setWindowFlags(Qt.Window)
        self._build_ui()
        self._connect_signals()
        self._refresh_sample_list()

    def _build_ui(self):
        main_layout = QHBoxLayout(self)

        # Left: Image display (shows collected sample preview)
        self._image_label = QLabel("Select a collected sample to preview")
        self._image_label.setAlignment(Qt.AlignCenter)
        self._image_label.setStyleSheet("background-color: #1a1a1a; color: #666;")
        self._image_label.setMinimumSize(640, 480)
        main_layout.addWidget(self._image_label, 1)

        # Right: Controls
        controls = QVBoxLayout()

        # Chessboard parameters info
        info_group = QGroupBox("Chessboard Parameters")
        info_layout = QFormLayout(info_group)
        info_layout.addRow("Board Size:", QLabel(f"{self._board_size[0]} x {self._board_size[1]} inner corners"))
        info_layout.addRow("Square Size:", QLabel(f"{self._square_size} mm"))
        controls.addWidget(info_group)

        # Collect button - grabs frame from main window
        self._collect_btn = QPushButton("Collect Current Frame")
        self._collect_btn.setToolTip("Grab current frame from main window and save if chessboard detected")
        controls.addWidget(self._collect_btn)

        # Sample list
        self._sample_list = QListWidget()
        self._sample_list.setIconSize(QSize(100, 75))
        self._sample_list.setSelectionMode(QListWidget.SingleSelection)
        self._sample_list.setAlternatingRowColors(True)
        controls.addWidget(QLabel("Collected Samples (click to preview):"))
        controls.addWidget(self._sample_list, 1)

        # Sample status
        self._sample_status = QLabel("0 valid samples (need at least 8)")
        controls.addWidget(self._sample_status)

        # Delete button
        self._delete_btn = QPushButton("Delete Selected")
        self._delete_btn.setEnabled(False)
        controls.addWidget(self._delete_btn)

        # Calib button
        self._calib_btn = QPushButton("Calibrate Camera")
        self._calib_btn.setEnabled(False)
        self._calib_btn.setToolTip("Run intrinsic calibration using collected samples")
        controls.addWidget(self._calib_btn)

        # Calib status
        self._calib_status = QLabel("")
        controls.addWidget(self._calib_status)

        # Rectify button
        self._rectify_btn = QPushButton("Rectify (Estimate Pose)")
        self._rectify_btn.setEnabled(False)
        self._rectify_btn.setToolTip("Estimate camera pose from selected sample for rectification")
        controls.addWidget(self._rectify_btn)

        # Rectify status
        self._rectify_status = QLabel("")
        controls.addWidget(self._rectify_status)

        # Close button
        self._close_btn = QPushButton("Close")
        controls.addWidget(self._close_btn)

        controls.addStretch()
        main_layout.addLayout(controls, 0)

    def _connect_signals(self):
        self._collect_btn.clicked.connect(self._on_collect)
        self._sample_list.itemSelectionChanged.connect(self._on_sample_selection_changed)
        self._delete_btn.clicked.connect(self._on_delete)
        self._calib_btn.clicked.connect(self._on_calib)
        self._rectify_btn.clicked.connect(self._on_rectify)
        self._close_btn.clicked.connect(self.close)

    def _get_current_frame_from_main(self) -> np.ndarray | None:
        """Get the current frame from the main window."""
        parent = self.parent()
        if parent and hasattr(parent, '_current_frame'):
            return parent._current_frame
        return None

    def _on_collect(self):
        """Grab current frame from main window and save if chessboard detected."""
        frame = self._get_current_frame_from_main()
        if frame is None:
            QMessageBox.warning(
                self, "No Frame",
                "No frame available from camera.\n"
                "Make sure the main window is showing live view."
            )
            return

        # Ensure BGR format
        if frame.ndim == 2:
            frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        elif frame.shape[2] == 1:
            frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)

        found, corners = detect_chessboard(frame, self._board_size)
        if not found:
            QMessageBox.warning(
                self, "Chessboard Not Detected",
                "The chessboard was not found in the current frame.\n"
                "Please adjust the camera angle or lighting and try again."
            )
            return

        # Save the frame
        idx = len([f for f in os.listdir(self._samples_dir)
                   if f.startswith("chessboard_") and f.endswith(".png")])
        filename = f"chessboard_{idx:03d}.png"
        filepath = os.path.join(self._samples_dir, filename)
        cv2.imwrite(filepath, frame)

        self._refresh_sample_list()

        # Select the newly added item
        for i in range(self._sample_list.count()):
            item = self._sample_list.item(i)
            if item.text() == filename:
                self._sample_list.setCurrentItem(item)
                break

    def _refresh_sample_list(self):
        """Reload sample list and count valid ones."""
        self._sample_list.clear()
        files = sorted([
            f for f in os.listdir(self._samples_dir)
            if f.startswith("chessboard_") and f.endswith(".png")
        ])
        self._valid_samples = 0

        for fname in files:
            filepath = os.path.join(self._samples_dir, fname)
            img = cv2.imread(filepath)
            if img is None:
                continue

            found, corners = detect_chessboard(img, self._board_size)
            if found:
                self._valid_samples += 1

            # Create thumbnail icon
            thumb = cv2.resize(img, (100, 75))
            rgb = cv2.cvtColor(thumb, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb.shape

            # Draw detected corners on thumbnail
            if found and corners is not None:
                sx = 100.0 / img.shape[1]
                sy = 75.0 / img.shape[0]
                for pt in corners:
                    cx = int(pt[0][0] * sx)
                    cy = int(pt[0][1] * sy)
                    cv2.circle(rgb, (cx, cy), 2, (0, 255, 0), -1)

            qimg = QImage(rgb.data, w, h, ch * w, QImage.Format_RGB888)
            pixmap = QPixmap.fromImage(qimg)

            icon = QIcon(pixmap)
            item = QListWidgetItem(icon, fname)
            item.setData(Qt.UserRole, filepath)
            self._sample_list.addItem(item)

        self._sample_status.setText(
            f"{self._valid_samples} valid samples (need at least 8)"
        )
        self._calib_btn.setEnabled(self._valid_samples >= 8)

    def _on_sample_selection_changed(self):
        """Preview selected sample and enable delete button."""
        selected = self._sample_list.selectedItems()
        self._delete_btn.setEnabled(len(selected) > 0)

        if selected:
            filepath = selected[0].data(Qt.UserRole)
            img = cv2.imread(filepath)
            if img is not None:
                display = img.copy()
                found, corners = detect_chessboard(img, self._board_size)
                if found:
                    cv2.drawChessboardCorners(display, self._board_size, corners, found)
                self._display_image(display)

    def _on_delete(self):
        """Delete selected sample file and refresh list."""
        selected = self._sample_list.selectedItems()
        if not selected:
            return

        for item in selected:
            filepath = item.data(Qt.UserRole)
            if os.path.exists(filepath):
                os.remove(filepath)

        self._image_label.setText("Select a collected sample to preview")
        self._image_label.setPixmap(QPixmap())
        self._refresh_sample_list()

    def _on_calib(self):
        """Run intrinsic calibration on collected samples."""
        files = sorted([
            os.path.join(self._samples_dir, f)
            for f in os.listdir(self._samples_dir)
            if f.startswith("chessboard_") and f.endswith(".png")
        ])

        if not files:
            self._calib_status.setText("No samples available")
            return

        self._calib_status.setText("Calibrating...")
        self._calib_btn.setEnabled(False)
        QApplication.processEvents()

        result = calibrate_camera(files, self._board_size, self._square_size)
        if result is None:
            self._calib_status.setText("Calibration failed: not enough valid images")
            self._calib_btn.setEnabled(self._valid_samples >= 8)
            return

        rms, K, D = result
        self._K = K
        self._D = D

        # Save calibration to XML
        calib_path = os.path.join(_app_dir(), "camera_calib.xml")
        save_calibration(calib_path, K, D)

        self._calib_status.setText(f"Done! RMS error: {rms:.4f}")
        self._rectify_btn.setEnabled(True)
        self.calibration_done.emit()

    def _on_rectify(self):
        """Estimate camera pose from selected sample for rectification."""
        # Ensure intrinsics are available
        if self._K is None or self._D is None:
            calib_path = os.path.join(_app_dir(), "camera_calib.xml")
            if os.path.exists(calib_path):
                K, D, _, _ = load_calibration(calib_path)
                if K is not None and D is not None:
                    self._K = K
                    self._D = D

        if self._K is None or self._D is None:
            self._rectify_status.setText("Run calibration first!")
            return

        # Get selected sample or current frame from main window
        selected = self._sample_list.selectedItems()
        if selected:
            filepath = selected[0].data(Qt.UserRole)
            frame = cv2.imread(filepath)
        else:
            frame = self._get_current_frame_from_main()

        if frame is None:
            self._rectify_status.setText("Select a sample or capture a frame first")
            return

        self._rectify_status.setText("Estimating pose...")
        QApplication.processEvents()

        result = estimate_pose(
            frame, self._K, self._D,
            self._board_size, self._square_size,
        )
        if result is None:
            self._rectify_status.setText("Chessboard not found!")
            return

        R, tvec = result

        # Save pose to XML
        calib_path = os.path.join(_app_dir(), "camera_calib.xml")
        save_calibration(calib_path, self._K, self._D, R, tvec)

        self._rectify_status.setText(
            "Pose estimated! Rectification ready.\n"
            "Enable 'Rectify' checkbox in main window."
        )
        self.calibration_done.emit()

    def _display_image(self, image_bgr: np.ndarray):
        """Display BGR numpy array in the image label."""
        if image_bgr.ndim == 2:
            image_bgr = cv2.cvtColor(image_bgr, cv2.COLOR_GRAY2BGR)
        elif image_bgr.shape[2] == 1:
            image_bgr = cv2.cvtColor(image_bgr, cv2.COLOR_GRAY2BGR)

        rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb.shape
        bytes_per_line = ch * w
        qimg = QImage(rgb.data, w, h, bytes_per_line, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(qimg)
        scaled = pixmap.scaled(
            self._image_label.size(),
            Qt.KeepAspectRatio, Qt.SmoothTransformation,
        )
        self._image_label.setPixmap(scaled)

    def closeEvent(self, event):
        """Notify main window when calibration window closes."""
        super().closeEvent(event)


# ════════════════════════════════════════════════════════════════════════
#  Main Window
# ════════════════════════════════════════════════════════════════════════

class MainWindow(QMainWindow):
    def __init__(self, camera: MindVisionCamera, config: dict):
        super().__init__()
        self._camera = camera
        self._config = config

        self._current_mode: str = "live"
        self._current_frame: np.ndarray | None = None
        self._display_pixmap: QPixmap | None = None

        self._rectifier = Rectifier()
        self._rectify_enabled: bool = config.get("rectify_enabled", False)
        self._rectify_check_block = False  # Prevent signal loops during init

        self._calib_window: CalibrationWindow | None = None

        self._build_ui()
        self._connect_signals()
        self._load_rectification()

    # ── UI construction ─────────────────────────────────────────────

    def _build_ui(self):
        self.setWindowTitle("Pick & Place Vision Guide")
        self.setMinimumSize(800, 600)

        # Central image display
        self._image_label = QLabel()
        self._image_label.setAlignment(Qt.AlignCenter)
        self._image_label.setStyleSheet("background-color: #1a1a1a;")
        self._image_label.setCursor(Qt.CrossCursor)
        self.setCentralWidget(self._image_label)

        # Menu bar: Calibration
        menubar = self.menuBar()
        calib_menu = menubar.addMenu("Calibration")
        self._calib_action = calib_menu.addAction("Open Calibration Window")
        self._calib_action.setToolTip(
            "Open window for collecting chessboard samples and running camera calibration"
        )

        # Toolbar
        toolbar = QToolBar("Main")
        toolbar.setMovable(False)
        self.addToolBar(toolbar)

        self._mode_combo = QComboBox()
        self._mode_combo.addItems(["Live View", "Single Frame"])
        self._mode_combo.setMinimumWidth(140)
        toolbar.addWidget(self._mode_combo)

        toolbar.addSeparator()

        self._grab_btn = QPushButton("Grab")
        self._grab_btn.setEnabled(False)
        self._grab_btn.setToolTip("Grab a single frame (software trigger)")
        toolbar.addWidget(self._grab_btn)

        self._save_btn = QPushButton("Save")
        self._save_btn.setEnabled(False)
        self._save_btn.setToolTip("Save current frame to file")
        toolbar.addWidget(self._save_btn)

        toolbar.addSeparator()

        self._rectify_check = QCheckBox("Rectify")
        self._rectify_check.setChecked(self._rectify_enabled)
        self._rectify_check.setToolTip(
            "Enable/disable image rectification (requires calibration)"
        )
        self._rectify_check.setEnabled(False)
        toolbar.addWidget(self._rectify_check)

        toolbar.addSeparator()

        self._settings_btn = QPushButton("Settings")
        self._settings_btn.setToolTip("Open camera settings window")
        toolbar.addWidget(self._settings_btn)

        # Status bar
        self._status_label = QLabel("No camera connected")
        self.statusBar().addWidget(self._status_label, 1)

        # Floating windows
        self._settings_window = CameraSettingsWindow(self)

    # ── Signal wiring ───────────────────────────────────────────────

    def _connect_signals(self):
        # Camera -> UI
        self._camera.signals.frame_ready.connect(self._on_live_frame)
        self._camera.signals.grab_done.connect(self._on_grab_frame)
        self._camera.signals.error.connect(self._on_camera_error)

        # Mode & buttons
        self._mode_combo.currentIndexChanged.connect(self._on_mode_changed)
        self._grab_btn.clicked.connect(self._on_grab_clicked)
        self._save_btn.clicked.connect(self._on_save)
        self._rectify_check.stateChanged.connect(self._on_rectify_toggled)
        self._settings_btn.clicked.connect(self._settings_window.show)
        self._calib_action.triggered.connect(self._on_open_calibration)

        # Settings -> camera + config save
        self._settings_window.settings_changed.connect(self._on_camera_settings_changed)

    # ── Rectification loading ───────────────────────────────────────

    def _load_rectification(self):
        """Try to load existing calibration and set up rectifier."""
        calib_path = os.path.join(_app_dir(), "camera_calib.xml")
        if not os.path.exists(calib_path):
            return

        K, D, R, tvec = load_calibration(calib_path)
        if K is None or D is None:
            return

        if R is not None and tvec is not None:
            if self._camera.is_open:
                w, h = self._camera.resolution
                self._rectifier.setup(K, D, R, tvec, (w, h))
                self._rectify_check.setEnabled(True)
                self._status_label.setText("Rectification ready")
                if self._rectify_enabled:
                    self._rectify_check_block = True
                    self._rectify_check.setChecked(True)
                    self._rectify_check_block = False

    def _setup_rectifier_from_calibration(self):
        """Reload rectification parameters (called after calibration window closes)."""
        calib_path = os.path.join(_app_dir(), "camera_calib.xml")
        K, D, R, tvec = load_calibration(calib_path)
        if K is not None and D is not None and R is not None and tvec is not None:
            if self._camera.is_open:
                w, h = self._camera.resolution
                self._rectifier.setup(K, D, R, tvec, (w, h))
                self._rectify_check.setEnabled(True)
                self._status_label.setText("Rectification ready")

    # ── Slots: camera frames ────────────────────────────────────────

    @Slot(np.ndarray)
    def _on_live_frame(self, frame: np.ndarray):
        frame = self._ensure_bgr(frame)
        self._current_frame = frame
        display = frame.copy()
        if self._rectify_enabled and self._rectifier.ready:
            display = self._rectifier.rectify(frame)
        self._display_image(display)

    @Slot(np.ndarray)
    def _on_grab_frame(self, frame: np.ndarray):
        self._current_frame = self._ensure_bgr(frame)
        display = self._current_frame.copy()
        if self._rectify_enabled and self._rectifier.ready:
            display = self._rectifier.rectify(self._current_frame)
        self._display_image(display)
        self._save_btn.setEnabled(True)
        self._status_label.setText("Frame captured")

    @Slot(str)
    def _on_camera_error(self, msg: str):
        self._status_label.setText(f"Camera error: {msg}")

    @Slot(CameraSettings)
    def _on_camera_settings_changed(self, settings: CameraSettings):
        self._camera.apply_settings(settings)
        cam_cfg = self._config.setdefault("camera", {})
        cam_cfg["exposure_us"] = settings.exposure_us
        cam_cfg["gamma"] = settings.gamma
        cam_cfg["contrast"] = settings.contrast
        cam_cfg["analog_gain"] = settings.analog_gain
        cam_cfg["ae_enabled"] = settings.ae_enabled
        cam_cfg["reverse_x"] = settings.reverse_x
        cam_cfg["reverse_y"] = settings.reverse_y
        self._save_config()

    # ── Slots: mode and buttons ─────────────────────────────────────

    @Slot(int)
    def _on_mode_changed(self, index: int):
        if index == 0:
            self._switch_to_live()
        else:
            self._switch_to_single_frame()

    def _switch_to_live(self):
        self._current_mode = "live"
        self._grab_btn.setEnabled(False)
        if self._camera.is_open:
            self._camera.set_live_mode()

    def _switch_to_single_frame(self):
        self._current_mode = "single_frame"
        self._grab_btn.setEnabled(True)
        if self._camera.is_open:
            self._camera.set_trigger_mode()

    @Slot()
    def _on_grab_clicked(self):
        self._status_label.setText("Grabbing frame...")
        self._camera.software_trigger()

    @Slot()
    def _on_save(self):
        if self._current_frame is None:
            return

        # Determine what to save: rectified or original
        if self._rectify_enabled and self._rectifier.ready:
            save_frame = self._rectifier.rectify(self._current_frame)
        else:
            save_frame = self._current_frame

        path, _ = QFileDialog.getSaveFileName(
            self, "Save Image", "frame.png", "Images (*.png *.jpg *.bmp)"
        )
        if not path:
            return

        cv2.imwrite(path, save_frame)
        self._status_label.setText(f"Saved to {path}")

    @Slot(int)
    def _on_rectify_toggled(self, state: int):
        if self._rectify_check_block:
            return
        self._rectify_enabled = bool(state)
        self._config["rectify_enabled"] = self._rectify_enabled
        self._save_config()

    @Slot()
    def _on_open_calibration(self):
        """Open the calibration window."""
        if self._calib_window is None:
            self._calib_window = CalibrationWindow(
                self._camera, self._config, self
            )
            self._calib_window.calibration_done.connect(
                self._setup_rectifier_from_calibration
            )

        self._calib_window.show()
        self._calib_window.raise_()
        self._calib_window.activateWindow()

    # ── Image display ───────────────────────────────────────────────

    @staticmethod
    def _ensure_bgr(frame: np.ndarray) -> np.ndarray:
        """Convert grayscale/mono frame to BGR if needed."""
        if frame.ndim == 2:
            return cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        if frame.shape[2] == 1:
            return cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        return frame.copy()

    def _display_image(self, image_bgr: np.ndarray):
        """Display a BGR numpy array in the central QLabel."""
        if image_bgr.ndim == 2:
            image_bgr = cv2.cvtColor(image_bgr, cv2.COLOR_GRAY2BGR)
        elif image_bgr.shape[2] == 1:
            image_bgr = cv2.cvtColor(image_bgr, cv2.COLOR_GRAY2BGR)

        rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb.shape
        bytes_per_line = ch * w
        qimg = QImage(rgb.data, w, h, bytes_per_line, QImage.Format_RGB888)
        self._display_pixmap = QPixmap.fromImage(qimg)

        scaled = self._display_pixmap.scaled(
            self._image_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation
        )
        self._image_label.setPixmap(scaled)

    # ── Config persistence ──────────────────────────────────────────

    def _save_config(self):
        config_path = os.path.join(_app_dir(), "config.yaml")
        calib_cfg = self._config.setdefault("calibration", {})
        calib_cfg["board_size"] = list(self._config.get("calibration", {}).get("board_size", [11, 8]))
        calib_cfg["square_size"] = self._config.get("calibration", {}).get("square_size", 5.0)
        try:
            with open(config_path, "w") as f:
                yaml.dump(self._config, f, default_flow_style=False)
        except Exception:
            pass

    def closeEvent(self, event):
        self._save_config()
        if self._calib_window is not None:
            self._calib_window.close()
        super().closeEvent(event)


# ════════════════════════════════════════════════════════════════════════
#  Entry point
# ════════════════════════════════════════════════════════════════════════

def main():
    app = QApplication(sys.argv)

    # Load config
    config_path = os.path.join(_app_dir(), "config.yaml")
    if os.path.exists(config_path):
        with open(config_path) as f:
            config = yaml.safe_load(f) or {}
    else:
        config = {}

    camera = MindVisionCamera()
    window = MainWindow(camera, config)

    # Connect camera
    devices = camera.enumerate_devices()
    if devices:
        try:
            camera.open(devices[0]["dev_info"])
            cam_cfg = config.get("camera", {})
            default_settings = CameraSettings(
                exposure_us=cam_cfg.get("exposure_us", 30000),
                gamma=cam_cfg.get("gamma", 100),
                contrast=cam_cfg.get("contrast", 100),
                analog_gain=cam_cfg.get("analog_gain", 16),
                ae_enabled=cam_cfg.get("ae_enabled", False),
                reverse_x=cam_cfg.get("reverse_x", False),
                reverse_y=cam_cfg.get("reverse_y", False),
            )
            camera.apply_settings(default_settings)

            ranges = camera.get_setting_ranges()
            window._settings_window.set_ranges(ranges)
            window._settings_window.set_values(camera.get_current_settings())

            camera.set_live_mode()
            window._status_label.setText(
                f"Camera: {devices[0]['name']} ({devices[0]['sn']})"
            )
        except Exception as e:
            window._status_label.setText(f"Camera init failed: {e}")
    else:
        window._status_label.setText("No camera found")

    window.show()
    ret = app.exec()

    camera.close()
    sys.exit(ret)


if __name__ == "__main__":
    main()