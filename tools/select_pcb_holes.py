"""Select fixture holes and matched CAD/image probe-pad anchors."""

from __future__ import annotations

import json
import sys
import time
from datetime import datetime
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np
import cv2
from PySide6.QtCore import QPointF, QRectF, Qt, Signal, QTimer
from PySide6.QtGui import QColor, QImage, QPainter, QPainterPath, QPen
from PySide6.QtWidgets import (
    QApplication,
    QComboBox,
    QFileDialog,
    QHBoxLayout,
    QLabel,
    QListWidget,
    QListWidgetItem,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QSplitter,
    QScrollArea,
    QVBoxLayout,
    QWidget,
)

from pcbvision.cad import BoardModel, load_board
from pcbvision.registration import SIFTReference, pose_matrix
from tools.align_pcb_cad import detect_anchor_centers, fit_similarity, _local_circle_center


DEFAULT_DXF = "/home/hotcat/Downloads/uploadPCB/H98_simple.dxf"
DEFAULT_IMAGE = "/home/hotcat/Downloads/uploadPCB/Image_20261109165458976.bmp"
SLOT_NAMES = ("A1", "A2", "A3", "A4")
SLOT_COLORS = ("#e4572e", "#2e86de", "#2a9d65", "#b04acb")
REGISTRATION_RECIPE = (
    "H98 recipe: estimate pose from fixture circles and the visible CAD contour, "
    "or use a taught SIFT camera reference. "
    "Fit the selected A1–A4 holes with strong detections, plus paired probe pads. "
    "Requires all four CAD slots. Timing excludes image loading and drawing."
)


class CADCanvas(QWidget):
    feature_clicked = Signal(int)
    assignments_changed = Signal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(560, 420)
        self.setMouseTracking(True)
        self.setToolTip("Left-click a hole to assign it to the active anchor slot")
        self.board: BoardModel | None = None
        self.assignments: dict[int, int] = {}
        self.active_slot = 0
        self.scale = 1.0
        self.center = np.zeros(2)
        self._panning = False
        self._last_pos = QPointF()
        self.selecting_feature = False
        self.pending_feature: int | None = None
        self.extra_anchors = []

    def set_board(self, board: BoardModel):
        self.board = board
        self.assignments.clear()
        self.fit_board()
        self.update()

    def fit_board(self):
        if self.board is None:
            return
        w, h = max(self.width(), 1), max(self.height(), 1)
        sx = (w - 70) / max(float(self.board.size[0]), 1e-6)
        sy = (h - 70) / max(float(self.board.size[1]), 1e-6)
        self.scale = min(sx, sy)
        self.center = self.board.size / 2.0
        self.update()

    def world_to_screen(self, point):
        x = (float(point[0]) - self.center[0]) * self.scale + self.width() / 2
        y = -(float(point[1]) - self.center[1]) * self.scale + self.height() / 2
        return QPointF(x, y)

    def screen_to_world(self, point):
        x = (point.x() - self.width() / 2) / self.scale + self.center[0]
        y = -(point.y() - self.height() / 2) / self.scale + self.center[1]
        return np.array([x, y])

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.fillRect(self.rect(), QColor("#171b20"))
        if self.board is None:
            painter.setPen(QColor("#aeb7c2"))
            painter.drawText(self.rect(), Qt.AlignCenter, "Open an H98 DXF to begin")
            return

        outline_pen = QPen(QColor("#d7dce2"), 1.6)
        painter.setPen(outline_pen)
        painter.setBrush(Qt.NoBrush)
        path = QPainterPath()
        first = True
        for segment in self.board.outline:
            for point in segment:
                qpoint = self.world_to_screen(point)
                if first:
                    path.moveTo(qpoint)
                    first = False
                else:
                    path.lineTo(qpoint)
        painter.drawPath(path)

        assigned_holes = {hole_idx: slot for slot, hole_idx in self.assignments.items()}
        for index, (center, radius) in enumerate(zip(self.board.holes, self.board.radii)):
            screen = self.world_to_screen(center)
            radius_px = max(float(radius) * self.scale, 5.0)
            slot = assigned_holes.get(index)
            color = QColor(SLOT_COLORS[slot]) if slot is not None else QColor("#f2c14e")
            painter.setPen(QPen(color, 2.3 if slot is not None else 1.8))
            painter.setBrush(QColor(color.red(), color.green(), color.blue(), 35))
            painter.drawEllipse(screen, radius_px, radius_px)
            painter.setBrush(color)
            painter.drawEllipse(screen, 2.8, 2.8)
            label = SLOT_NAMES[slot] if slot is not None else f"H{index + 1}"
            painter.setPen(QPen(color, 1))
            painter.drawText(screen + QPointF(10, -8), label)

        used_features = {item["feature_index"] for item in self.extra_anchors}
        for index, feature in enumerate(self.board.circle_features):
            screen = self.world_to_screen(feature.center)
            color = QColor("#f06cae") if index in used_features else QColor("#57c7c2")
            if index == self.pending_feature:
                color = QColor("#ffe66d")
            painter.setPen(QPen(color, 1.4))
            painter.setBrush(Qt.NoBrush)
            painter.drawEllipse(screen, max(float(feature.radius) * self.scale, 3),
                               max(float(feature.radius) * self.scale, 3))
            painter.setBrush(color)
            painter.drawEllipse(screen, 2.2, 2.2)
            painter.setPen(QPen(color, 1))
            painter.drawText(screen + QPointF(5, 12), f"C{index + 1}")

        for anchor_index, anchor in enumerate(self.extra_anchors):
            screen = self.world_to_screen(anchor["cad_center"])
            color = QColor(SLOT_COLORS[anchor_index % len(SLOT_COLORS)])
            painter.setPen(QPen(color, 2))
            painter.drawLine(screen + QPointF(-5, 0), screen + QPointF(5, 0))
            painter.drawLine(screen + QPointF(0, -5), screen + QPointF(0, 5))

        painter.setPen(QColor("#aeb7c2"))
        painter.drawText(12, 22, f"{self.board.size[0]:.3f} x {self.board.size[1]:.3f} mm")

    def mousePressEvent(self, event):
        if event.button() in (Qt.MiddleButton, Qt.RightButton):
            self._panning = True
            self._last_pos = event.position()
            return
        if event.button() != Qt.LeftButton or self.board is None:
            return
        point = event.position()
        feature_points = [self.world_to_screen(feature.center)
                          for feature in self.board.circle_features]
        feature_distances = [np.hypot(p.x() - point.x(), p.y() - point.y())
                             for p in feature_points]
        candidates = [self.world_to_screen(center) for center in self.board.holes]
        distances = [np.hypot(p.x() - point.x(), p.y() - point.y()) for p in candidates]
        if feature_distances:
            index = int(np.argmin(feature_distances))
            if feature_distances[index] <= 18 and (
                self.selecting_feature or feature_distances[index] < min(distances)
            ):
                self.feature_clicked.emit(index)
                return
        if self.selecting_feature:
            return
        index = int(np.argmin(distances))
        if distances[index] <= 22:
            for slot, old_index in tuple(self.assignments.items()):
                if old_index == index:
                    del self.assignments[slot]
            self.assignments[self.active_slot] = index
            self.update()
            self.assignments_changed.emit()

    def mouseMoveEvent(self, event):
        if self._panning:
            pos = event.position()
            delta = pos - self._last_pos
            self.center[0] -= delta.x() / self.scale
            self.center[1] += delta.y() / self.scale
            self._last_pos = pos
            self.update()

    def mouseReleaseEvent(self, event):
        if event.button() in (Qt.MiddleButton, Qt.RightButton):
            self._panning = False

    def wheelEvent(self, event):
        if self.board is None:
            return
        before = self.screen_to_world(event.position())
        factor = 1.15 if event.angleDelta().y() > 0 else 1 / 1.15
        self.scale = min(max(self.scale * factor, 1.0), 100000.0)
        after = self.screen_to_world(event.position())
        self.center += before - after
        self.update()

    def resizeEvent(self, event):
        if self.board is not None and self.scale <= 1:
            self.fit_board()
        super().resizeEvent(event)


class CameraImageCanvas(QWidget):
    point_clicked = Signal(float, float)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(420, 360)
        self.setToolTip("Wheel to zoom, right or middle drag to pan, left-click an image point")
        self.image = QImage()
        self.zoom = 1.0
        self.pan = QPointF()
        self._panning = False
        self._last_pos = QPointF()

    def set_bgr_image(self, image):
        rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        h, w = rgb.shape[:2]
        self.image = QImage(rgb.data, w, h, rgb.strides[0], QImage.Format_RGB888).copy()
        self.update()

    def image_rect(self):
        if self.image.isNull():
            return QRectF()
        bounds = self.rect().adjusted(12, 12, -12, -12)
        fit = min(bounds.width() / self.image.width(), bounds.height() / self.image.height())
        width = self.image.width() * fit * self.zoom
        height = self.image.height() * fit * self.zoom
        return QRectF((self.width() - width) / 2 + self.pan.x(),
                      (self.height() - height) / 2 + self.pan.y(), width, height)

    def mousePressEvent(self, event):
        if event.button() in (Qt.MiddleButton, Qt.RightButton):
            self._panning = True
            self._last_pos = event.position()
        elif event.button() == Qt.LeftButton and not self.image.isNull():
            rect = self.image_rect()
            if rect.contains(event.position()):
                x = (event.position().x() - rect.left()) / rect.width() * self.image.width()
                y = (event.position().y() - rect.top()) / rect.height() * self.image.height()
                self.point_clicked.emit(x, y)

    def mouseMoveEvent(self, event):
        if self._panning:
            pos = event.position()
            self.pan += pos - self._last_pos
            self._last_pos = pos
            self.update()

    def mouseReleaseEvent(self, event):
        if event.button() in (Qt.MiddleButton, Qt.RightButton):
            self._panning = False

    def wheelEvent(self, event):
        if self.image.isNull():
            return
        pos = event.position()
        old_rect = self.image_rect()
        image_x = (pos.x() - old_rect.left()) / old_rect.width()
        image_y = (pos.y() - old_rect.top()) / old_rect.height()
        self.zoom = min(max(self.zoom * (1.2 if event.angleDelta().y() > 0 else 1 / 1.2), 1.0), 16.0)
        new_rect = self.image_rect()
        target_left = pos.x() - image_x * new_rect.width()
        target_top = pos.y() - image_y * new_rect.height()
        self.pan += QPointF(target_left - new_rect.left(), target_top - new_rect.top())
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.fillRect(self.rect(), QColor("#171b20"))
        if self.image.isNull():
            painter.setPen(QColor("#aeb7c2"))
            painter.drawText(self.rect(), Qt.AlignCenter, "Open a camera image")
            return
        painter.drawImage(self.image_rect(), self.image)


class HoleSelector(QMainWindow):
    def __init__(self, initial_path: str | None = None, initial_image: str | None = None):
        super().__init__()
        self.setWindowTitle("PCB Fixture Hole Selector")
        self.resize(1500, 780)
        self.board: BoardModel | None = None
        self.cad_path: Path | None = None
        self.image_path: Path | None = None
        self.image_source = ""
        self.gray_image: np.ndarray | None = None
        self.source_bgr_image: np.ndarray | None = None
        self.camera_window = None
        self._closing_after_camera = False
        self.sift_reference = None
        self.last_registration_matrix = None
        self.extra_anchors = []
        self.pending_feature: int | None = None
        self.canvas = CADCanvas(self)
        self.image_canvas = CameraImageCanvas(self)
        self.canvas.feature_clicked.connect(self._on_cad_feature_clicked)
        self.canvas.assignments_changed.connect(self.update_assignments)
        self.image_canvas.point_clicked.connect(self._on_image_point_clicked)
        self.slot_combo = QComboBox()
        self.slot_combo.addItems(SLOT_NAMES)
        self.slot_combo.currentIndexChanged.connect(self._set_active_slot)
        self.assignment_list = QListWidget()
        self.assignment_list.setMinimumWidth(275)
        self.open_button = QPushButton("Open DXF…")
        self.open_button.clicked.connect(self.open_dialog)
        self.fit_button = QPushButton("Fit drawing")
        self.fit_button.clicked.connect(self.canvas.fit_board)
        self.clear_button = QPushButton("Clear active")
        self.clear_button.clicked.connect(self.clear_active)
        self.clear_all_button = QPushButton("Clear all")
        self.clear_all_button.clicked.connect(self.clear_all)
        self.save_button = QPushButton("Save anchor recipe…")
        self.save_button.setEnabled(False)
        self.save_button.clicked.connect(self.save_dialog)
        self.load_recipe_button = QPushButton("Load anchor recipe…")
        self.load_recipe_button.clicked.connect(self.load_recipe_dialog)
        self.open_image_button = QPushButton("Open camera image…")
        self.open_image_button.clicked.connect(self.open_image_dialog)
        self.camera_button = QPushButton("Hikvision camera…")
        self.camera_button.clicked.connect(self.show_camera_window)
        self.register_button = QPushButton("Register CAD to image")
        self.register_button.setEnabled(False)
        self.register_button.clicked.connect(self.run_registration)
        self.runtime_label = QLabel("Vision algorithm: —")
        self.recipe_label = QLabel(REGISTRATION_RECIPE)
        self.recipe_label.setWordWrap(True)
        self.recipe_label.setMaximumWidth(330)
        self.reference_label = QLabel("SIFT reference: none (using holes + contour)")
        self.reference_label.setWordWrap(True)
        self.reference_label.setMaximumWidth(330)
        self.teach_reference_button = QPushButton("Teach SIFT from verified overlay")
        self.teach_reference_button.setEnabled(False)
        self.teach_reference_button.clicked.connect(self.teach_sift_reference)
        self.save_reference_button = QPushButton("Save SIFT reference…")
        self.save_reference_button.setEnabled(False)
        self.save_reference_button.clicked.connect(self.save_sift_dialog)
        self.load_reference_button = QPushButton("Load SIFT reference…")
        self.load_reference_button.clicked.connect(self.load_sift_dialog)
        self.clear_reference_button = QPushButton("Clear SIFT reference")
        self.clear_reference_button.clicked.connect(self.clear_sift_reference)
        self.selection_help = QLabel("Select A1–A4 in the CAD view, then click Register CAD to image.")
        self.selection_help.setWordWrap(True)
        self.selection_help.setMaximumWidth(330)
        self.add_extra_button = QPushButton("Add probe-pad anchor")
        self.add_extra_button.setCheckable(True)
        self.add_extra_button.toggled.connect(self._toggle_extra_selection)
        self.extra_anchor_list = QListWidget()
        self.extra_anchor_list.setMaximumHeight(145)
        self.remove_extra_button = QPushButton("Remove selected pair")
        self.remove_extra_button.clicked.connect(self.remove_extra_anchor)

        controls = QVBoxLayout()
        controls.addWidget(self.open_button)
        controls.addWidget(self.open_image_button)
        controls.addWidget(self.camera_button)
        controls.addWidget(self.register_button)
        controls.addWidget(self.runtime_label)
        controls.addWidget(self.recipe_label)
        controls.addWidget(self.reference_label)
        controls.addWidget(self.teach_reference_button)
        reference_buttons = QHBoxLayout()
        reference_buttons.addWidget(self.save_reference_button)
        reference_buttons.addWidget(self.load_reference_button)
        controls.addLayout(reference_buttons)
        controls.addWidget(self.clear_reference_button)
        controls.addWidget(QLabel("Assign clicked hole to"))
        controls.addWidget(self.slot_combo)
        controls.addWidget(self.assignment_list, 1)
        controls.addWidget(QLabel("Additional matched anchors"))
        controls.addWidget(self.add_extra_button)
        controls.addWidget(self.selection_help)
        controls.addWidget(self.extra_anchor_list)
        controls.addWidget(self.remove_extra_button)
        controls.addWidget(self.fit_button)
        controls.addWidget(self.clear_button)
        controls.addWidget(self.clear_all_button)
        controls.addWidget(self.save_button)
        controls.addWidget(self.load_recipe_button)

        views = QSplitter(Qt.Horizontal)
        views.addWidget(self.canvas)
        views.addWidget(self.image_canvas)
        views.setStretchFactor(0, 1)
        views.setStretchFactor(1, 1)

        root = QHBoxLayout()
        root.addWidget(views, 1)
        controls_widget = QWidget()
        controls_widget.setLayout(controls)
        controls_scroll = QScrollArea()
        controls_scroll.setWidgetResizable(True)
        controls_scroll.setWidget(controls_widget)
        controls_scroll.setMinimumWidth(355)
        controls_scroll.setMaximumWidth(430)
        root.addWidget(controls_scroll)
        container = QWidget()
        container.setLayout(root)
        self.setCentralWidget(container)
        self.statusBar().showMessage("No CAD loaded")
        if initial_path:
            self.load_path(initial_path)
        if initial_image:
            self.load_image(initial_image)

    def _set_active_slot(self, index):
        self.canvas.active_slot = index

    def open_dialog(self):
        path, _ = QFileDialog.getOpenFileName(self, "Open H98 DXF", "", "DXF files (*.dxf);;All files (*)")
        if path:
            self.load_path(path)

    def open_image_dialog(self):
        path, _ = QFileDialog.getOpenFileName(
            self, "Open camera image", "", "Images (*.bmp *.png *.jpg *.jpeg *.tif *.tiff);;All files (*)"
        )
        if path:
            self.load_image(path)

    def load_image(self, path):
        image = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
        if image is None:
            QMessageBox.critical(self, "Could not load image", f"Could not read image: {path}")
            return
        self._set_source_image(image, Path(path).name, Path(path))

    def load_camera_frame(self, frame):
        self._set_source_image(frame, f"Hikvision snapshot {datetime.now():%H:%M:%S.%f}")

    def _set_source_image(self, image, source, path=None):
        if image.dtype != np.uint8 or image.ndim not in (2, 3) or (image.ndim == 3 and image.shape[2] != 3):
            raise ValueError("Expected an 8-bit monochrome or BGR camera image")
        self.source_bgr_image = (cv2.cvtColor(image, cv2.COLOR_GRAY2BGR) if image.ndim == 2 else image.copy())
        self.gray_image = cv2.cvtColor(self.source_bgr_image, cv2.COLOR_BGR2GRAY)
        self.image_path = path
        self.image_source = source
        for anchor in self.extra_anchors:
            anchor["clicked_image_px"] = None
            anchor["image_center_px"] = None
            anchor["refined"] = False
            anchor["status"] = "awaiting detection"
            anchor.pop("residual_px", None)
        self.pending_feature = None
        self.canvas.pending_feature = None
        self.canvas.extra_anchors = self.extra_anchors
        self.add_extra_button.setChecked(False)
        self.update_extra_anchor_list()
        self.image_canvas.zoom = 1.0
        self.image_canvas.pan = QPointF()
        self.image_canvas.set_bgr_image(self.source_bgr_image)
        self.invalidate_registration()
        self.selection_help.setText(
            f"Kept {len(self.extra_anchors)} CAD probe selections. Click Register to locate them in this image."
        )
        self.canvas.update()
        self.statusBar().showMessage(f"Loaded {source} | click Register CAD to image when ready")

    def show_camera_window(self):
        if self.camera_window is None:
            from pcbvision.camera_window import CameraWindow
            self.camera_window = CameraWindow(self)
            self.camera_window.snapshot_ready.connect(self.load_camera_frame)
            self.camera_window.closed.connect(self._camera_closed)
        self.camera_window.show()
        self.camera_window.raise_()
        self.camera_window.activateWindow()

    def _camera_closed(self):
        window = self.camera_window
        self.camera_window = None
        if window is not None:
            window.deleteLater()
        if self._closing_after_camera:
            QTimer.singleShot(0, self.close)

    def closeEvent(self, event):
        if self.camera_window is not None:
            self._closing_after_camera = True
            event.ignore()
            self.camera_window.close()
            return
        event.accept()

    def load_path(self, path):
        try:
            board = load_board(path)
        except Exception as exc:
            QMessageBox.critical(self, "Could not load CAD", str(exc))
            return
        self.board = board
        self.cad_path = Path(path)
        self.clear_sift_reference()
        self.extra_anchors.clear()
        self.pending_feature = None
        self.canvas.pending_feature = None
        self.canvas.extra_anchors = self.extra_anchors
        self.extra_anchor_list.clear()
        self.add_extra_button.setChecked(False)
        self.canvas.set_board(board)
        self.update_assignments()
        self.statusBar().showMessage(f"Loaded {self.cad_path.name} | 0/4 anchors")

    def invalidate_registration(self):
        self.last_registration_matrix = None
        self.teach_reference_button.setEnabled(False)
        self.register_button.setEnabled(
            self.board is not None and self.gray_image is not None
            and len(self.canvas.assignments) == 4
        )
        self.runtime_label.setText("Vision algorithm: — (click Register)")
        self.recipe_label.setText(REGISTRATION_RECIPE)
        if self.gray_image is not None:
            preview = self.source_bgr_image.copy()
            for i, anchor in enumerate(self.extra_anchors):
                if anchor["clicked_image_px"] is None:
                    continue
                center = tuple(np.round(anchor["clicked_image_px"]).astype(int))
                cv2.drawMarker(preview, center, (255, 255, 0), cv2.MARKER_TILTED_CROSS, 18, 2)
                cv2.putText(preview, f"P{i + 1}", (center[0] + 8, center[1] - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, .55, (255, 255, 0), 2, cv2.LINE_AA)
            self.image_canvas.set_bgr_image(preview)

    def run_registration(self):
        if self.board is None or self.gray_image is None:
            return
        if len(self.canvas.assignments) != 4:
            self.selection_help.setText("Assign all four CAD fixture holes to A1–A4 before registering.")
            return
        started = time.perf_counter()
        diagnostics = {}
        try:
            anchor_image, observed, detected, weak, hole_residuals, candidate_count = (
                detect_anchor_centers(self.gray_image, self.board,
                                      reference=self.sift_reference, diagnostics=diagnostics)
            )
            selected = [self.canvas.assignments[slot] for slot in range(4)]
            supported = [i for i in selected if detected[i] and not weak[i] and observed[i] is not None]
            if len(supported) < 3:
                raise ValueError("Fewer than three selected fixture holes have strong image detections")
            cad_points = [self.board.holes[i] for i in supported]
            image_points = [observed[i] for i in supported]
            base_count = len(cad_points)
            base_params, _ = fit_similarity(cad_points, image_points)
            a, b = base_params[0] * np.cos(base_params[1]), base_params[0] * np.sin(base_params[1])
            base_matrix = np.array([[a, b], [b, -a]])
            used_extra = []
            for anchor in self.extra_anchors:
                feature = self.board.circle_features[anchor["feature_index"]]
                clicked = anchor["clicked_image_px"]
                seed = (base_matrix @ feature.center + base_params[2:]
                        if clicked is None else np.asarray(clicked, dtype=np.float64))
                inside = 0 <= seed[0] < self.gray_image.shape[1] and 0 <= seed[1] < self.gray_image.shape[0]
                refined = (_local_circle_center(self.gray_image, seed, feature.radius * base_params[0])
                           if inside else None)
                image_point = clicked if refined is None else refined
                anchor.pop("residual_px", None)
                if image_point is None:
                    anchor["image_center_px"] = None
                    anchor["refined"] = False
                    anchor["status"] = "not found; excluded"
                    continue
                anchor["image_center_px"] = np.asarray(image_point, dtype=np.float64)
                anchor["refined"] = refined is not None
                anchor["status"] = "detected" if refined is not None else "manual click"
                used_extra.append(anchor)
                cad_points.append(anchor["cad_center"])
                image_points.append(image_point)
            cad_points = np.asarray(cad_points, dtype=np.float64)
            image_points = np.asarray(image_points, dtype=np.float64)
            params, fitted = fit_similarity(cad_points, image_points)
            scale, angle = params[:2]
            c, s = np.cos(angle), np.sin(angle)
            matrix = scale * np.array([[c, s], [s, -c]])
            translation = fitted.mean(axis=0) - cad_points.mean(axis=0) @ matrix.T
            projected_holes = self.board.holes @ matrix.T + translation
            projected_extra = cad_points[base_count:] @ matrix.T + translation
            fit_residuals = np.linalg.norm(fitted - image_points, axis=1)
            for anchor, residual in zip(used_extra, fit_residuals[base_count:]):
                anchor["residual_px"] = float(residual)
            elapsed_ms = (time.perf_counter() - started) * 1000.0
        except Exception as exc:
            self.last_registration_matrix = None
            self.teach_reference_button.setEnabled(False)
            elapsed_ms = (time.perf_counter() - started) * 1000.0
            self.image_canvas.set_bgr_image(self.source_bgr_image)
            self.runtime_label.setText(f"Vision algorithm: {elapsed_ms:.1f} ms (failed)")
            self.statusBar().showMessage(f"Registration failed after {elapsed_ms:.1f} ms: {exc}")
            QMessageBox.warning(self, "Registration failed", str(exc))
            return

        overlay = self.source_bgr_image.copy()
        for segment in self.board.outline:
            points = (segment @ matrix.T + translation).round().astype(np.int32)
            cv2.polylines(overlay, [points], False, (30, 230, 55), 2, cv2.LINE_AA)
        for i, (point, radius) in enumerate(zip(projected_holes, self.board.radii)):
            center = tuple(np.round(point).astype(int))
            cv2.circle(overlay, center, max(2, round(radius * scale)), (255, 160, 0), 2, cv2.LINE_AA)
            cv2.drawMarker(overlay, center, (0, 0, 255), cv2.MARKER_CROSS, 14, 2)
            slot = selected.index(i)
            cv2.putText(overlay, SLOT_NAMES[slot], (center[0] + 8, center[1] - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, .55, (0, 0, 255), 2, cv2.LINE_AA)
        for anchor, predicted, observed_point in zip(used_extra, projected_extra, image_points[base_count:]):
            i = next(i for i, item in enumerate(self.extra_anchors) if item is anchor)
            predicted_center = tuple(np.round(predicted).astype(int))
            observed_center = tuple(np.round(observed_point).astype(int))
            cv2.circle(overlay, predicted_center, 7, (0, 255, 255), 1, cv2.LINE_AA)
            cv2.drawMarker(overlay, observed_center, (255, 255, 0), cv2.MARKER_TILTED_CROSS, 18, 2)
            cv2.line(overlay, predicted_center, observed_center, (255, 255, 0), 1, cv2.LINE_AA)
            cv2.putText(overlay, f"P{i + 1}", (observed_center[0] + 8, observed_center[1] - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, .55, (255, 255, 0), 2, cv2.LINE_AA)
        self.image_canvas.set_bgr_image(overlay)
        self.last_registration_matrix = pose_matrix(params)
        self.teach_reference_button.setEnabled(True)
        self.update_extra_anchor_list()
        self.canvas.update()
        matched_count = sum(bool(value) for value in detected)
        weak_count = sum(bool(value) for value in weak)
        self.runtime_label.setText(f"Vision algorithm: {elapsed_ms:.1f} ms")
        self.recipe_label.setText(
            f"Coarse: {diagnostics.get('method', 'fixture circles')}. "
            f"H98 recipe: {base_count}/4 selected fixture-hole detections used "
            f"(weak or missing holes excluded), plus {len(used_extra)}/{len(self.extra_anchors)} probe pairs. "
            f"Scale {scale:.3f} px/mm; angle {np.degrees(angle):.4f}°. "
            "Timing excludes image loading and drawing."
        )
        if "contour_support" in diagnostics:
            self.recipe_label.setText(
                self.recipe_label.text() + f" Visible contour edge support: {diagnostics['contour_support']:.0%}."
            )
        if "sift_rejection" in diagnostics:
            self.recipe_label.setText(self.recipe_label.text() + f" SIFT fallback: {diagnostics['sift_rejection']}.")
        self.statusBar().showMessage(
            f"Registered {self.cad_path.name} to {self.image_source} | "
            f"{candidate_count} image candidates, {matched_count}/4 centers supported "
            f"({weak_count} weak) | "
            f"{len(used_extra)}/{len(self.extra_anchors)} probe anchors used "
            f"({sum(bool(a.get('refined')) for a in self.extra_anchors)} refined) | "
            f"anchor residual {np.sqrt(np.mean(fit_residuals ** 2)):.2f} px"
        )

    def teach_sift_reference(self):
        if self.last_registration_matrix is None:
            return
        try:
            reference = SIFTReference.teach(self.gray_image, self.board, self.last_registration_matrix)
        except (ValueError, cv2.error) as exc:
            QMessageBox.warning(self, "Could not teach SIFT reference", str(exc))
            return
        self.sift_reference = reference
        self.save_reference_button.setEnabled(True)
        self.reference_label.setText(f"SIFT reference taught: {len(reference.points)} PCB features. Retained across images.")
        self.statusBar().showMessage("SIFT reference taught from this overlay. Verify the overlay before teaching.")

    def clear_sift_reference(self):
        self.sift_reference = None
        self.save_reference_button.setEnabled(False)
        self.reference_label.setText("SIFT reference: none (using holes + contour)")

    def save_sift_dialog(self):
        if self.sift_reference is None:
            return
        path, _ = QFileDialog.getSaveFileName(self, "Save SIFT reference", "pcb_sift.npz", "SIFT reference (*.npz)")
        if path:
            try:
                self.sift_reference.save(path)
                self.statusBar().showMessage(f"Saved SIFT reference to {path}")
            except OSError as exc:
                QMessageBox.warning(self, "Could not save SIFT reference", str(exc))

    def load_sift_dialog(self):
        if self.board is None:
            self.selection_help.setText("Open the matching CAD first")
            return
        path, _ = QFileDialog.getOpenFileName(self, "Load SIFT reference", "", "SIFT reference (*.npz)")
        if path:
            try:
                self.sift_reference = SIFTReference.load(path, self.board)
                self.save_reference_button.setEnabled(True)
                self.reference_label.setText(f"SIFT reference loaded: {len(self.sift_reference.points)} PCB features")
            except (OSError, ValueError, KeyError, cv2.error) as exc:
                QMessageBox.warning(self, "Could not load SIFT reference", str(exc))

    def _toggle_extra_selection(self, enabled):
        self.canvas.selecting_feature = enabled
        if enabled:
            self.canvas.setToolTip("Click a CAD circle center to start a matched anchor pair")
            if self.pending_feature is not None:
                self.selection_help.setText(
                    f"C{self.pending_feature + 1} selected. Click its matching center in the camera image."
                )
            else:
                self.selection_help.setText("Click a CAD circle (C1, C2, …), then its matching center in the camera image.")
        else:
            self.canvas.pending_feature = None
            self.pending_feature = None
            self.canvas.setToolTip("Left-click a hole to assign it to the active anchor slot")
            self.selection_help.setText("Click a CAD circle, then Add probe-pad anchor to pair it with the image.")
        self.statusBar().showMessage(self.selection_help.text())
        self.canvas.update()

    def _on_cad_feature_clicked(self, feature_index):
        self.pending_feature = feature_index
        self.canvas.pending_feature = feature_index
        self.canvas.update()
        feature = self.board.circle_features[feature_index]
        next_step = ("Click its matching center in the camera image." if self.add_extra_button.isChecked()
                     else "Click Add probe-pad anchor, then its matching center in the camera image.")
        self.selection_help.setText(f"C{feature_index + 1}: {feature.handle} ({feature.layer}) selected. {next_step}")
        self.statusBar().showMessage(self.selection_help.text())

    def _on_image_point_clicked(self, x, y):
        if not self.add_extra_button.isChecked() or self.pending_feature is None:
            return
        feature = self.board.circle_features[self.pending_feature]
        pair = {
            "feature_index": self.pending_feature,
            "cad_center": feature.center.copy(),
            "clicked_image_px": np.array([x, y], dtype=np.float64),
            "image_center_px": np.array([x, y], dtype=np.float64),
            "refined": False,
            "status": "manual click",
        }
        existing = next((a for a in self.extra_anchors if a["feature_index"] == self.pending_feature), None)
        if existing is None:
            self.extra_anchors.append(pair)
        else:
            existing.clear()
            existing.update(pair)
        self.pending_feature = None
        self.canvas.pending_feature = None
        self.canvas.extra_anchors = self.extra_anchors
        self.canvas.update()
        self.update_extra_anchor_list()
        self.invalidate_registration()
        self.selection_help.setText(
            "Anchor paired. Select another CAD circle, or click Register CAD to image to apply the pairs."
        )
        self.statusBar().showMessage(self.selection_help.text())

    def update_extra_anchor_list(self):
        self.extra_anchor_list.clear()
        if self.board is None:
            return
        for index, anchor in enumerate(self.extra_anchors):
            feature = self.board.circle_features[anchor["feature_index"]]
            point = anchor["image_center_px"]
            position = f"({point[0]:.1f}, {point[1]:.1f}) px" if point is not None else "no image center"
            source = anchor.get("status", "manual click")
            residual = f" residual {anchor['residual_px']:.2f} px" if "residual_px" in anchor else ""
            self.extra_anchor_list.addItem(
                f"P{index + 1} C{anchor['feature_index'] + 1} {feature.handle} {feature.layer} -> "
                f"{position} [{source}]{residual}"
            )

    def remove_extra_anchor(self):
        index = self.extra_anchor_list.currentRow()
        if index < 0 or index >= len(self.extra_anchors):
            return
        self.extra_anchors.pop(index)
        self.canvas.extra_anchors = self.extra_anchors
        self.canvas.update()
        self.update_extra_anchor_list()
        self.invalidate_registration()

    def update_assignments(self):
        self.assignment_list.clear()
        if self.board is None:
            self.save_button.setEnabled(False)
            return
        for slot, name in enumerate(SLOT_NAMES):
            index = self.canvas.assignments.get(slot)
            if index is None:
                text = f"{name}: not selected"
            else:
                xy = self.board.holes[index]
                cad_xy = self.board.origin + xy
                handle = self.board.hole_handles[index]
                text = f"{name}: {handle}  ({cad_xy[0]:.3f}, {cad_xy[1]:.3f}) mm"
            item = QListWidgetItem(text)
            item.setForeground(QColor(SLOT_COLORS[slot]))
            self.assignment_list.addItem(item)
        self.save_button.setEnabled(len(self.canvas.assignments) == 4)
        self.invalidate_registration()
        self.statusBar().showMessage(
            f"Loaded {self.cad_path.name} | {len(self.canvas.assignments)}/4 anchors"
        )

    def clear_active(self):
        self.canvas.assignments.pop(self.slot_combo.currentIndex(), None)
        self.canvas.update()
        self.update_assignments()

    def clear_all(self):
        self.canvas.assignments.clear()
        self.extra_anchors.clear()
        self.pending_feature = None
        self.canvas.pending_feature = None
        self.canvas.extra_anchors = self.extra_anchors
        self.add_extra_button.setChecked(False)
        self.canvas.update()
        self.update_extra_anchor_list()
        self.update_assignments()

    def save_dialog(self):
        if self.board is None or len(self.canvas.assignments) != 4:
            return
        default = self.cad_path.with_name(self.cad_path.stem + "_anchors.json")
        path, _ = QFileDialog.getSaveFileName(self, "Save anchor recipe", str(default), "JSON files (*.json)")
        if path:
            self.save_anchors(path)

    def save_anchors(self, path):
        if self.board is None or len(self.canvas.assignments) != 4:
            return
        anchors = []
        for slot, name in enumerate(SLOT_NAMES):
            index = self.canvas.assignments[slot]
            local = self.board.holes[index]
            cad = self.board.origin + local
            anchors.append({
                "name": name,
                "dxf_handle": self.board.hole_handles[index],
                "center_cad_mm": [float(v) for v in cad],
                "center_board_mm": [float(v) for v in local],
                "radius_mm": float(self.board.radii[index]),
            })
        payload = {
            "schema": "pcbvision.fixture_anchors.v2",
            "cad_file": str(self.cad_path.resolve()),
            "units": self.board.units,
            "board_origin_cad_mm": [float(v) for v in self.board.origin],
            "anchors": anchors,
            "probe_anchors": [
                {
                    "dxf_handle": self.board.circle_features[a["feature_index"]].handle,
                    "layer": self.board.circle_features[a["feature_index"]].layer,
                    "center_board_mm": a["cad_center"].tolist(),
                    "radius_mm": float(self.board.circle_features[a["feature_index"]].radius),
                }
                for a in self.extra_anchors
            ],
        }
        try:
            Path(path).write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
        except OSError as exc:
            QMessageBox.critical(self, "Could not save anchors", str(exc))
            return
        self.statusBar().showMessage(
            f"Saved A1–A4 and {len(self.extra_anchors)} probe selections to {path}"
        )

    def load_recipe_dialog(self):
        if self.board is None:
            self.selection_help.setText("Open the matching CAD drawing before loading an anchor recipe.")
            return
        path, _ = QFileDialog.getOpenFileName(self, "Load anchor recipe", "", "JSON files (*.json)")
        if path:
            try:
                self.load_anchors(path)
            except (OSError, ValueError, KeyError, TypeError, IndexError) as exc:
                QMessageBox.warning(self, "Could not load anchor recipe", str(exc))

    def load_anchors(self, path):
        """Validate recipe geometry against the open CAD before changing selections."""
        if self.board is None:
            raise ValueError("Open the matching CAD drawing first")
        payload = json.loads(Path(path).read_text(encoding="utf-8"))
        if payload.get("schema") not in ("pcbvision.fixture_anchors.v1", "pcbvision.fixture_anchors.v2"):
            raise ValueError("Unsupported anchor recipe format")
        if payload.get("units") != self.board.units:
            raise ValueError("Recipe units do not match the open CAD")
        origin = np.asarray(payload["board_origin_cad_mm"], dtype=np.float64)
        if origin.shape != (2,) or not np.allclose(origin, self.board.origin, atol=1e-4, rtol=0):
            raise ValueError("Recipe origin does not match the open CAD")

        def same_geometry(entry, center, radius):
            point = np.asarray(entry["center_board_mm"], dtype=np.float64)
            return (point.shape == (2,) and np.allclose(point, center, atol=1e-4, rtol=0)
                    and abs(float(entry["radius_mm"]) - radius) < 1e-4)

        assignments = {}
        entries = payload["anchors"]
        if len(entries) != 4:
            raise ValueError("Recipe must contain four fixture assignments")
        for entry in entries:
            slot = SLOT_NAMES.index(entry["name"])
            if slot in assignments:
                raise ValueError("Recipe has duplicate fixture slot names")
            matches = [i for i, handle in enumerate(self.board.hole_handles)
                       if handle == entry["dxf_handle"]
                       and same_geometry(entry, self.board.holes[i], self.board.radii[i])]
            if len(matches) != 1 or matches[0] in assignments.values():
                raise ValueError("Recipe fixture holes do not match the open CAD")
            assignments[slot] = matches[0]
        probes = []
        used = set()
        for entry in payload.get("probe_anchors", []):
            matches = [i for i, feature in enumerate(self.board.circle_features)
                       if feature.handle == entry["dxf_handle"] and feature.layer == entry["layer"]
                       and same_geometry(entry, feature.center, feature.radius)]
            if len(matches) != 1 or matches[0] in used:
                raise ValueError("Recipe probe selections do not match the open CAD")
            index = matches[0]
            used.add(index)
            probes.append({
                "feature_index": index,
                "cad_center": self.board.circle_features[index].center.copy(),
                "clicked_image_px": None,
                "image_center_px": None,
                "refined": False,
                "status": "awaiting detection",
            })
        self.canvas.assignments = assignments
        self.extra_anchors[:] = probes
        self.canvas.extra_anchors = self.extra_anchors
        self.pending_feature = None
        self.canvas.pending_feature = None
        self.add_extra_button.setChecked(False)
        self.update_assignments()
        self.update_extra_anchor_list()
        self.canvas.update()
        self.selection_help.setText(
            f"Loaded A1–A4 and {len(probes)} probe selections. Click Register to detect fresh image centers."
        )
        self.statusBar().showMessage(self.selection_help.text())


def main():
    app = QApplication(sys.argv)
    initial_path = sys.argv[1] if len(sys.argv) > 1 else (DEFAULT_DXF if Path(DEFAULT_DXF).exists() else None)
    initial_image = sys.argv[2] if len(sys.argv) > 2 else (DEFAULT_IMAGE if Path(DEFAULT_IMAGE).exists() else None)
    window = HoleSelector(initial_path, initial_image)
    window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
