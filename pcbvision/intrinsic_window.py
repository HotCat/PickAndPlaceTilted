"""Guided intrinsic capture inside the existing camera preview, with CPU jobs off UI.

One SDK owner remains CameraWorker; detection/fitting use a separate short-lived
QThread. No robot API is imported. Closing waits for computation without deleting
a running QThread, and every accepted native image/corner set is saved immediately.
"""
from __future__ import annotations

from datetime import datetime
from pathlib import Path
import uuid

import cv2
import numpy as np
from PySide6.QtCore import QSettings, Qt, QThread, Signal
from PySide6.QtGui import QImage, QPixmap
from PySide6.QtWidgets import (
    QCheckBox, QDialog, QDoubleSpinBox, QFileDialog, QFormLayout, QHBoxLayout,
    QLabel, QLineEdit, QListWidget, QListWidgetItem, QMessageBox, QPushButton,
    QScrollArea, QSpinBox, QSplitter, QVBoxLayout, QWidget,
)

from .intrinsics import IntrinsicSession, MIN_VIEWS, RECOMMENDED_VIEWS, calibration_root, coverage


class CalibrationJob(QThread):
    succeeded = Signal(object)
    failed = Signal(str)
    progress = Signal(str)

    def __init__(self, function, parent):
        super().__init__(parent)
        self.function = function

    def run(self):
        try:
            self.succeeded.emit(self.function())
        except Exception as exc:
            self.failed.emit(str(exc))


class IntrinsicWindow(QDialog):
    closed = Signal()

    def __init__(self, camera_window):
        super().__init__(camera_window, Qt.Window)
        self.camera_window = camera_window
        self.settings = QSettings('PickPlaceVision', 'Intrinsics')
        self.job = None
        self._waiting = False
        self._close_pending = False
        self._capture_available = camera_window._is_open and not camera_window._busy
        self._rendering = False
        self._image = QImage()
        self.session = IntrinsicSession(self.new_directory())
        self.setWindowTitle('Camera intrinsics / 相机内参标定')
        self.resize(1260, 900)
        self.setStyleSheet('''
          QDialog, QWidget { background: #252d38; color: #edf1f7; }
          QLineEdit, QSpinBox, QDoubleSpinBox, QListWidget { background: #151d27; color: #edf1f7;
            border: 1px solid #45586c; padding: 5px; selection-background-color: #285e86; }
          QPushButton { background: #34485b; color: white; padding: 9px; border: 1px solid #526c83; border-radius: 4px; }
          QPushButton:disabled { color: #8492a3; background: #263441; }
          QPushButton#capture { background: #176c55; }
          QLabel#status { color: #85ddc5; }
          QScrollArea { border: none; }
        ''')
        outer = QVBoxLayout(self)
        instructions = QLabel(
            '1  Lock focus/aperture and camera mode.  2  Capture 20–30 DIFFERENT views: center, edges, corners, distances and tilts about BOTH axes. '
            '3  Fit, inspect per-view errors, then export.\n'
            '固定焦距/光圈与采集模式；采集20–30个不同位置、距离及双轴倾角的完整棋盘视图；计算后检查误差并导出。\n'
            'Move the board by hand with the robot stationary if possible. This window never moves the robot. / 可固定机器人，手动移动棋盘。')
        instructions.setWordWrap(True)
        outer.addWidget(instructions)
        splitter = QSplitter()
        outer.addWidget(splitter, 1)
        left = QWidget()
        left_layout = QVBoxLayout(left)
        self.preview = QLabel('Capture a native-resolution view / 采集完整分辨率图像')
        self.preview.setAlignment(Qt.AlignCenter)
        self.preview.setMinimumSize(450, 320)
        self.preview.setStyleSheet('background: #111820; color: #b7c7d8')
        left_layout.addWidget(self.preview, 1)
        self.live_framing = QCheckBox('Live framing / 实时取景（显示缩放，采集仍用原图）')
        self.live_framing.setToolTip('Watch this while arranging the next board view. Corner detection occurs on Capture.')
        self.live_framing.toggled.connect(lambda checked: None if checked else self.show_selected())
        left_layout.addWidget(self.live_framing)
        self.undistort = QCheckBox('Show undistorted selected image / 显示选中图像去畸变结果')
        self.undistort.toggled.connect(self.show_selected)
        left_layout.addWidget(self.undistort)
        self.image_info = QLabel('Overlay uses detected native pixel coordinates / 角点在原始像素坐标中检测')
        self.image_info.setWordWrap(True)
        left_layout.addWidget(self.image_info)
        splitter.addWidget(left)
        panel = QWidget()
        side = QVBoxLayout(panel)
        form = QFormLayout()
        self.cols, self.rows = QSpinBox(), QSpinBox()
        for spin, value in ((self.cols, 11), (self.rows, 8)):
            spin.setRange(3, 40)
            spin.setValue(value)
        self.square = QDoubleSpinBox()
        self.square.setRange(.01, 1000.)
        self.square.setDecimals(3)
        self.square.setSuffix(' mm')
        self.square.setValue(3.)
        form.addRow('Inner columns / 内角点列', self.cols)
        form.addRow('Inner rows / 内角点行', self.rows)
        form.addRow('Square side / 方格边长', self.square)
        side.addLayout(form)
        explain = QLabel('11 × 8 INNER corners = 12 × 9 squares. Your board label: 12*9-3. / 此处填写内角点数。')
        explain.setWordWrap(True)
        side.addWidget(explain)
        self.lens = QLineEdit('Current fixed lens/focus')
        self.lens.setPlaceholderText('Lens ID, focus/aperture marks / 镜头编号与对焦刻度')
        side.addWidget(QLabel('Lens/focus setup / 镜头及对焦设置'))
        side.addWidget(self.lens)
        self.locked = QCheckBox('Focus/aperture fixed; board flat / 镜头已固定，棋盘平整')
        self.locked.toggled.connect(self.update_controls)
        side.addWidget(self.locked)
        self.capture = QPushButton('Capture view / 采集当前视图', objectName='capture')
        self.capture.clicked.connect(self.request_capture)
        side.addWidget(self.capture)
        self.samples = QListWidget()
        self.samples.setMinimumHeight(150)
        self.samples.itemChanged.connect(self.toggle_sample)
        self.samples.currentRowChanged.connect(self.show_selected)
        side.addWidget(QLabel('Views (uncheck to exclude) / 样本（取消勾选可排除）'))
        side.addWidget(self.samples, 1)
        self.summary = QLabel('')
        self.summary.setWordWrap(True)
        side.addWidget(self.summary)
        self.fit = QPushButton('Fit intrinsics / 计算内参')
        self.fit.clicked.connect(self.fit_intrinsics)
        side.addWidget(self.fit)
        self.reprocess_button = QPushButton('Reprocess saved images / 重新检测已保存图像')
        self.reprocess_button.setToolTip('Create a new session with corrected corner detection and refit. Original session/images remain unchanged. / 新建副本重新检测并计算，保留原始采集。')
        self.reprocess_button.clicked.connect(self.reprocess_images)
        side.addWidget(self.reprocess_button)
        self.result = QLabel('No fitted result / 尚未计算')
        self.result.setWordWrap(True)
        self.result.setTextInteractionFlags(Qt.TextSelectableByMouse)
        side.addWidget(self.result)
        self.export_button = QPushButton('Export JSON / XML / 导出')
        self.export_button.clicked.connect(self.export)
        side.addWidget(self.export_button)
        row = QHBoxLayout()
        self.new_button = QPushButton('New session / 新建')
        self.new_button.clicked.connect(self.new_session)
        self.load_button = QPushButton('Resume session / 恢复采集')
        self.load_button.clicked.connect(self.resume)
        row.addWidget(self.new_button)
        row.addWidget(self.load_button)
        side.addLayout(row)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(panel)
        scroll.setMinimumWidth(410)
        splitter.addWidget(scroll)
        splitter.setSizes([780, 440])
        self.path_label = QLabel('')
        self.path_label.setWordWrap(True)
        self.path_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        outer.addWidget(self.path_label)
        self.status = QLabel('Ready. Keep the whole board visible / 就绪，请保持完整棋盘在视野内', objectName='status')
        self.status.setWordWrap(True)
        outer.addWidget(self.status)
        self.camera_window.worker.command_done.connect(self.camera_command_done)
        self.camera_window.worker.preview_ready.connect(self.show_live)
        last = self.settings.value('last_session', '')
        if last and (Path(last)/'session.json').exists():
            try:
                self.session = IntrinsicSession.load(last)
                self.apply_session_settings()
                self.status.setText('Restored last session. Confirm lens/focus unchanged before adding views. / 已恢复采集，请确认镜头状态一致。')
            except Exception as exc:
                self.status.setText(f'Could not resume old session: {exc}. A new session is ready.')
        self.render_samples()

    def new_directory(self):
        return calibration_root()/(datetime.now().strftime('%Y%m%d-%H%M%S')+'-'+uuid.uuid4().hex[:6])

    def update_capture_available(self, available):
        self._capture_available = available
        self.update_controls()

    def update_controls(self, *_):
        busy = self.job is not None or self._waiting
        for widget in (self.new_button, self.load_button, self.samples):
            widget.setEnabled(not busy)
        self.capture.setEnabled(not busy and self._capture_available and self.locked.isChecked() and not self._close_pending)
        self.fit.setEnabled(not busy and len(self.session.active) >= MIN_VIEWS)
        self.reprocess_button.setEnabled(not busy and bool(self.session.samples))
        self.export_button.setEnabled(not busy and 'result' in self.session.data)
        self.undistort.setEnabled(not busy and 'result' in self.session.data)
        fixed = bool(self.session.samples) or busy
        for widget in (self.cols, self.rows, self.square, self.lens):
            widget.setEnabled(not fixed)
        self.locked.setEnabled(not busy)

    def request_capture(self):
        if not self.capture.isEnabled():
            return
        if not self.session.samples:
            self.session = IntrinsicSession(self.session.directory, (self.cols.value(), self.rows.value()), self.square.value())
        self._waiting = True
        self.update_controls()
        self.status.setText('Acquiring a fresh native frame… / 正在采集原始图像…')
        try:
            self.camera_window.capture_intrinsic_frame()
        except Exception as exc:
            self._waiting = False
            self.status.setText(str(exc))
            self.update_controls()

    def camera_command_done(self):
        if self._waiting:
            self._waiting = False
            self.status.setText('Capture failed: '+self.camera_window.status.text())
            self.update_controls()

    def accept_capture(self, payload):
        if not self._waiting or self._close_pending:
            return
        self._waiting = False
        lens = self.lens.text()
        self.live_framing.setChecked(False)
        self.start_job(lambda: self.session.add(payload, lens), self.sample_added,
                       'Detecting SB subpixel corners… / 正在检测SB亚像素角点…')

    def start_job(self, function, success, message):
        if self.job is not None:
            return
        self.status.setText(message)
        job = CalibrationJob(function, self)
        self.job = job
        job.succeeded.connect(success)
        job.failed.connect(self.job_failed)
        job.progress.connect(self.status.setText)
        job.finished.connect(self.job_finished)
        self.update_controls()
        job.start()

    def job_failed(self, text):
        self.status.setText(text)
        self.camera_window._message('Intrinsic calibration: '+text)

    def job_finished(self):
        job, self.job = self.job, None
        job.deleteLater()
        self.update_controls()
        if self._close_pending:
            self.close()

    def sample_added(self, _):
        self.settings.setValue('last_session', str(self.session.directory))
        self.render_samples()
        self.samples.setCurrentRow(len(self.session.samples)-1)
        self.status.setText('View saved. Change board position AND tilt for the next view. / 已保存，请改变位置和倾角后继续。')
        self.camera_window._message(f'Intrinsic view {len(self.session.samples)} saved with {len(self.session.samples[-1]["corners"])} corners: {self.session.manifest}')

    def render_samples(self):
        self._rendering = True
        selected = self.samples.currentRow()
        self.samples.clear()
        errors = {v['image']: v['rms_px'] for v in self.session.data.get('result', {}).get('views', [])}
        for i, sample in enumerate(self.session.samples):
            metric = sample['metrics']
            error = f" · RMS {errors[sample['image']]:.3f} px" if sample['image'] in errors else ''
            item = QListWidgetItem(f"{i+1:02d} · area {metric['area_fraction']*100:.1f}% · sharp {metric['sharpness']:.0f}{error}")
            item.setFlags(item.flags() | Qt.ItemIsUserCheckable)
            item.setCheckState(Qt.Checked if sample.get('enabled', True) else Qt.Unchecked)
            item.setToolTip(sample['image'])
            self.samples.addItem(item)
        self._rendering = False
        if self.session.samples:
            self.samples.setCurrentRow(max(0, min(selected, len(self.session.samples)-1)))
        else:
            self._image = QImage()
            self.preview.setText('Capture a native-resolution view / 采集完整分辨率图像')
        active = self.session.active
        size = self.session.data['identity']['image_size'] if self.session.data['identity'] else (1, 1)
        report = coverage(active, size)
        cells = ' / '.join(''.join('■' if row*3+col in report['corner_cells'] else '□' for col in range(3)) for row in range(3))
        legacy = sum('detector' not in sample for sample in active)
        legacy_text = f'\n{legacy} legacy views: use Reprocess saved images / 旧版角点请点击重新检测' if legacy else ''
        self.summary.setText(f'{len(active)} active / 有效视图 · minimum {MIN_VIEWS}, recommended {RECOMMENDED_VIEWS}–30\n'
                             f'Corner regions / 角点分区: {cells}\nCorner hull / 角点覆盖 {report["image_hull_fraction"]*100:.1f}%\n'
                             f'X/Y span / 跨度 {report["corner_span"][0]*100:.1f}% / {report["corner_span"][1]*100:.1f}%'
                             f'{legacy_text}')
        self.path_label.setText('Session / 采集目录: '+str(self.session.directory))
        self.render_result()
        self.update_controls()

    def toggle_sample(self, item):
        if self._rendering or self.job is not None:
            return
        try:
            self.session.set_enabled(self.samples.row(item), item.checkState() == Qt.Checked)
            self.render_samples()
            self.status.setText('Dataset changed; refit before exporting / 数据集已修改，请重新计算')
        except Exception as exc:
            self.status.setText(str(exc))

    def fit_intrinsics(self):
        self.start_job(self.session.solve, self.fit_done,
                       'Fitting camera intrinsics (no robot motion)… / 正在计算内参，不会移动机器人…')

    def reprocess_images(self):
        source, destination = self.session, self.new_directory()

        def process():
            target = source.reprocess(destination, self.job.progress.emit)
            fit_error = None
            if len(target.active) >= MIN_VIEWS:
                self.job.progress.emit('Refitting corrected corners… / 正在用重新检测的角点计算内参…')
                try:
                    target.solve()
                except Exception as exc:
                    fit_error = str(exc)
            return target, fit_error

        def complete(value):
            self.session, fit_error = value
            self.settings.setValue('last_session', str(self.session.directory))
            self.apply_session_settings()
            self.render_samples()
            message = f'Reprocessed copy saved / 重新检测副本已保存: {self.session.directory}. Original / 原始: {source.directory}'
            if 'result' in self.session.data:
                self.fit_done(None)
                message += f'. RMS {self.session.data["result"]["rms_px"]:.4f} px'
            if fit_error:
                message += '. Fit needs review / 拟合需检查: '+fit_error
            self.status.setText(message)
            self.camera_window._message(message)

        self.start_job(process, complete, 'Reprocessing saved images into a separate session… / 正在新建副本重新检测…')

    def fit_done(self, _):
        self.render_samples()
        self.status.setText('Result saved in session. Review warnings and per-view pixel errors before export. / 结果已保存，请检查警告及每张图像的像素误差。')
        result = self.session.data['result']
        self.camera_window._message(f'Intrinsics fitted: RMS {result["rms_px"]:.4f} px, {len(result["views"])} views, warnings={result["warnings"]}. {self.session.directory / "intrinsics.json"}')

    def render_result(self):
        result = self.session.data.get('result')
        if result is None:
            self.result.setText('No fitted result / 尚未计算')
            return
        k = np.array(result['K'])
        warnings = '\n'.join(result['warnings']) or 'Fit checks passed; independent validation still required / 拟合检查通过，仍需独立验证'
        self.result.setText(f'RMS {result["rms_px"]:.4f} px · max view {max(v["rms_px"] for v in result["views"]):.4f} px\n'
                            f'fx {k[0,0]:.2f}, fy {k[1,1]:.2f}\ncx {k[0,2]:.2f}, cy {k[1,2]:.2f}\n'
                            f'Normal spread / 法向变化 {result["normal_spread_deg"]:.1f}°\n{warnings}\n'
                            'Pixel fit error ≠ TCP accuracy. Not hand–eye calibration. / 像素误差不等于TCP精度，并非手眼标定。')

    def show_selected(self, *_):
        if self._rendering:
            return
        self.live_framing.blockSignals(True)
        self.live_framing.setChecked(False)
        self.live_framing.blockSignals(False)
        index = self.samples.currentRow()
        if not 0 <= index < len(self.session.samples):
            return
        sample = self.session.samples[index]
        image = cv2.imread(str(self.session.directory/sample['image']))
        if image is None:
            self.status.setText('Cannot read saved image / 无法读取样本图像')
            return
        result = self.session.data.get('result')
        if result is not None and self.undistort.isChecked():
            k, d = np.asarray(result['K']), np.asarray(result['D'])
            image = cv2.undistort(image, k, d, None, k)
            label = 'Undistorted selected image (same K) / 选中图像去畸变'
        else:
            corners = np.asarray(sample['corners'], np.float32).reshape(-1, 1, 2)
            cv2.drawChessboardCorners(image, self.session.board_size, corners, True)
            label = f'{len(corners)} subpixel inner corners / 亚像素内角点'
            working = sample.get('detector', {}).get('working_size')
            if working:
                label += f' · SB image {working[0]} × {working[1]}'
        h, w = image.shape[:2]
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        self._image = QImage(image.data, w, h, image.strides[0], QImage.Format_RGB888).copy()
        self.image_info.setText(f'{w} × {h} · {label}\n{sample["image"]}')
        self.paint_image()

    def show_live(self, image):
        if not self.live_framing.isChecked() or self._close_pending:
            return
        if image.ndim == 2:
            image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        self._image = QImage(rgb.data, rgb.shape[1], rgb.shape[0], rgb.strides[0], QImage.Format_RGB888).copy()
        self.image_info.setText('Live display only; Capture acquires a new native image. / 实时图仅用于取景，采集将读取原图。')
        self.paint_image()

    def paint_image(self):
        if not self._image.isNull():
            self.preview.setPixmap(QPixmap.fromImage(self._image).scaled(self.preview.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))

    def resizeEvent(self, event):
        super().resizeEvent(event)
        if hasattr(self, 'preview'):
            self.paint_image()

    def apply_session_settings(self):
        self.cols.setValue(self.session.board_size[0])
        self.rows.setValue(self.session.board_size[1])
        self.square.setValue(self.session.data['square_size_mm'])
        if self.session.data['identity']:
            self.lens.setText(self.session.data['identity']['lens_setup'])
        self.locked.setChecked(False)

    def new_session(self):
        if self.session.samples and QMessageBox.question(self, 'New session / 新建采集',
                'Start a new session? Existing images/results remain saved. / 新建采集？原有数据保留。') != QMessageBox.Yes:
            return
        self.session = IntrinsicSession(self.new_directory(), (self.cols.value(), self.rows.value()), self.square.value())
        self.locked.setChecked(False)
        self.settings.remove('last_session')
        self.render_samples()
        self.status.setText('New session. Check board settings before the first capture. / 新采集，请先核对棋盘参数。')

    def resume(self):
        directory = QFileDialog.getExistingDirectory(self, 'Session folder / 选择采集目录', str(calibration_root()), QFileDialog.DontUseNativeDialog)
        if directory:
            def loaded(session):
                self.session = session
                self.apply_session_settings()
                self.settings.setValue('last_session', str(session.directory))
                self.render_samples()
                self.status.setText('Session restored; confirm fixed lens setup / 已恢复，请确认镜头设置')
            self.start_job(lambda: IntrinsicSession.load(directory), loaded, 'Loading saved views… / 正在读取图像…')

    def export(self):
        path, _ = QFileDialog.getSaveFileName(self, 'Export intrinsics / 导出内参', str(self.session.directory/'camera-intrinsics.json'),
                                            'JSON (*.json);;OpenCV XML (*.xml)', options=QFileDialog.DontUseNativeDialog)
        if path:
            try:
                self.session.export(path)
                self.status.setText('Exported / 已导出: '+path)
            except Exception as exc:
                self.status.setText(str(exc))

    def closeEvent(self, event):
        if self.job is not None:
            self._close_pending = True
            self.status.setText('Finishing calibration job before closing… / 等待计算结束后关闭…')
            self.update_controls()
            event.ignore()
            return
        self._waiting = False
        self.camera_window.worker.command_done.disconnect(self.camera_command_done)
        self.camera_window.worker.preview_ready.disconnect(self.show_live)
        self.closed.emit()
        event.accept()
        self.deleteLater()
