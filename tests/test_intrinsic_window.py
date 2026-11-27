"""Native capture → background detection → durable session, without a camera/robot."""
import os
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

import threading
import time

import cv2
import numpy as np
from PySide6.QtCore import QSettings
from PySide6.QtTest import QTest
from PySide6.QtWidgets import QApplication

from pcbvision.camera_window import CameraWindow
from pcbvision.intrinsics import IntrinsicSession


class BoardCamera:
    def __init__(self):
        self.is_open = False
        self.is_streaming = False
        self.resolution = (2000, 1500)
        self.thread_ids = set()
        self.frame = np.full((1500, 2000, 3), 200, np.uint8)
        for y in range(9):
            for x in range(12):
                cv2.rectangle(self.frame, (200+x*100, 250+y*100),
                              (200+(x+1)*100, 250+(y+1)*100),
                              (0, 0, 0) if (x+y)%2 == 0 else (255, 255, 255), -1)

    def record(self):
        self.thread_ids.add(threading.get_ident())

    def enumerate_devices(self):
        self.record()
        return [{'name': 'Test Board Camera', 'sn': 'board-123', 'dev_info': 'board'}]

    def open(self, _):
        self.record()
        self.is_open = True

    def parameters(self):
        self.record()
        return {'resolution_id': 0, 'reverse_x': False, 'reverse_y': False,
                'acquisition_geometry': {'Width': 2000, 'Height': 1500}}

    def snapshot(self):
        self.record()
        return self.frame.copy()

    def close(self):
        self.record()
        self.is_open = self.is_streaming = False


def wait_for(predicate, seconds=8):
    deadline = time.monotonic()+seconds
    while time.monotonic() < deadline:
        QTest.qWait(20)
        if predicate():
            return
    raise AssertionError('Timed out waiting for calibration UI job')


def make_window(tmp_path, monkeypatch):
    import pcbvision.intrinsic_window as module
    monkeypatch.setattr(module, 'QSettings', lambda *_: QSettings(str(tmp_path/'settings.ini'), QSettings.IniFormat))
    fake = BoardCamera()
    window = CameraWindow(camera_factory=lambda: fake, camera_name='MindVision', preview_only=True)
    window.show()
    wait_for(lambda: not window._busy)
    window.open_button.click()
    wait_for(lambda: window._is_open and not window._busy)
    window.open_intrinsics()
    child = window.intrinsic_window
    child.session = IntrinsicSession(tmp_path/'session')
    child.render_samples()
    return window, child, fake


def test_native_capture_background_detection_resume_and_duplicate_rejection(tmp_path, monkeypatch):
    app = QApplication.instance() or QApplication([])
    window, child, fake = make_window(tmp_path, monkeypatch)
    try:
        assert not child.capture.isEnabled()
        child.locked.setChecked(True)
        child.live_framing.setChecked(True)
        window.worker.preview_ready.emit(cv2.resize(fake.frame, (800, 600)))
        app.processEvents()
        assert child.live_framing.isChecked() and child._image.width() == 800
        child.capture.click()
        wait_for(lambda: child.job is None and not child._waiting and len(child.session.samples) == 1)
        assert child.session.data['identity']['image_size'] == [2000, 1500]
        assert len(child.session.samples[0]['corners']) == 88
        assert child._image.width() == 2000  # never 1280-wide live preview coordinates
        assert not child.live_framing.isChecked()
        assert not child.fit.isEnabled() and not child.export_button.isEnabled()
        assert not child.cols.isEnabled() and not child.lens.isEnabled()
        assert threading.get_ident() not in fake.thread_ids
        child.capture.click()
        wait_for(lambda: child.job is None and not child._waiting)
        assert len(child.session.samples) == 1
        assert 'duplicates' in child.status.text()
        source = child.session
        original = source.manifest.read_bytes()
        child.reprocess_button.click()
        wait_for(lambda: child.job is None and child.session.directory != source.directory)
        assert source.manifest.read_bytes() == original
        assert child.session.data['reprocessed_from']['directory'] == str(source.directory)
        assert len(child.session.samples) == 1
        assert not child.locked.isChecked()
        assert 'Reprocessed copy saved' in child.status.text()
        child.close()
        assert window.intrinsic_window is None
        window.open_intrinsics()
        restored = window.intrinsic_window
        assert len(restored.session.samples) == 1
        assert restored.square.value() == 3.
        assert restored.cols.value() == 11 and restored.rows.value() == 8
        assert not restored.locked.isChecked()
    finally:
        window.close()
        wait_for(lambda: not window.worker.isRunning())
        app.processEvents()


def test_camera_close_waits_for_background_calibration_job(tmp_path, monkeypatch):
    app = QApplication.instance() or QApplication([])
    window, child, _ = make_window(tmp_path, monkeypatch)
    entered, release = threading.Event(), threading.Event()

    def work():
        entered.set()
        release.wait(4)
        return None

    try:
        child.start_job(work, lambda _: None, 'Testing close')
        wait_for(entered.is_set)
        window.close()
        assert child.job is not None and window.worker.isRunning()
        assert not child.capture.isEnabled()
        release.set()
        wait_for(lambda: not window.worker.isRunning())
        assert window.intrinsic_window is None
    finally:
        release.set()
        window.close()
        wait_for(lambda: not window.worker.isRunning())
        app.processEvents()
