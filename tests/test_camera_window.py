"""Exercise asynchronous camera controls and selector delivery with a fake camera."""
import os
import sys
import threading
import time
import unittest
from unittest.mock import patch

import numpy as np

try:
    from PySide6.QtTest import QTest
    from PySide6.QtWidgets import QApplication
except ImportError:
    QApplication = None
else:
    from pcbvision.camera_window import CameraWindow
    from tools.select_pcb_holes import HoleSelector
    from pcbvision.cad import BoardModel, CircleFeature


class FakeCamera:
    def __init__(self):
        self.is_open = False
        self.is_streaming = False
        self.resolution = (1600, 1200)
        self.thread_ids = set()
        self.applied = None
        self.snapshot_entered = threading.Event()
        self.snapshot_release = threading.Event()
        self.snapshot_release.set()
        self.fail_snapshot = False

    def record(self):
        self.thread_ids.add(threading.get_ident())

    def enumerate_devices(self):
        self.record()
        return [{"name": "Test Hikvision (123)", "sn": "123", "dev_info": "device"}]

    def open(self, device):
        self.record()
        assert device == "device"
        self.is_open = True

    def parameters(self):
        self.record()
        return {"exposure": {"value": 30000., "min": 100., "max": 1000000.},
                "gain": {"value": 3.5, "min": 0., "max": 24.}, "gamma": None,
                "auto_exposure": False, "reverse_x": False, "reverse_y": None}

    def apply_parameters(self, values):
        self.record()
        self.applied = values

    def start_live(self):
        self.record()
        self.is_streaming = True

    def stop_live(self):
        self.record()
        self.is_streaming = False

    def capture_frame(self, timeout_ms=200):
        self.record()
        return np.full((1200, 1600, 3), 9, np.uint8)

    def snapshot(self):
        self.record()
        self.snapshot_entered.set()
        if not self.snapshot_release.wait(2):
            raise RuntimeError("Fake snapshot blocked")
        if self.fail_snapshot:
            raise RuntimeError("Snapshot timed out")
        return np.full((1200, 1600, 3), (20, 60, 100), np.uint8)

    def close(self):
        self.record()
        self.is_streaming = self.is_open = False


@unittest.skipIf(QApplication is None, "PySide6 is not installed")
class CameraWindowTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        os.environ["QT_QPA_PLATFORM"] = "offscreen"
        cls.app = QApplication.instance() or QApplication([])

    def setUp(self):
        self.errors = []
        self.old_hook = sys.excepthook
        sys.excepthook = lambda *args: self.errors.append(args)
        self.fake = FakeCamera()
        self.window = CameraWindow(camera_factory=lambda: self.fake)
        self.worker = self.window.worker
        self.window.show()
        self.wait_until(lambda: self.window.open_button.isEnabled())

    def tearDown(self):
        self.fake.snapshot_release.set()
        self.window.close()
        self.wait_until(lambda: not self.worker.isRunning())
        self.app.processEvents()
        sys.excepthook = self.old_hook
        self.assertEqual(self.errors, [])

    def wait_until(self, predicate):
        deadline = time.monotonic() + 3
        while not predicate() and time.monotonic() < deadline:
            QTest.qWait(10)
        self.assertTrue(predicate())

    def open_camera(self):
        self.window.open_button.click()
        self.wait_until(lambda: self.window.snapshot_button.isEnabled())

    def test_controls_parameters_live_and_native_resolution_snapshot(self):
        self.assertFalse(self.window.snapshot_button.isEnabled())
        self.open_camera()
        self.assertFalse(self.window.float_controls["gamma"].isEnabled())
        self.assertFalse(self.window.reverse_y.isEnabled())
        self.window.auto_exposure.setChecked(True)
        self.assertFalse(self.window.float_controls["exposure"].isEnabled())
        self.window.auto_exposure.setChecked(False)
        self.window.float_controls["gain"].setValue(5.25)
        self.window.apply_button.click()
        self.wait_until(lambda: not self.window._busy)
        self.assertEqual(self.fake.applied["gain"], 5.25)
        self.assertNotIn("gamma", self.fake.applied)
        self.window.live_button.click()
        self.wait_until(lambda: not self.window._preview_image.isNull())
        self.assertEqual(self.window._preview_image.width(), 1280)
        snapshots = []
        self.window.snapshot_ready.connect(snapshots.append)
        self.window.snapshot_button.click()
        self.wait_until(lambda: bool(snapshots) and not self.window._busy)
        self.assertEqual(snapshots[0].shape, (1200, 1600, 3))
        np.testing.assert_array_equal(snapshots[0][0, 0], [20, 60, 100])
        self.assertTrue(self.window._is_live)
        self.window.live_button.click()
        self.wait_until(lambda: not self.window._is_live and not self.window._busy)
        self.window.disconnect_button.click()
        self.wait_until(lambda: not self.window._is_open and not self.window._busy)
        self.assertEqual(len(self.fake.thread_ids), 1)
        self.assertNotIn(threading.get_ident(), self.fake.thread_ids)

    def test_failed_snapshot_does_not_send_preview_as_snapshot(self):
        self.open_camera()
        self.fake.fail_snapshot = True
        snapshots = []
        self.window.snapshot_ready.connect(snapshots.append)
        self.window.snapshot_button.click()
        self.wait_until(lambda: not self.window._busy)
        self.assertEqual(snapshots, [])
        self.assertIn("timed out", self.window.status.text())
        self.assertTrue(self.window.snapshot_button.isEnabled())

    def test_close_during_snapshot_waits_for_worker_and_discards_late_frame(self):
        self.open_camera()
        self.fake.snapshot_release.clear()
        snapshots = []
        self.window.snapshot_ready.connect(snapshots.append)
        self.window.snapshot_button.click()
        self.wait_until(self.fake.snapshot_entered.is_set)
        self.window.close()
        self.assertTrue(self.worker.isRunning())
        self.assertTrue(self.window._closing)
        self.fake.snapshot_release.set()
        self.wait_until(lambda: not self.worker.isRunning() and not self.window.isVisible())
        self.assertEqual(snapshots, [])
        self.assertFalse(self.fake.is_open)

    def test_snapshot_loads_into_selector_and_keeps_recipe_without_registering(self):
        root = HoleSelector()
        board = BoardModel(np.zeros(2), [], np.array([[1., 15.], [33., 15.], [1., 2.], [33., 2.]]),
                           np.full(4, .6), ["a", "b", "c", "d"], np.array([34., 21.]), "mm",
                           [CircleFeature(np.array([8., 8.]), .4, "probe", "TOPSOLDER")])
        with patch("tools.select_pcb_holes.load_board", return_value=board):
            root.load_path("board.dxf")
        root.canvas.assignments = {i: i for i in range(4)}
        root.update_assignments()
        root.extra_anchors.append({"feature_index": 0, "cad_center": np.array([8., 8.]),
                                   "clicked_image_px": np.array([200., 200.]),
                                   "image_center_px": np.array([200., 200.]), "refined": False})
        self.window.snapshot_ready.connect(root.load_camera_frame)
        self.open_camera()
        with patch("tools.select_pcb_holes.detect_anchor_centers") as detector:
            self.window.snapshot_button.click()
            self.wait_until(lambda: root.gray_image is not None)
            detector.assert_not_called()
        self.assertEqual(root.gray_image.shape, (1200, 1600))
        np.testing.assert_array_equal(root.source_bgr_image[0, 0], [20, 60, 100])
        self.assertEqual(len(root.extra_anchors), 1)
        self.assertIsNone(root.extra_anchors[0]["clicked_image_px"])
        self.assertIsNone(root.image_path)
        self.assertIn("Hikvision snapshot", root.image_source)
        self.assertTrue(root.register_button.isEnabled())
        self.assertNotIn("ms", root.runtime_label.text())
        root.close()

    def test_missing_sdk_is_reported_and_find_can_retry(self):
        self.window.worker._factory = lambda: (_ for _ in ()).throw(RuntimeError("SDK missing"))
        # Use a separate worker/window so the already-created fake isn't reused.
        other = CameraWindow(camera_factory=self.window.worker._factory)
        other_worker = other.worker
        try:
            self.wait_until(lambda: not other._busy)
            self.assertIn("SDK missing", other.status.text())
            self.assertTrue(other.refresh_button.isEnabled())
            self.assertFalse(other.open_button.isEnabled())
        finally:
            other.close()
            self.wait_until(lambda: not other_worker.isRunning())

    def test_main_window_owns_single_camera_window_and_waits_for_camera_on_exit(self):
        root = HoleSelector()
        root.show()
        fake = FakeCamera()
        with patch("pcbvision.camera_window.CameraWindow",
                   side_effect=lambda parent: CameraWindow(parent, camera_factory=lambda: fake)) as factory:
            try:
                root.camera_button.click()
                camera_window = root.camera_window
                self.wait_until(lambda: camera_window.open_button.isEnabled())
                root.camera_button.click()
                self.assertEqual(factory.call_count, 1)
                self.assertIs(root.camera_window, camera_window)
                camera_window.open_button.click()
                self.wait_until(lambda: camera_window.snapshot_button.isEnabled())
                camera_window.close()
                self.wait_until(lambda: root.camera_window is None)
                self.assertTrue(root.isVisible())
                root.camera_button.click()
                self.assertEqual(factory.call_count, 2)
                second_window = root.camera_window
                self.wait_until(lambda: second_window.open_button.isEnabled())
                second_window.open_button.click()
                self.wait_until(lambda: second_window.snapshot_button.isEnabled())
                second_window.live_button.click()
                self.wait_until(lambda: second_window._is_live)
                root.close()
                self.wait_until(lambda: root.camera_window is None and not root.isVisible())
                self.assertFalse(fake.is_open)
                self.assertFalse(fake.is_streaming)
            finally:
                root.close()
                self.wait_until(lambda: root.camera_window is None)


if __name__ == "__main__":
    unittest.main()
