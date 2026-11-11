"""Check native-buffer ownership and fresh software-trigger acquisition without hardware."""
import unittest
from ctypes import c_ubyte
from types import SimpleNamespace

import numpy as np

from pcbvision.hikvision import HikvisionCamera


class NativeCamera:
    def __init__(self):
        self.calls = []
        self.value = 17
        self.timeout = False

    def MV_CC_GetImageForBGR(self, buffer, length, info, timeout):
        self.calls.append(("read", timeout))
        if self.timeout:
            return 7
        np.frombuffer(buffer, np.uint8)[:] = self.value
        info.nWidth, info.nHeight = 4, 3
        return 0

    def MV_CC_GetFloatValue(self, key, value):
        value.fCurValue, value.fMin, value.fMax = 30000., 10., 1000000.
        return 0

    def __getattr__(self, name):
        if name.startswith("MV_CC_"):
            def call(*args):
                self.calls.append((name, *args))
                return 0
            return call
        raise AttributeError(name)


class HikvisionBufferTests(unittest.TestCase):
    def setUp(self):
        self.camera = HikvisionCamera.__new__(HikvisionCamera)
        self.native = NativeCamera()
        self.camera._cam = self.native
        self.camera.sdk = SimpleNamespace(
            MV_FRAME_OUT_INFO_EX=lambda: SimpleNamespace(nWidth=0, nHeight=0),
            MVCC_FLOATVALUE=lambda: SimpleNamespace(fCurValue=0., fMin=0., fMax=0.),
            MV_E_NODATA=7, MV_TRIGGER_MODE_OFF=0, MV_TRIGGER_MODE_ON=1,
            MV_TRIGGER_SOURCE_SOFTWARE=7, MV_GrabStrategy_OneByOne=0,
            MV_GrabStrategy_LatestImagesOnly=2,
        )
        self.camera._buffer = (c_ubyte * 36)()
        self.camera.resolution = (4, 3)
        self.camera.is_streaming = True

    def test_returned_frame_owns_memory_after_next_sdk_read(self):
        first = self.camera.capture_frame()
        self.native.value = 99
        second = self.camera.capture_frame()
        self.assertEqual(first.shape, (3, 4, 3))
        self.assertTrue(np.all(first == 17))
        self.assertTrue(np.all(second == 99))

    def test_snapshot_triggers_fresh_frame_and_resumes_live_view(self):
        frame = self.camera.snapshot()
        self.assertTrue(np.all(frame == 17))
        self.assertTrue(self.camera.is_streaming)
        calls = self.native.calls
        trigger = calls.index(("MV_CC_SetCommandValue", "TriggerSoftware"))
        read = next(i for i, call in enumerate(calls) if call[0] == "read")
        self.assertLess(trigger, read)
        self.assertIn(("MV_CC_ClearImageBuffer",), calls[:trigger])
        self.assertEqual(calls[-1], ("MV_CC_StartGrabbing",))

    def test_snapshot_from_paused_camera_stays_paused(self):
        self.camera.is_streaming = False
        self.camera.snapshot()
        self.assertFalse(self.camera.is_streaming)

    def test_snapshot_timeout_reports_failure_and_resumes_live(self):
        self.native.timeout = True
        with self.assertRaisesRegex(RuntimeError, "timed out"):
            self.camera.snapshot()
        self.assertTrue(self.camera.is_streaming)


if __name__ == "__main__":
    unittest.main()
