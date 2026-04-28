"""
Camera abstraction for MindVision industrial cameras.

Wraps the mvsdk SDK behind a Qt-aware interface with:
  - QThread-based live view worker
  - Software trigger single-frame grab
  - Camera settings (exposure, gamma, contrast, gain, AE)
  - Qt signals for frame delivery and errors
"""

import sys
import os
import platform
from dataclasses import dataclass
from typing import Optional

import numpy as np

# Add driver directory to path for mvsdk import (not needed when frozen)
if not getattr(sys, 'frozen', False):
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'driver'))
import mvsdk

from PySide6.QtCore import QObject, Signal, Slot, QThread


# ── Data types ──────────────────────────────────────────────────────────

@dataclass
class CameraSettings:
    exposure_us: int = 30000
    gamma: int = 100
    contrast: int = 100
    analog_gain: int = 16
    ae_enabled: bool = False
    reverse_x: bool = False   # horizontal mirror (MIRROR_DIRECTION_HORIZONTAL)
    reverse_y: bool = False   # vertical mirror (MIRROR_DIRECTION_VERTICAL)


@dataclass
class CameraSettingRanges:
    exposure_min_us: int = 100
    exposure_max_us: int = 1000000
    exposure_step_us: int = 100
    gamma_min: int = 1
    gamma_max: int = 500
    contrast_min: int = 1
    contrast_max: int = 500
    analog_gain_min: int = 0
    analog_gain_max: int = 100


# ── Signal emitter (must be QObject to emit) ────────────────────────────

class CameraSignalEmitter(QObject):
    frame_ready = Signal(np.ndarray)   # live view frame (BGR)
    grab_done = Signal(np.ndarray)     # software-triggered frame (BGR)
    error = Signal(str)                # error message


# ── Live view thread (subclasses QThread for reliable cleanup) ──────────

class _LiveViewThread(QThread):
    """Continuously grabs frames from camera in trigger-mode-0."""
    frame_ready = Signal(np.ndarray)

    def __init__(self, camera: 'MindVisionCamera'):
        super().__init__()
        self._camera = camera
        self._running = False

    def run(self):
        """Poll CameraGetImageBuffer in a loop, emit numpy frames."""
        self._running = True
        while self._running and not self.isInterruptionRequested():
            try:
                frame = self._camera._grab_frame(timeout_ms=200)
                if frame is not None:
                    self.frame_ready.emit(frame)
            except mvsdk.CameraException as e:
                if e.error_code != mvsdk.CAMERA_STATUS_TIME_OUT:
                    self._camera._signals.error.emit(
                        f"Live view error: {e.message}")
            except RuntimeError:
                # Camera may have been closed
                break

    def stop(self):
        """Signal the thread to stop and wait for it."""
        self._running = False
        self.requestInterruption()
        self.wait(3000)  # 3s timeout


# ── Main camera class ──────────────────────────────────────────────────

class MindVisionCamera:
    """High-level camera abstraction wrapping the MindVision SDK."""

    def __init__(self):
        self._hCamera: Optional[int] = None
        self._pFrameBuffer = None
        self._cap = None
        self._mono: bool = False
        self._width: int = 0
        self._height: int = 0
        self._signals = CameraSignalEmitter()
        self._live_thread: Optional[_LiveViewThread] = None
        self._mode: str = "closed"   # "live", "trigger", "closed"

    # ── Connection ──────────────────────────────────────────────────

    def enumerate_devices(self) -> list[dict]:
        """Return list of available cameras as dicts."""
        try:
            DevList = mvsdk.CameraEnumerateDevice()
        except mvsdk.CameraException:
            return []

        result = []
        for dev in DevList:
            result.append({
                "name": dev.GetFriendlyName(),
                "sn": dev.GetSn(),
                "port_type": dev.GetPortType(),
                "dev_info": dev,
            })
        return result

    def open(self, dev_info) -> None:
        """Open camera, query capability, allocate frame buffer."""
        if self._hCamera is not None:
            self.close()

        try:
            self._hCamera = mvsdk.CameraInit(dev_info, -1, -1)
        except mvsdk.CameraException as e:
            self._signals.error.emit(f"CameraInit failed: {e.message}")
            raise

        self._cap = mvsdk.CameraGetCapability(self._hCamera)
        self._mono = (self._cap.sIspCapacity.bMonoSensor != 0)

        if self._mono:
            mvsdk.CameraSetIspOutFormat(self._hCamera, mvsdk.CAMERA_MEDIA_TYPE_MONO8)
        else:
            mvsdk.CameraSetIspOutFormat(self._hCamera, mvsdk.CAMERA_MEDIA_TYPE_BGR8)

        self._width = self._cap.sResolutionRange.iWidthMax
        self._height = self._cap.sResolutionRange.iHeightMax

        FrameBufferSize = self._width * self._height * (1 if self._mono else 3)
        self._pFrameBuffer = mvsdk.CameraAlignMalloc(FrameBufferSize, 16)

        # Apply default settings
        mvsdk.CameraSetAeState(self._hCamera, 0)
        mvsdk.CameraSetExposureTime(self._hCamera, 30000)

    def close(self) -> None:
        """Stop capture, uninit camera, free frame buffer."""
        self._stop_worker()
        if self._hCamera is not None:
            try:
                mvsdk.CameraStop(self._hCamera)
                mvsdk.CameraUnInit(self._hCamera)
            except mvsdk.CameraException:
                pass
            self._hCamera = None
        if self._pFrameBuffer is not None:
            mvsdk.CameraAlignFree(self._pFrameBuffer)
            self._pFrameBuffer = None
        self._mode = "closed"

    @property
    def is_open(self) -> bool:
        return self._hCamera is not None

    @property
    def resolution(self) -> tuple[int, int]:
        return self._width, self._height

    @property
    def signals(self) -> CameraSignalEmitter:
        return self._signals

    # ── Mode switching ──────────────────────────────────────────────

    def set_live_mode(self) -> None:
        """Switch to continuous capture mode, start live view worker."""
        if self._hCamera is None:
            return

        # Stop existing worker if running
        self._stop_worker()

        # Switch to continuous mode
        mvsdk.CameraStop(self._hCamera)
        mvsdk.CameraSetTriggerMode(self._hCamera, 0)
        mvsdk.CameraPlay(self._hCamera)

        # Start live view worker
        self._start_worker()
        self._mode = "live"

    def set_trigger_mode(self) -> None:
        """Switch to software trigger mode."""
        if self._hCamera is None:
            return

        # Stop live view worker
        self._stop_worker()

        # Switch to software trigger mode
        mvsdk.CameraStop(self._hCamera)
        mvsdk.CameraSetTriggerMode(self._hCamera, 1)
        mvsdk.CameraPlay(self._hCamera)

        self._mode = "trigger"

    def software_trigger(self) -> None:
        """Fire software trigger, grab one frame, emit grab_done."""
        if self._hCamera is None or self._mode != "trigger":
            self._signals.error.emit("Camera not in trigger mode")
            return

        try:
            mvsdk.CameraSoftTrigger(self._hCamera)
            frame = self._grab_frame(timeout_ms=2000)
            if frame is not None:
                self._signals.grab_done.emit(frame)
            else:
                self._signals.error.emit("Software trigger: no frame received")
        except mvsdk.CameraException as e:
            self._signals.error.emit(f"Software trigger error: {e.message}")

    # ── Settings ────────────────────────────────────────────────────

    def get_setting_ranges(self) -> CameraSettingRanges:
        """Query SDK for min/max/step of adjustable parameters."""
        if self._hCamera is None:
            return CameraSettingRanges()

        try:
            min_exp, max_exp, step_exp = mvsdk.CameraGetExposureTimeRange(self._hCamera)
            # SDK returns microseconds directly
            min_us = int(min_exp)
            max_us = int(max_exp)
            step_us = max(int(step_exp), 1)
        except mvsdk.CameraException:
            min_us, max_us, step_us = 100, 1000000, 100

        try:
            cap = self._cap
            gamma_min = cap.sGammaRange.iMin if hasattr(cap, 'sGammaRange') else 1
            gamma_max = cap.sGammaRange.iMax if hasattr(cap, 'sGammaRange') else 500
            contrast_min = cap.sContrastRange.iMin if hasattr(cap, 'sContrastRange') else 1
            contrast_max = cap.sContrastRange.iMax if hasattr(cap, 'sContrastRange') else 500
            gain_min = cap.sExposeDesc.uiAnalogGainMin if hasattr(cap, 'sExposeDesc') else 0
            gain_max = cap.sExposeDesc.uiAnalogGainMax if hasattr(cap, 'sExposeDesc') else 100
        except Exception:
            gamma_min, gamma_max = 1, 500
            contrast_min, contrast_max = 1, 500
            gain_min, gain_max = 0, 100

        return CameraSettingRanges(
            exposure_min_us=min_us,
            exposure_max_us=max_us,
            exposure_step_us=step_us,
            gamma_min=gamma_min,
            gamma_max=gamma_max,
            contrast_min=contrast_min,
            contrast_max=contrast_max,
            analog_gain_min=gain_min,
            analog_gain_max=gain_max,
        )

    def apply_settings(self, settings: CameraSettings) -> None:
        """Apply all settings to the camera hardware."""
        if self._hCamera is None:
            return

        try:
            mvsdk.CameraSetAeState(self._hCamera, 1 if settings.ae_enabled else 0)
            if not settings.ae_enabled:
                mvsdk.CameraSetExposureTime(self._hCamera, settings.exposure_us)
            mvsdk.CameraSetGamma(self._hCamera, settings.gamma)
            mvsdk.CameraSetContrast(self._hCamera, settings.contrast)
            mvsdk.CameraSetAnalogGain(self._hCamera, settings.analog_gain)
            mvsdk.CameraSetMirror(self._hCamera, 0, settings.reverse_x)   # 0 = horizontal
            mvsdk.CameraSetMirror(self._hCamera, 1, settings.reverse_y)   # 1 = vertical
        except mvsdk.CameraException as e:
            self._signals.error.emit(f"Setting error: {e.message}")

    def get_current_settings(self) -> CameraSettings:
        """Read current values from camera hardware."""
        if self._hCamera is None:
            return CameraSettings()

        try:
            ae = mvsdk.CameraGetAeState(self._hCamera)
            exposure_us = int(mvsdk.CameraGetExposureTime(self._hCamera))
            gamma = mvsdk.CameraGetGamma(self._hCamera)
            contrast = mvsdk.CameraGetContrast(self._hCamera)
            gain = mvsdk.CameraGetAnalogGain(self._hCamera)
            # CameraSetMirror dir=0 is horizontal, dir=1 is vertical
            reverse_x = bool(mvsdk.CameraGetMirror(self._hCamera, 0))
            reverse_y = bool(mvsdk.CameraGetMirror(self._hCamera, 1))
            return CameraSettings(
                exposure_us=exposure_us,
                gamma=gamma,
                contrast=contrast,
                analog_gain=gain,
                ae_enabled=(ae != 0),
                reverse_x=reverse_x,
                reverse_y=reverse_y,
            )
        except mvsdk.CameraException:
            return CameraSettings()

    # ── Internal helpers ────────────────────────────────────────────

    def _grab_frame(self, timeout_ms: int = 200) -> Optional[np.ndarray]:
        """Grab a single frame from camera, return as BGR numpy array."""
        if self._hCamera is None:
            return None

        pRawData, FrameHead = mvsdk.CameraGetImageBuffer(self._hCamera, timeout_ms)
        mvsdk.CameraImageProcess(self._hCamera, pRawData, self._pFrameBuffer, FrameHead)
        mvsdk.CameraReleaseImageBuffer(self._hCamera, pRawData)

        # Flip on Windows
        if platform.system() == "Windows":
            mvsdk.CameraFlipFrameBuffer(self._pFrameBuffer, FrameHead, 1)

        # Convert to numpy
        frame_data = (mvsdk.c_ubyte * FrameHead.uBytes).from_address(self._pFrameBuffer)
        frame = np.frombuffer(frame_data, dtype=np.uint8)

        if FrameHead.uiMediaType == mvsdk.CAMERA_MEDIA_TYPE_MONO8:
            frame = frame.reshape((FrameHead.iHeight, FrameHead.iWidth, 1))
        else:
            frame = frame.reshape((FrameHead.iHeight, FrameHead.iWidth, 3))

        return frame.copy()

    def _start_worker(self) -> None:
        """Create and start the live view thread."""
        self._live_thread = _LiveViewThread(self)
        self._live_thread.frame_ready.connect(self._signals.frame_ready)
        self._live_thread.start()

    def _stop_worker(self) -> None:
        """Stop the live view thread and wait for it to finish."""
        if self._live_thread is not None:
            self._live_thread.stop()
            self._live_thread = None
