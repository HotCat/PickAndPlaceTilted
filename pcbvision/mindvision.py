"""Synchronous MindVision adapter for CameraWorker (no SDK calls on the GUI thread).

Uses the repository's vendor Python wrapper with the user's unpacked Linux SDK.
Every native image buffer is returned even when ISP processing fails; published
numpy frames own their memory and survive the next acquisition or camera close.
"""
from __future__ import annotations

import importlib
from ctypes import c_ubyte

import numpy as np


def _load_sdk():
    try:
        return importlib.import_module("driver.mvsdk")
    except (OSError, ImportError) as exc:
        raise RuntimeError(
            f"Cannot load MindVision SDK: {exc}. Use --sdk-root /path/to/linuxSDK "
            "or set MINDVISION_SDK_ROOT (or MINDVISION_SDK_LIBRARY)."
        ) from exc


class MindVisionCamera:
    """Call only from the camera worker; sdk injection supports hardware-free tests."""

    def __init__(self, sdk=None):
        self.sdk = sdk if sdk is not None else _load_sdk()
        self._handle = None
        self._buffer = None
        self._capacity = 0
        self._cap = None
        self.is_streaming = False
        self.resolution = (0, 0)
        self._call("CameraSdkInit", 1)

    def _call(self, name, *args):
        # The vendor wrapper raises for GetImageBuffer/Init but only records
        # errors for most other APIs (including getters). Check those too.
        result = getattr(self.sdk, name)(*args)
        code = self.sdk.GetLastError()
        if code:
            raise self.sdk.CameraException(code)
        return result

    @property
    def is_open(self):
        return self._handle is not None

    def enumerate_devices(self):
        try:
            devices = self._call("CameraEnumerateDevice")
        except self.sdk.CameraException as exc:
            if exc.error_code == self.sdk.CAMERA_STATUS_NO_DEVICE_FOUND:
                return []
            raise
        return [{"name": f"{d.GetFriendlyName()} ({d.GetSn()})",
                 "sn": d.GetSn(), "dev_info": d} for d in devices]

    def open(self, dev_info):
        self.close()
        try:
            self._handle = self._call("CameraInit", dev_info, -1, -1)
            self._cap = self._call("CameraGetCapability", self._handle)
            self._mono = bool(self._cap.sIspCapacity.bMonoSensor)
            fmt = self.sdk.CAMERA_MEDIA_TYPE_MONO8 if self._mono else self.sdk.CAMERA_MEDIA_TYPE_BGR8
            self._call("CameraSetIspOutFormat", self._handle, fmt)
            size = self._call("CameraGetImageResolution", self._handle)
            self.resolution = (size.iWidth, size.iHeight)
            self._capacity = (self._cap.sResolutionRange.iWidthMax *
                              self._cap.sResolutionRange.iHeightMax * 3)
            self._buffer = self.sdk.CameraAlignMalloc(self._capacity, 16)
            if not self._buffer:
                raise MemoryError("MindVision image-buffer allocation failed")
            # Exposure/gain are read from the camera, never overwritten on open.
        except Exception:
            self.close()
            raise

    def start_live(self):
        if not self.is_open:
            raise RuntimeError("Open a camera first")
        if self.is_streaming:
            return
        self._call("CameraSetTriggerMode", self._handle, 0)
        self._call("CameraPlay", self._handle)
        self.is_streaming = True

    def stop_live(self):
        if self.is_open and self.is_streaming:
            self._call("CameraStop", self._handle)
        self.is_streaming = False

    def capture_frame(self, timeout_ms=200):
        if not self.is_streaming:
            raise RuntimeError("Start live view first")
        try:
            raw, head = self._call("CameraGetImageBuffer", self._handle, timeout_ms)
        except self.sdk.CameraException as exc:
            if exc.error_code == self.sdk.CAMERA_STATUS_TIME_OUT:
                return None
            raise
        try:
            channels = 1 if self._mono else 3
            length = head.iWidth * head.iHeight * channels
            if min(head.iWidth, head.iHeight) <= 0 or length > self._capacity:
                raise RuntimeError("Invalid MindVision frame dimensions")
            self._call("CameraImageProcess", self._handle, raw, self._buffer, head)
            native = (c_ubyte * length).from_address(self._buffer)
            frame = np.frombuffer(native, dtype=np.uint8).reshape(head.iHeight, head.iWidth, channels)
            # CameraWindow accepts BGR for both mono and color backends.
            frame = np.repeat(frame, 3, axis=2) if self._mono else frame.copy()
            self.resolution = (head.iWidth, head.iHeight)
            return frame
        finally:
            self._call("CameraReleaseImageBuffer", self._handle, raw)

    def snapshot(self):
        resume = self.is_streaming
        try:
            self.start_live()
            # Discard queued previews so a snapshot is a fresh full-size image.
            self._call("CameraClearBuffer", self._handle)
            exposure = self._call("CameraGetExposureTime", self._handle)
            frame = self.capture_frame(timeout_ms=max(1000, min(10000, int(exposure / 1000) + 500)))
            if frame is None:
                raise RuntimeError("MindVision snapshot timed out")
            return frame
        finally:
            if not resume:
                self.stop_live()

    def parameters(self):
        def optional(name, *args):
            try:
                return self._call(name, self._handle, *args)
            except self.sdk.CameraException as exc:
                if exc.error_code == self.sdk.CAMERA_STATUS_NOT_SUPPORTED:
                    return None
                raise

        def node(name, low, high):
            value = optional(name)
            return None if value is None else {"value": value, "min": low, "max": high}

        exposure_range = optional("CameraGetExposureTimeRange")
        cap = self._cap
        return {
            "exposure": node("CameraGetExposureTime", *exposure_range[:2]) if exposure_range else None,
            # Native analogue gain is an integer SDK index, not Hikvision dB.
            "gain": node("CameraGetAnalogGain", cap.sExposeDesc.uiAnalogGainMin, cap.sExposeDesc.uiAnalogGainMax),
            "gamma": node("CameraGetGamma", cap.sGammaRange.iMin, cap.sGammaRange.iMax),
            "auto_exposure": optional("CameraGetAeState"),
            "reverse_x": optional("CameraGetMirror", 0),
            "reverse_y": optional("CameraGetMirror", 1),
        }

    def apply_parameters(self, values):
        for key, name in (("auto_exposure", "CameraSetAeState"),
                          ("exposure", "CameraSetExposureTime"),
                          ("gain", "CameraSetAnalogGain"), ("gamma", "CameraSetGamma")):
            if key in values and not (key == "exposure" and values.get("auto_exposure")):
                value = values[key] if key == "exposure" else int(round(values[key]))
                self._call(name, self._handle, value)
        for index, key in enumerate(("reverse_x", "reverse_y")):
            if key in values:
                self._call("CameraSetMirror", self._handle, index, int(values[key]))

    def close(self):
        # Release allocation even if the camera was unplugged or Stop fails.
        try:
            if self._handle is not None:
                try:
                    self.stop_live()
                finally:
                    handle, self._handle = self._handle, None
                    self._call("CameraUnInit", handle)
        finally:
            self.is_streaming = False
            if self._buffer is not None:
                self.sdk.CameraAlignFree(self._buffer)
                self._buffer = None
            self.resolution = (0, 0)
