"""Synchronous MVS backend adapted from cadrefs/cadviewer/camera/hikvision.py.

The camera window owns this backend in one worker thread. SDK imports are lazy,
so file-based registration also works on machines without MVS installed.
"""
from __future__ import annotations

import importlib
import os
import sys
from ctypes import POINTER, c_bool, c_ubyte, cast
from pathlib import Path

import numpy as np


_sdk_initialized = False


def _load_sdk():
    root = Path(os.environ.get("HIKVISION_MVS_ROOT", "/opt/MVS"))
    os.environ.setdefault("MVCAM_COMMON_RUNENV", str(root / "lib"))
    sdk_dir = root / "Samples" / "64" / "Python" / "MvImport"
    if str(sdk_dir) not in sys.path:
        sys.path.insert(0, str(sdk_dir))
    try:
        return importlib.import_module("MvCameraControl_class")
    except Exception as exc:
        raise RuntimeError(
            f"Cannot load Hikrobot MVS SDK: {exc}. Install MVS or set "
            "HIKVISION_MVS_ROOT to its installation directory (default /opt/MVS)."
        ) from exc


def _check(result, action):
    if result != 0:
        raise RuntimeError(f"{action} failed: 0x{result & 0xffffffff:08x}")


def _text(value):
    data = bytes(value).split(b"\0", 1)[0]
    for encoding in ("utf-8", "gbk", "latin-1"):
        try:
            return data.decode(encoding).strip()
        except UnicodeDecodeError:
            pass


class HikvisionCamera:
    def __init__(self):
        global _sdk_initialized
        self.sdk = _load_sdk()
        if not _sdk_initialized:
            _check(self.sdk.MvCamera.MV_CC_Initialize(), "Initialize MVS")
            _sdk_initialized = True
        self._cam = None
        self.is_streaming = False
        self.resolution = (0, 0)
        self._buffer = None

    @property
    def is_open(self):
        return self._cam is not None

    def enumerate_devices(self):
        sdk = self.sdk
        listing = sdk.MV_CC_DEVICE_INFO_LIST()
        transports = sdk.MV_GIGE_DEVICE | sdk.MV_USB_DEVICE
        for name in ("MV_GENTL_GIGE_DEVICE", "MV_GENTL_CAMERALINK_DEVICE",
                     "MV_GENTL_CXP_DEVICE", "MV_GENTL_XOF_DEVICE"):
            transports |= getattr(sdk, name, 0)
        _check(sdk.MvCamera.MV_CC_EnumDevices(transports, listing), "Find cameras")
        devices = []
        for i in range(listing.nDeviceNum):
            info = cast(listing.pDeviceInfo[i], POINTER(sdk.MV_CC_DEVICE_INFO)).contents
            # Enumeration owns the original pointer; retain an independent structure.
            info = sdk.MV_CC_DEVICE_INFO.from_buffer_copy(info)
            transport = int(info.nTLayerType)
            members = (("MV_GIGE_DEVICE", "stGigEInfo"), ("MV_GENTL_GIGE_DEVICE", "stGigEInfo"),
                       ("MV_USB_DEVICE", "stUsb3VInfo"), ("MV_GENTL_CAMERALINK_DEVICE", "stCMLInfo"),
                       ("MV_GENTL_CXP_DEVICE", "stCXPInfo"), ("MV_GENTL_XOF_DEVICE", "stXoFInfo"))
            name, serial = "Hikvision camera", ""
            for constant, member in members:
                if transport == getattr(sdk, constant, -1):
                    details = getattr(info.SpecialInfo, member)
                    name, serial = _text(details.chModelName), _text(details.chSerialNumber)
                    break
            devices.append({"name": f"{name} ({serial})" if serial else name,
                            "sn": serial, "dev_info": info})
        return devices

    def open(self, dev_info):
        self.close()
        self._cam = self.sdk.MvCamera()
        try:
            create = getattr(self._cam, "MV_CC_CreateHandleWithoutLog", self._cam.MV_CC_CreateHandle)
            _check(create(dev_info), "Create camera handle")
            _check(self._cam.MV_CC_OpenDevice(self.sdk.MV_ACCESS_Exclusive, 0), "Open camera")
            if int(dev_info.nTLayerType) in (self.sdk.MV_GIGE_DEVICE,
                                           getattr(self.sdk, "MV_GENTL_GIGE_DEVICE", -1)):
                packet = self._cam.MV_CC_GetOptimalPacketSize()
                if packet > 0:
                    self._cam.MV_CC_SetIntValue("GevSCPSPacketSize", packet)
            self.resolution = (self._get_int("Width"), self._get_int("Height"))
            if min(self.resolution) <= 0:
                raise RuntimeError("Camera width/height unavailable")
            self._buffer = (c_ubyte * (self.resolution[0] * self.resolution[1] * 3))()
        except Exception:
            self.close()
            raise

    def start_live(self):
        if not self.is_open:
            raise RuntimeError("Open a camera first")
        self.stop_live()
        _check(self._cam.MV_CC_SetEnumValueByString("AcquisitionMode", "Continuous"), "Set continuous acquisition")
        _check(self._cam.MV_CC_SetEnumValue("TriggerMode", self.sdk.MV_TRIGGER_MODE_OFF), "Disable trigger")
        self._cam.MV_CC_SetGrabStrategy(self.sdk.MV_GrabStrategy_LatestImagesOnly)
        _check(self._cam.MV_CC_StartGrabbing(), "Start live view")
        self.is_streaming = True

    def stop_live(self):
        if self.is_streaming and self._cam is not None:
            _check(self._cam.MV_CC_StopGrabbing(), "Stop acquisition")
        self.is_streaming = False

    def capture_frame(self, timeout_ms=200):
        if not self.is_streaming:
            return None
        info = self.sdk.MV_FRAME_OUT_INFO_EX()
        result = self._cam.MV_CC_GetImageForBGR(self._buffer, len(self._buffer), info, timeout_ms)
        if result == self.sdk.MV_E_NODATA:
            return None
        _check(result, "Read camera frame")
        width, height = int(info.nWidth), int(info.nHeight)
        if width <= 0 or height <= 0 or width * height * 3 > len(self._buffer):
            raise RuntimeError("Unexpected camera frame dimensions")
        return np.frombuffer(self._buffer, dtype=np.uint8, count=width * height * 3).reshape(height, width, 3).copy()

    def snapshot(self):
        """Trigger a fresh native-resolution frame; never substitute a preview."""
        if not self.is_open:
            raise RuntimeError("Open a camera first")
        resume = self.is_streaming
        self.stop_live()
        try:
            _check(self._cam.MV_CC_SetEnumValue("TriggerMode", self.sdk.MV_TRIGGER_MODE_ON), "Enable trigger")
            _check(self._cam.MV_CC_SetEnumValue("TriggerSource", self.sdk.MV_TRIGGER_SOURCE_SOFTWARE), "Select software trigger")
            self._cam.MV_CC_SetGrabStrategy(self.sdk.MV_GrabStrategy_OneByOne)
            _check(self._cam.MV_CC_StartGrabbing(), "Start snapshot acquisition")
            self.is_streaming = True
            self._cam.MV_CC_ClearImageBuffer()
            _check(self._cam.MV_CC_SetCommandValue("TriggerSoftware"), "Trigger snapshot")
            exposure = self._float_node("ExposureTime")
            timeout = max(2000, int(exposure["value"] / 1000 + 1000) if exposure else 2000)
            frame = self.capture_frame(timeout_ms=timeout)
            if frame is None:
                raise RuntimeError("Snapshot timed out; no new image was received")
            return frame
        finally:
            self.stop_live()
            self._cam.MV_CC_SetEnumValue("TriggerMode", self.sdk.MV_TRIGGER_MODE_OFF)
            if resume:
                self.start_live()

    def parameters(self):
        if not self.is_open:
            return {}
        auto = self.sdk.MVCC_ENUMVALUE()
        auto_value = (int(auto.nCurValue) if self._cam.MV_CC_GetEnumValue("ExposureAuto", auto) == 0 else None)
        return {
            "exposure": self._float_node("ExposureTime"),
            "gain": self._float_node("Gain"),
            "gamma": self._float_node("Gamma"),
            "auto_exposure": None if auto_value is None else auto_value != 0,
            "reverse_x": self._bool_node("ReverseX"),
            "reverse_y": self._bool_node("ReverseY"),
        }

    def apply_parameters(self, values):
        if not self.is_open:
            raise RuntimeError("Open a camera first")
        failures = []
        def record(result, name):
            if result != 0:
                failures.append(f"{name}: 0x{result & 0xffffffff:08x}")
        if "auto_exposure" in values:
            record(self._cam.MV_CC_SetEnumValueByString(
                "ExposureAuto", "Continuous" if values["auto_exposure"] else "Off"), "Auto exposure")
        for key, node in (("exposure", "ExposureTime"), ("gain", "Gain"), ("gamma", "Gamma")):
            if key not in values or (key == "exposure" and values.get("auto_exposure")):
                continue
            if key == "gain":
                self._cam.MV_CC_SetEnumValueByString("GainAuto", "Off")
            if key == "gamma" and self._bool_node("GammaEnable") is not None:
                record(self._cam.MV_CC_SetBoolValue("GammaEnable", True), "Gamma enable")
            record(self._cam.MV_CC_SetFloatValue(node, float(values[key])), node)
        for key, node in (("reverse_x", "ReverseX"), ("reverse_y", "ReverseY")):
            if key in values:
                record(self._cam.MV_CC_SetBoolValue(node, bool(values[key])), node)
        if failures:
            raise RuntimeError("Some parameters were rejected: " + "; ".join(failures))

    def _get_int(self, node):
        value = self.sdk.MVCC_INTVALUE_EX()
        _check(self._cam.MV_CC_GetIntValueEx(node, value), f"Read {node}")
        return int(value.nCurValue)

    def _float_node(self, node):
        value = self.sdk.MVCC_FLOATVALUE()
        if self._cam.MV_CC_GetFloatValue(node, value) != 0:
            return None
        return {"value": float(value.fCurValue), "min": float(value.fMin), "max": float(value.fMax)}

    def _bool_node(self, node):
        value = c_bool()
        return bool(value.value) if self._cam.MV_CC_GetBoolValue(node, value) == 0 else None

    def close(self):
        if self._cam is None:
            return
        try:
            self.stop_live()
        finally:
            try:
                self._cam.MV_CC_CloseDevice()
            finally:
                self._cam.MV_CC_DestroyHandle()
                self._cam = None
                self._buffer = None
                self.is_streaming = False
