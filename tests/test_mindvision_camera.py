"""Exercise native SDK ownership, timeout/error handling and camera state offline."""
from ctypes import addressof, c_ubyte
from types import SimpleNamespace as NS

import numpy as np
import pytest

from pcbvision.mindvision import MindVisionCamera


class CameraError(Exception):
    def __init__(self, code):
        self.error_code = code
        super().__init__(f"SDK {code}")


class FakeSDK:
    CAMERA_STATUS_TIME_OUT = -12
    CAMERA_STATUS_NO_DEVICE_FOUND = -16
    CAMERA_STATUS_NOT_SUPPORTED = -4
    CAMERA_MEDIA_TYPE_MONO8 = 1
    CAMERA_MEDIA_TYPE_BGR8 = 3
    CameraException = CameraError

    def __init__(self, mono=False):
        self.native = (c_ubyte * 36)()
        self.calls = []
        self.errors = {}
        self.last_error = 0
        self.timeout = False
        self.mono = mono
        self.value = 17
        self.presets = [NS(iIndex=0, iWidth=4, iHeight=3, GetDescription=lambda: "Full"),
                        NS(iIndex=1, iWidth=2, iHeight=2, GetDescription=lambda: "Small")]
        self.current_size = self.presets[0]

    def GetLastError(self):
        return self.last_error

    def CameraAlignMalloc(self, size, align):
        self.calls.append(("allocate", size))
        return addressof(self.native)

    def CameraAlignFree(self, pointer):
        self.calls.append(("free", pointer))

    def __getattr__(self, name):
        def call(*args):
            self.calls.append((name, *args))
            self.last_error = self.errors.get(name, 0)
            if name == "CameraInit":
                return 7
            if name == "CameraGetCapability":
                return NS(sIspCapacity=NS(bMonoSensor=self.mono),
                          sResolutionRange=NS(iWidthMax=4, iHeightMax=3),
                          sExposeDesc=NS(uiAnalogGainMin=1, uiAnalogGainMax=64),
                          sGammaRange=NS(iMin=1, iMax=500),
                          sContrastRange=NS(iMin=0, iMax=255),
                          pImageSizeDesc=self.presets, iImageSizeDesc=len(self.presets))
            if name == "CameraGetImageResolution":
                return self.current_size
            if name == "CameraSetImageResolution":
                self.current_size = args[1]
            if name == "CameraGetImageBuffer":
                if self.timeout:
                    raise CameraError(-12)
                return 123, NS(iWidth=4, iHeight=3)
            if name == "CameraImageProcess":
                np.frombuffer(self.native, dtype=np.uint8)[:] = self.value
            if name == "CameraGetExposureTimeRange":
                return 10., 100000., 1.
            if name == "CameraGetExposureTime":
                return 30000.
            return 0
        return call


@pytest.fixture
def camera():
    sdk = FakeSDK()
    camera = MindVisionCamera(sdk)
    camera.open("device")
    camera.start_live()
    yield camera, sdk
    camera.close()


@pytest.mark.parametrize("mono", [False, True])
def test_owned_bgr_frames_survive_next_capture_and_close(mono):
    sdk = FakeSDK(mono)
    camera = MindVisionCamera(sdk)
    camera.open("device")
    camera.start_live()
    first = camera.capture_frame()
    sdk.value = 90
    second = camera.capture_frame()
    camera.close()
    assert first.shape == (3, 4, 3)
    assert np.all(first == 17) and np.all(second == 90)
    assert len([c for c in sdk.calls if c[0] == "CameraReleaseImageBuffer"]) == 2
    assert not any(c[0] == "CameraSetExposureTime" for c in sdk.calls)


def test_isp_failure_still_releases_raw_buffer(camera):
    c, sdk = camera
    sdk.errors["CameraImageProcess"] = -1
    with pytest.raises(CameraError):
        c.capture_frame()
    assert sdk.calls[-1] == ("CameraReleaseImageBuffer", 7, 123)


def test_timeout_returns_none_but_transport_error_is_not_hidden(camera):
    c, sdk = camera
    sdk.timeout = True
    assert c.capture_frame() is None
    sdk.timeout = False
    sdk.errors["CameraGetImageBuffer"] = -38
    with pytest.raises(CameraError):
        c.capture_frame()


def test_snapshot_clears_queued_images_and_restores_paused_state(camera):
    c, sdk = camera
    c.stop_live()
    sdk.calls.clear()
    assert c.snapshot().shape == (3, 4, 3)
    assert not c.is_streaming
    names = [call[0] for call in sdk.calls]
    assert names.index("CameraClearBuffer") < names.index("CameraGetImageBuffer")
    assert names[-1] == "CameraStop"


def test_close_frees_allocation_even_when_stop_fails(camera):
    c, sdk = camera
    sdk.errors["CameraStop"] = -38
    with pytest.raises(CameraError):
        c.close()
    assert not c.is_open and not c.is_streaming
    assert sdk.calls[-1][0] == "free"
    assert any(call[0] == "CameraUnInit" for call in sdk.calls)
    c.close()  # idempotent cleanup after unplug


def test_partial_open_failure_releases_handle(camera):
    c, sdk = camera
    sdk.errors["CameraSetIspOutFormat"] = -1
    with pytest.raises(CameraError):
        c.open("device")
    assert not c.is_open
    assert sdk.calls[-1] == ("CameraUnInit", 7)


def test_parameter_units_and_setter_errors(camera):
    c, sdk = camera
    assert c.parameters()["gain"]["max"] == 64
    c.apply_parameters({"auto_exposure": True, "exposure": 1000., "gain": 3.8})
    assert ("CameraSetAnalogGain", 7, 4) in sdk.calls
    assert not any(call[0] == "CameraSetExposureTime" for call in sdk.calls)
    sdk.errors["CameraSetGamma"] = -7
    with pytest.raises(CameraError):
        c.apply_parameters({"gamma": 150})


def test_resolution_uses_complete_camera_preset_and_updates_readback(camera):
    c, sdk = camera
    params = c.parameters()
    assert len(params["resolutions"]) == 2
    assert params["resolution_id"] == 0
    with pytest.raises(RuntimeError, match="Stop acquisition"):
        c.apply_parameters({"resolution_id": 1})
    c.stop_live()
    c.apply_parameters({"resolution_id": 1})
    assert c.resolution == (2, 2)
    assert c.parameters()["resolution_id"] == 1
    assert ("CameraSetImageResolution", 7, c._presets[1]) in sdk.calls
    with pytest.raises(ValueError):
        c.apply_parameters({"resolution_id": -1})


def test_gamma_contrast_select_dynamic_lut_and_manual_exposure_disables_ae(camera):
    c, sdk = camera
    c.apply_parameters({"auto_exposure": False, "exposure": 2345., "gamma": 110, "contrast": 120})
    assert ("CameraSetLutMode", 7, 0) in sdk.calls
    assert ("CameraSetGamma", 7, 110) in sdk.calls
    assert ("CameraSetContrast", 7, 120) in sdk.calls
    assert sdk.calls.index(("CameraSetAeState", 7, 0)) < sdk.calls.index(("CameraSetExposureTime", 7, 2345.))
