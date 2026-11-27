"""Save/load through the real camera worker with a stateful fake SDK."""
import copy
import json
import os
import threading
import time

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
from PySide6.QtTest import QTest
from PySide6.QtWidgets import QApplication
import numpy as np
import pytest

from pcbvision.camera_window import CameraWindow
from test_camera_profiles import parameters


class ProfileCamera:
    def __init__(self):
        self.is_open = self.is_streaming = False
        self.resolution = (2592, 1944)
        self.values = parameters()
        self.applied = []
        self.thread_ids = set()
        self.fail_apply = False
        self.entered = threading.Event()
        self.release = threading.Event()
        self.release.set()

    def record(self):
        self.thread_ids.add(threading.get_ident())

    def enumerate_devices(self):
        self.record()
        return [{'name': 'Profile camera', 'sn': '123', 'dev_info': 'device'}]

    def open(self, _):
        self.record()
        self.is_open = True

    def parameters(self):
        self.record()
        return copy.deepcopy(self.values)

    def apply_parameters(self, values):
        self.record()
        self.entered.set()
        self.release.wait(4)
        if self.fail_apply:
            raise RuntimeError('SDK rejected parameters')
        self.applied.append(dict(values))
        for key, value in values.items():
            if key in ('gain', 'exposure', 'gamma', 'contrast'):
                self.values[key]['value'] = value
            elif key == 'resolution_id':
                mode = parameters(value)
                self.values.update({k: mode[k] for k in ('resolution_id', 'acquisition_geometry')})
                self.resolution = (2592, 1944) if value == 0 else (1296, 972)
            else:
                self.values[key] = value

    def start_live(self):
        self.record()
        self.is_streaming = True

    def stop_live(self):
        self.record()
        self.is_streaming = False

    def capture_frame(self, **_):
        self.record()
        return np.zeros((120, 160, 3), np.uint8)

    def close(self):
        self.record()
        self.is_open = self.is_streaming = False


def wait_for(predicate):
    until = time.monotonic()+5
    while time.monotonic() < until:
        QTest.qWait(10)
        if predicate():
            return
    raise AssertionError('Timed out waiting for profile worker')


@pytest.fixture
def profile_window(tmp_path, monkeypatch):
    monkeypatch.setenv('XDG_CONFIG_HOME', str(tmp_path))
    app = QApplication.instance() or QApplication([])
    camera = ProfileCamera()
    window = CameraWindow(camera_factory=lambda: camera, camera_name='MindVision', preview_only=True)
    window.show()
    wait_for(lambda: not window._busy)
    window.open_button.click()
    wait_for(lambda: window._is_open and not window._busy)
    yield window, camera
    camera.release.set()
    window.close()
    wait_for(lambda: not window.worker.isRunning())
    app.processEvents()


def save(window, name):
    window._start_profile_operation('profile_save', name)
    wait_for(lambda: not window._busy and window._profile_pending is None)


def test_two_profiles_restore_values_mode_mirrors_and_survive_reopen(profile_window):
    window, camera = profile_window
    window.live_button.click()
    wait_for(lambda: window._is_live and not window._busy)
    save(window, '棋盘 A')
    camera.values = parameters(1)
    camera.values['gamma']['value'] = 125
    camera.values['reverse_y'] = False
    camera.resolution = (1296, 972)
    save(window, '零件 B')
    assert window.profile_combo.count() == 2
    window.profile_combo.setCurrentIndex(window.profile_combo.findText('棋盘 A'))
    assert camera.values['gamma']['value'] == 125  # selecting is not loading
    window.profile_load_button.click()
    wait_for(lambda: not window._busy)
    assert camera.values['gamma']['value'] == 95
    assert camera.values['reverse_y'] is True
    assert camera.resolution == (2592, 1944)
    assert window.float_controls['gamma'].value() == 95
    assert window._is_live and camera.is_streaming
    assert 'loaded' in window.status.text()
    # Existing Save updates only the selected file.
    original_b = window._profile_store().path('零件 B').read_bytes()
    camera.values['gamma']['value'] = 99
    window.profile_save_button.click()
    wait_for(lambda: not window._busy)
    assert window._profile_store().load('棋盘 A')['settings']['gamma'] == 99
    assert window._profile_store().path('零件 B').read_bytes() == original_b
    window.disconnect_button.click()
    wait_for(lambda: not window._is_open and not window._busy)
    assert window.profile_combo.count() == 0
    window.open_button.click()
    wait_for(lambda: window._is_open and not window._busy)
    assert window.profile_combo.count() == 2
    assert threading.get_ident() not in camera.thread_ids


def test_save_waits_for_inflight_and_pending_tunes_then_records_readback(profile_window):
    window, camera = profile_window
    camera.release.clear()
    window._queue_tuning('gamma', 101)
    wait_for(camera.entered.is_set)
    window._queue_tuning('gamma', 115)
    window._start_profile_operation('profile_save', 'Tuned')
    assert window._profile_pending is not None
    assert not window.parameters_group.isEnabled()
    assert not window._profile_store().path('Tuned').exists()
    camera.release.set()
    wait_for(lambda: not window._busy and window._profile_pending is None)
    assert window._profile_store().load('Tuned')['settings']['gamma'] == 115
    assert len(camera.applied) == 2


def test_load_cancels_pending_drag_and_rejects_bad_profile_without_sdk_write(profile_window):
    window, camera = profile_window
    save(window, 'A')
    window._queue_tuning('gamma', 123)
    window.profile_load_button.click()
    wait_for(lambda: not window._busy)
    assert camera.values['gamma']['value'] == 95
    assert len(camera.applied) == 1
    path = window._profile_store().path('A')
    data = json.loads(path.read_text())
    data['settings']['gain'] = 1000
    path.write_text(json.dumps(data))
    window.profile_load_button.click()
    wait_for(lambda: not window._busy)
    assert len(camera.applied) == 1
    assert 'outside' in window.status.text()
    assert window.profile_load_button.isEnabled()


def test_failed_sdk_apply_resumes_live_and_does_not_claim_profile_loaded(profile_window):
    window, camera = profile_window
    save(window, 'A')
    window.live_button.click()
    wait_for(lambda: not window._busy)
    camera.fail_apply = True
    window.profile_load_button.click()
    wait_for(lambda: not window._busy)
    assert 'rejected' in window.status.text()
    assert window._is_live and camera.is_streaming
    assert window.profile_load_button.isEnabled()
    # A failed tuning write must not save an unconfirmed profile.
    window._queue_tuning('gamma', 105)
    window._start_profile_operation('profile_save', 'Bad')
    wait_for(lambda: not window._busy and window._profile_pending is None and not window._tuning)
    assert not window._profile_store().path('Bad').exists()
    assert 'cancelled' in window.status.text()
