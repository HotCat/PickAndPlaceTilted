"""Operator controls map settings to launch specs and preserve extensible fields."""
import os
os.environ.setdefault('QT_QPA_PLATFORM','offscreen')

from PySide6.QtCore import Qt
from PySide6.QtTest import QTest
from PySide6.QtWidgets import QApplication

from launcher.config import ConfigStore
from launcher.ui import LauncherWindow


class FakeManager:
    def __init__(self, directory):
        self.directory = directory
        self.records = {}
        self.launches = []
        self.stops = []

    def status(self,key):
        return self.records.get(key,{'running':False,'state':'ready'})

    def log_tail(self,key):
        return 'Test log\n'

    def launch(self,key,spec):
        self.launches.append((key,spec))
        self.records[key] = {'running':True,'state':'running','pid':42}

    def stop(self,key):
        self.stops.append(key)
        self.records[key] = {'running':True,'state':'stopping','pid':42}


def test_start_restart_and_parameter_persistence(tmp_path):
    app = QApplication.instance() or QApplication([])
    store = ConfigStore(tmp_path/'config.json')
    store.load()
    for tool in store.data['tools']:
        tool.pop('exclusive_module',None)  # Real operator windows must not affect fake lifecycle tests.
    manager = FakeManager(tmp_path)
    window = LauncherWindow(store,manager)
    errors=[]
    window.show_error=lambda error:errors.append(str(error))
    window.show()
    try:
        window.editors['camera_serial'].setCurrentText('camera-123')
        window.start_button.click()
        assert len(manager.launches)==1
        assert 'camera-123' in manager.launches[0][1]['command']
        window.refresh()
        assert not window.start_button.isEnabled()
        window.restart_button.click()
        assert manager.stops==['camera-preview']
        window.refresh()
        assert len(manager.launches)==1  # must wait for normal exit
        manager.records['camera-preview']={'running':False,'state':'exited'}
        window.refresh()
        assert len(manager.launches)==2
        window.tool_list.setCurrentItem(window.rows['fr3-studio'])
        window.editors['robot_ip'].setText('192.168.58.3')
        window.editors['backend'].setCurrentText('simulator')
        window.save_button.click()
        saved=store.data['tools'][1]['values']
        assert saved['robot_ip']=='192.168.58.3'
        assert saved['backend']=='simulator'
        window.tool_list.setCurrentItem(window.rows['camera-preview'])
        assert window.editors['camera_serial'].currentText()=='camera-123'
        assert not errors
    finally:
        window.close()
        app.processEvents()
    assert manager.status('camera-preview')['running']  # closing launcher leaves apps alone


def test_camera_discovery_populates_serial_selector_without_launching_camera(tmp_path):
    import sys
    import time
    app = QApplication.instance() or QApplication([])
    store = ConfigStore(tmp_path/'config.json')
    store.load()
    tool=store.data['tools'][0]
    tool['device_discovery']['command']=[sys.executable,'-c',
        'print(\'CAMERA_DEVICES_JSON:[{{"name":"Test Camera (abc)","sn":"abc"}}]\')']
    manager=FakeManager(tmp_path)
    window=LauncherWindow(store,manager)
    errors=[]
    window.show_error=lambda error:errors.append(str(error))
    try:
        window.find_devices()
        deadline=time.monotonic()+5
        while not window.discovered and time.monotonic()<deadline:
            QTest.qWait(20)
        assert not errors
        selector=window.editors['camera_serial']
        assert selector.findText('abc')>=0
        assert window.discovered['camera-preview'][0]['name']=='Test Camera (abc)'
        assert manager.launches==[]
    finally:
        window.close()
        app.processEvents()
