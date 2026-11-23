"""Exercise the real Qt/process boundary without opening any robot connection."""
import os
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

import time
import json
from dataclasses import asdict
import numpy as np
import pytest
from PySide6.QtCore import QEvent, QSettings, Qt
from PySide6.QtWidgets import QApplication, QFileDialog, QScrollArea
from PySide6.QtTest import QTest
from fr3_cartesian.app import MainWindow
from fr3_cartesian.geometry import Plane


def wait_until(app, condition, seconds=5):
    deadline = time.monotonic()+seconds
    while time.monotonic() < deadline:
        app.processEvents()
        if condition():
            return
        QTest.qWait(10)
    raise AssertionError('Timed out waiting for Qt worker result')


@pytest.mark.parametrize('connected', [False, True])
def test_failure_sends_stop_only_after_connection_was_established(tmp_path, monkeypatch, connected):
    app = QApplication.instance() or QApplication([])
    settings = QSettings(str(tmp_path/'failure.ini'), QSettings.IniFormat)
    window = MainWindow(settings=settings, session_path=tmp_path/'session.json')
    actions = []
    monkeypatch.setattr(window.client, 'shutdown', lambda: actions.append('shutdown'))
    monkeypatch.setattr(window.client, 'stop', lambda: actions.append('stop'))
    try:
        window.connected = connected
        window.failure('Connection refused')
        assert actions == (['shutdown', 'stop'] if connected else ['shutdown'])
        assert not window.connected and not window.tool_applied
        assert not window.arm.isChecked()
    finally:
        window.close()


@pytest.mark.parametrize('language', ['en', 'zh_CN'])
def test_origin_target_explains_outside_start_and_sends_no_command(tmp_path, monkeypatch, language):
    from fr3_cartesian.worker import Session
    app = QApplication.instance() or QApplication([])
    window = MainWindow(settings=QSettings(str(tmp_path/'limits.ini'), QSettings.IniFormat), language=language)
    calls = []
    monkeypatch.setattr(window.client, 'call', lambda *args, **kwargs: calls.append((args, kwargs)))
    try:
        window.planes = [Plane.teach('tilted', [[100, 200, 300], [200, 200, 300], [100, 280, 360]])]
        window.sync_planes()
        window.connected = True
        window.tool_applied = True
        window.state = Session().read()
        window.state['tcp'] = [*window.planes[0].to_base([230, 0, 5]), 180, 0, 0]
        window.state_time = time.monotonic()
        window.orientation = [180, 0, 0]
        window.axes_verified.setChecked(True)
        window.arm.setChecked(True)
        window.refresh()
        assert 'X=230.000' in window.position_limit_status.text()
        assert '[-200.000, 200.000]' in window.position_limit_status.text()
        window.move_button.click()  # The default target is the valid plane origin.
        assert not calls
        assert window.connected and window.tool_applied
        assert not window.arm.isChecked()
        assert 'X=230.000' in window.messages.toPlainText().splitlines()[-1]
    finally:
        window.connected = False
        window.close()


@pytest.mark.parametrize('language', ['en', 'zh_CN'])
def test_plane_apply_explains_each_blocker_and_guards_direct_calls(tmp_path, monkeypatch, language):
    from fr3_cartesian.worker import Session
    app = QApplication.instance() or QApplication([])
    window = MainWindow(settings=QSettings(str(tmp_path/'upload.ini'), QSettings.IniFormat), language=language)
    calls = []
    monkeypatch.setattr(window.client, 'call', lambda *args, **kwargs: calls.append((args, kwargs)))
    try:
        window.planes = [Plane.teach('pick', [[100, 200, 300], [200, 200, 300], [100, 300, 300]])]
        window.sync_planes()
        for blocker, english, chinese in [
            ('offline', 'Connect to the robot', '先连接机器人'),
            ('busy', 'Wait for the current', '等待当前控制器指令'),
            ('plane', 'Select a saved plane', '选择已保存的平面'),
            ('stale', 'Waiting for fresh', '等待最新'),
            ('fault', 'fault 1/2', '故障 1/2'),
            ('unsafe', 'stop, collision', '停止、碰撞或故障'),
            ('moving', 'Release jogging', '松开点动'),
            ('program', 'Lua program', 'Lua 程序'),
            ('tcp', 'Verify or apply the saved TCP', '验证或应用已保存的 TCP'),
            ('review', 'check the replacement acknowledgment', '勾选此按钮上方的覆盖确认项'),
        ]:
            window.connected = True
            window.state = Session().read()
            window.state_time = time.monotonic()
            window.tool_applied = True
            window.plane_combo.setCurrentIndex(0)
            window.plane_replace_confirm.setChecked(True)
            window.client.pending = None
            if blocker == 'offline': window.connected = False
            elif blocker == 'busy': window.client.pending = (1, 'capture', time.monotonic())
            elif blocker == 'plane': window.plane_combo.setCurrentIndex(-1)
            elif blocker == 'stale': window.state_time -= 2
            elif blocker == 'fault': window.state['errors'] = [1, 2]
            elif blocker == 'unsafe': window.state['safe'] = False
            elif blocker == 'moving': window.state['stopped'] = False
            elif blocker == 'program': window.state['program_stopped'] = False
            elif blocker == 'tcp': window.tool_applied = False
            elif blocker == 'review': window.plane_replace_confirm.setChecked(False)
            window.refresh()
            expected = english if language == 'en' else chinese
            assert not window.apply_plane_button.isEnabled(), blocker
            assert expected in window.plane_apply_guidance.text(), blocker
            assert expected in window.apply_plane_button.toolTip(), blocker
            window.apply_plane()  # Programmatic invocation must obey the same gates.
            assert not calls, blocker
        window.plane_replace_confirm.setChecked(True)
        assert window.apply_plane_button.isEnabled()
        window.workpiece_slot.setValue(2)
        assert not window.plane_replace_confirm.isChecked()
        assert not window.apply_plane_button.isEnabled()
        window.plane_replace_confirm.setChecked(True)
        window.apply_plane_button.click()
        assert calls[0][0] == ('apply_plane', dict(plane=asdict(window.planes[0]), slot=2))
        assert not window.arm.isChecked() and window.orientation is None
    finally:
        window.client.pending = None
        window.connected = False
        window.close()


@pytest.mark.parametrize('language', ['en', 'zh_CN'])
def test_guided_simulator_workflow_and_stop(tmp_path, monkeypatch, language):
    app = QApplication.instance() or QApplication([])
    settings = QSettings(str(tmp_path/'preferences.ini'), QSettings.IniFormat)
    window = MainWindow(settings=settings, language=language)
    window.mode.setCurrentIndex(0)
    window.show()
    try:
        window.connect_robot()
        wait_until(app, lambda: window.connected and not window.client.busy)
        window.timer.stop()
        for i in range(4):
            window.demo_tool()
            wait_until(app, lambda: len(window.tool_samples) == i+1 and not window.client.busy)
        window.solve_tool()
        np.testing.assert_allclose(window.tool_result['tool'][:3], [0,0,120], atol=1e-8)
        window.apply_tool()
        wait_until(app, lambda: window.tool_applied and not window.client.busy)
        for i in range(3):
            window.demo_plane()
            wait_until(app, lambda: len(window.plane_points) == i+1 and not window.client.busy)
        window.add_plane()
        assert len(window.planes) == 1
        # A local plane is only uploaded following explicit review of its slot
        # and coordinates. The simulator applies the same reference semantics.
        assert not window.apply_plane_button.isEnabled()
        window.workpiece_slot.setValue(2)
        assert 'Wobj2' in window.plane_upload_preview.text()
        window.plane_replace_confirm.setChecked(True)
        window.apply_plane_button.click()
        wait_until(app, lambda: window.last_plane_application is not None and not window.client.busy)
        assert window.state['user'] == 2 and window.tool_applied
        assert not window.plane_replace_confirm.isChecked()
        assert not window.arm.isChecked() and window.orientation is None
        assert window.last_plane_application['pose'] == pytest.approx(window.planes[0].pose())
        # Explicitly return to base for the app's software-transformed XYZ path.
        window.base_frame_button.click()
        wait_until(app, lambda: window.state['user'] == 0 and not window.client.busy)
        window.tabs.setCurrentIndex(2)
        assert window.current_plane().name == window.planes[0].name
        assert window.lock_button.isEnabled()
        window.lock_button.click()
        wait_until(app, lambda: window.orientation is not None and not window.client.busy)
        window.arm.setChecked(True)
        assert not window.arm.isChecked()
        assert window.orientation is not None
        expected = 'Review the plane directions and travel limits' if language == 'en' else '请检查平面方向与行程限制'
        assert expected in window.messages.toPlainText().splitlines()[-1]
        window.axes_verified.setChecked(True)
        window.state['user'] = 7
        window.arm.setChecked(True)
        assert not window.arm.isChecked()
        assert window.connected and window.tool_applied
        expected = 'Use base workpiece 0' if language == 'en' else '使用基坐标系工件 0'
        assert expected in window.messages.toPlainText().splitlines()[-1]
        assert window.base_frame_button.isEnabled()
        window.base_frame_button.click()
        wait_until(app, lambda: window.state['user'] == 0 and not window.client.busy)
        assert window.connected and window.tool_applied
        assert window.orientation is None and not window.axes_verified.isChecked()
        assert not window.arm.isChecked()
        window.lock_button.click()
        wait_until(app, lambda: window.orientation is not None and not window.client.busy)
        window.axes_verified.setChecked(True)
        window.arm.setChecked(True)
        assert window.arm.isChecked()
        # A worker-side validation refusal must disarm without discarding this
        # calibrated session, and must cancel any pending hold repetition.
        window.client.call('move', dict(plane=asdict(window.current_plane()), target=[999, 0, 0],
                                       orientation=window.orientation, bounds=window.limits(), speed=3))
        window.repeat_direction = (0, 1)
        wait_until(app, lambda: not window.client.busy)
        assert window.connected and window.tool_applied
        assert window.client.process is not None
        assert not window.arm.isChecked() and window.repeat_direction is None
        expected = 'Connection remains open' if language == 'en' else '连接保持打开'
        assert expected in window.messages.toPlainText().splitlines()[-1]
        window.arm.setChecked(True)
        assert window.arm.isChecked()
        before = window.state['tcp'][:3]
        window.send_step(2, 1)
        wait_until(app, lambda: not window.client.busy)
        assert np.linalg.norm(np.array(window.state['tcp'][:3])-before) > .9
        assert window.state['tcp'][3:] == [180., 0., 0.]
        if os.environ.get('FR3_TEST_SCREENSHOT'):
            from pathlib import Path
            screenshot = Path(os.environ['FR3_TEST_SCREENSHOT'])
            window.grab().save(str(screenshot.with_stem(screenshot.stem+'-'+language)))

        # Translation is presentation-only, even with Cartesian controls armed.
        other = 'zh_CN' if language == 'en' else 'en'
        pose_before = list(window.state['tcp'])
        orientation_before = list(window.orientation)
        plane_before = window.current_plane()
        target_before = window.targets[0].value()
        window.plane_name.setText('Station A / 取料工位')
        window.mark_plane_name_edited(window.plane_name.text())
        for code in (other, language):
            with monkeypatch.context() as context:
                def unexpected_command(*args, **kwargs):
                    raise AssertionError('Language switching must never issue robot commands')
                context.setattr(window.client, 'call', unexpected_command)
                window.language_combo.setCurrentIndex(window.language_combo.findData(code))
            assert window.arm.isChecked() and window.tool_applied
            assert window.axes_verified.isChecked()
            assert window.current_plane() is plane_before
            assert window.orientation == orientation_before
            assert window.state['tcp'] == pose_before
            assert window.targets[0].value() == target_before
            assert window.plane_name.text() == 'Station A / 取料工位'
            assert settings.value('ui/language') == code
            assert window.stop_button.text() == ('停止  ·  Esc' if code == 'zh_CN' else 'STOP  ·  Esc')
            assert window.messages.toPlainText().endswith('直线运动已完成' if code == 'zh_CN' else 'Linear move completed')

        # Holding repeats only while the button remains held; release cannot queue another move.
        window.disarm()
        window.hold.setChecked(True)
        window.arm.setChecked(True)
        window.arrow_pressed(2, 1)
        window.arrow_released()
        wait_until(app, lambda: not window.client.busy)
        QTest.qWait(150)
        app.processEvents()
        assert window.repeat_direction is None

        # Save/load recalculates calibration and never restores an armed state.
        window.disarm()
        path = str(tmp_path / 'calibration.json')
        monkeypatch.setattr('fr3_cartesian.app.QFileDialog.getSaveFileName', lambda *a, **kw: (path, ''))
        window.save()
        saved = json.loads((tmp_path/'calibration.json').read_text())
        assert 'language' not in saved
        assert saved['source'] == 'simulation' and saved['schema'] == 3
        assert saved['tool']['tool'][2] == pytest.approx(120)
        window.clear_tool_samples()
        monkeypatch.setattr('fr3_cartesian.app.QFileDialog.getOpenFileName', lambda *a, **kw: (path, ''))
        window.load()
        assert len(window.planes) == 1
        assert not window.arm.isChecked()
        assert window.current_plane() is window.planes[0]
        # Loading while connected verifies the already-applied TCP by reading it.
        wait_until(app, lambda: window.tool_applied and not window.client.busy)
        window.plane_list.setCurrentRow(0)
        assert window.lock_button.isEnabled()
        window.lock_button.click()
        wait_until(app, lambda: window.orientation is not None and not window.client.busy)
        window.grab().save(str(tmp_path / 'motion-ui.png'))
        window.stop()
        assert not window.connected and window.client.process is None
        restored = MainWindow(settings=settings)
        assert restored.language == language
        assert not restored.arm.isChecked() and not restored.connected
        restored.close()
    finally:
        window.close()
        app.processEvents()


def test_live_poll_keeps_jog_usable_and_backend_can_switch(tmp_path):
    app = QApplication.instance() or QApplication([])
    window = MainWindow(settings=QSettings(str(tmp_path/'ui.ini'), QSettings.IniFormat))
    try:
        window.mode.setCurrentIndex(0)
        window.connect_robot()
        wait_until(app, lambda: window.connected and window.client.pending is None)
        window.timer.stop()
        assert window.mode.count() == 2
        assert window.tabs.count() == 4 and not hasattr(window, 'pendant')
        window.teach_enabled.setChecked(True)
        window.refresh()
        assert all(button.isEnabled() for button in window.joint_buttons)
        assert window.joint_limit.isEnabled()
        assert window.base_linear_limit.isEnabled()
        assert window.base_rotation_limit.isEnabled()
        status = window.status.text()
        for _ in range(3):
            window.poll()
            assert window.client.reading and not window.client.busy
            assert window.status.text() == status
            assert all(button.isEnabled() for button in window.joint_buttons)
            assert window.mode.isEnabled()
            wait_until(app, lambda: window.client.pending is None)
        window.poll()
        before = window.state['joints'][0]
        steady_status = window.status.text()
        QTest.mousePress(window.joint_buttons[1], Qt.LeftButton)
        QTest.qWait(120)
        assert window.active_jog is not None
        assert window.status.text() == steady_status
        assert not window.joint_limit.isEnabled()
        assert not window.base_linear_limit.isEnabled()
        assert not window.base_rotation_limit.isEnabled()
        QTest.mouseRelease(window.joint_buttons[1], Qt.LeftButton)
        assert window.active_jog is None
        assert not window.joint_buttons[1].isDown()
        wait_until(app, lambda: window.client.pending is None)
        assert before < window.state['joints'][0] < before+.5
        assert window.connected
        base_before = window.state['tcp'][0]
        window.base_jog_speed_slider.setValue(5)
        assert window.base_jog_speed.value()==5 and window.jog_speed.value()==3
        base_x_plus = window.base_jog_buttons[1]
        QTest.mousePress(base_x_plus, Qt.LeftButton)
        assert not window.base_jog_speed.isEnabled() and not window.base_jog_speed_slider.isEnabled()
        QTest.qWait(120)
        # Exercise the application-level fallback when scrolling reroutes release.
        QTest.mouseRelease(window.status, Qt.LeftButton)
        assert window.active_jog is None
        wait_until(app, lambda: window.client.pending is None)
        assert base_before < window.state['tcp'][0] < base_before+5
        assert window.base_jog_speed.isEnabled() and window.base_jog_speed_slider.isEnabled()
        window.jog_speed_slider.setValue(6)
        assert window.jog_speed.value() == 6
        assert window.joint_limit.maximum() == 10
        assert '#101820' in window.findChild(QScrollArea).viewport().styleSheet()
        assert '#101820' in window.findChild(QScrollArea).styleSheet()
        # Selecting Real FR3 leaves simulation and makes the real connection
        # editable; it does not connect or move automatically.
        window.mode.setCurrentIndex(1)
        assert not window.connected
        assert window.client.process is None
        assert window.mode.isEnabled() and window.ip.isEnabled() and window.sdk.isEnabled()
        assert window.client.simulation  # No real connection was opened.
        assert window.demo_tool_button.isHidden()
    finally:
        window.close()
        app.processEvents()


def test_first_launch_selects_real_without_connecting_and_remembers_backend(tmp_path):
    app = QApplication.instance() or QApplication([])
    settings = QSettings(str(tmp_path/'backend.ini'), QSettings.IniFormat)
    window = MainWindow(settings=settings)
    try:
        assert window.mode.currentIndex() == 1
        assert not window.connected and window.client.process is None
        window.mode.setCurrentIndex(0)
        assert settings.value('connection/backend') == 'simulation'
    finally:
        window.close()
    restored = MainWindow(settings=settings)
    try:
        assert restored.mode.currentIndex() == 0
        assert not restored.connected
    finally:
        restored.close()


def test_focus_change_keeps_idle_session_and_stops_held_jog_without_disconnect(tmp_path):
    app = QApplication.instance() or QApplication([])
    window = MainWindow(settings=QSettings(str(tmp_path/'focus.ini'), QSettings.IniFormat))
    window.mode.setCurrentIndex(0)
    window.connect_robot()
    wait_until(app, lambda: window.connected and window.client.pending is None)
    window.timer.stop()
    window.teach_enabled.setChecked(True)
    try:
        app.sendEvent(window, QEvent(QEvent.WindowDeactivate))
        assert window.connected and window.teach_enabled.isChecked()

        button = window.joint_buttons[1]
        QTest.mousePress(button, Qt.LeftButton)
        QTest.qWait(80)
        assert window.active_jog is not None
        app.sendEvent(window, QEvent(QEvent.WindowDeactivate))
        assert window.active_jog is None
        wait_until(app, lambda: window.client.pending is None)
        assert window.connected and window.state['stopped']
    finally:
        window.close()
        app.processEvents()


@pytest.mark.parametrize('field,value,expected', [
    ('enabled', False, 'Enable the robot'),
    ('mode', 0, 'manual mode'),
    ('stopped', False, 'stationary'),
    ('program_stopped', False, 'Lua program'),
    ('safe', False, 'stop, collision'),
])
def test_jog_explains_controller_blockers(tmp_path, field, value, expected):
    app = QApplication.instance() or QApplication([])
    from fr3_cartesian.worker import Session
    window = MainWindow(settings=QSettings(str(tmp_path/'ui.ini'), QSettings.IniFormat), language='en')
    try:
        window.connected = True
        window.state = Session().read()
        window.state_time = time.monotonic()
        window.teach_enabled.setChecked(True)
        window.state[field] = value
        window.refresh()
        assert expected in window.jog_status.text()
        assert not any(button.isEnabled() for button in window.joint_buttons)
    finally:
        window.connected = False
        window.close()


def test_base_jog_uses_its_own_speed_and_matching_release_stop(tmp_path,monkeypatch):
    from fr3_cartesian.worker import Session
    app=QApplication.instance() or QApplication([])
    window=MainWindow(settings=QSettings(str(tmp_path/'speed.ini'),QSettings.IniFormat))
    window.timer.stop()
    calls=[]
    monkeypatch.setattr(window.client,'call',lambda method,args=None:calls.append((method,args)) or True)
    try:
        window.connected=True
        window.state=Session().read();window.state_time=time.monotonic()
        window.teach_enabled.setChecked(True)
        window.jog_speed.setValue(2);window.base_jog_speed_slider.setValue(7)
        for axis in (1,4):
            window.begin_base_jog(axis,1,window.base_jog_buttons[(axis-1)*2+1])
            assert calls[-1]==('start_jog',dict(ref=2,axis=axis,direction=1,
                distance=5 if axis==1 else 1,speed=7))
            window.end_jog(3)
            assert calls[-1]==('stop_jog',dict(stop_ref=3))
        window.begin_jog(0,1,1,.5,window.joint_buttons[1])
        assert calls[-1]==('start_jog',dict(ref=0,axis=1,direction=1,distance=.5,speed=2))
        window.end_jog(1)
        assert calls[-1]==('stop_jog',dict(stop_ref=1))
    finally:
        window.connected=False;window.close();app.processEvents()


@pytest.mark.parametrize('hold', [False, True])
@pytest.mark.parametrize('release', ['button', 'redirected', 'focus', 'disarm'])
def test_plane_arrow_release_interrupts_current_step_and_keeps_connection(tmp_path, hold, release):
    from fr3_cartesian.geometry import calibrate_pivot
    from fr3_cartesian.worker import Session
    app = QApplication.instance() or QApplication([])
    window = MainWindow(settings=QSettings(str(tmp_path/'plane-release.ini'), QSettings.IniFormat))
    window.mode.setCurrentIndex(0)
    window.show()
    try:
        # Prepare a valid synthetic tilted plane and tool, then exercise actual
        # Qt mouse events, pipe cancellation and the simulator worker process.
        sim = Session()
        window.tool_samples = [sim.demo_pose('pivot', i)['flange'] for i in range(4)]
        window.tool_result = asdict(calibrate_pivot(window.tool_samples))
        window.source = 'simulation'
        window.planes = [Plane.teach('tilted', [[300, 0, 200], [400, 20, 200], [280, 100, 210]])]
        window.sync_planes()
        window.connect_robot()
        wait_until(app, lambda: window.connected and window.client.pending is None)
        window.timer.stop()
        window.apply_tool()
        wait_until(app, lambda: window.tool_applied and window.client.pending is None)
        window.tabs.setCurrentIndex(2)
        window.capture('orientation')
        wait_until(app, lambda: window.orientation is not None and window.client.pending is None)
        window.step.setValue(10)
        window.speed.setValue(.5)
        window.hold.setChecked(hold)
        window.axes_verified.setChecked(True)
        window.arm.setChecked(True)
        before = np.array(window.state['tcp'][:3])
        button = window.arrows[2]  # +X of the tilted plane
        QTest.mousePress(button, Qt.LeftButton)
        QTest.qWait(160)
        assert window.plane_arrow_held and button.isEnabled()
        assert window.client.busy
        if release == 'button':
            QTest.mouseRelease(button, Qt.LeftButton)
        elif release == 'redirected':
            QTest.mouseRelease(window.status, Qt.LeftButton)
        elif release == 'focus':
            app.sendEvent(window, QEvent(QEvent.WindowDeactivate))
        else:
            window.arm.setChecked(False)
        assert not window.plane_arrow_held and not button.isDown()
        assert window.repeat_direction is None and not window.plane_repeat_timer.isActive()
        wait_until(app, lambda: window.client.pending is None)
        stopped = np.array(window.state['tcp'][:3])
        assert 0 < np.linalg.norm(stopped-before) < (1 if hold else 10)
        assert window.state['stopped'] and window.connected and window.tool_applied
        assert window.state['tcp'][3:] == pytest.approx(window.orientation)
        delta = window.current_plane().from_base(stopped)-np.array(window.current_plane().from_base(before))
        assert delta[0] > 0 and delta[1:] == pytest.approx([0, 0], abs=1e-8)
        QTest.qWait(150)
        app.processEvents()
        assert window.client.pending is None  # No delayed repeat after release.
        np.testing.assert_allclose(window.state['tcp'][:3], stopped)
        # Another stroke needs no reconnect or TCP reapplication.
        window.arm.setChecked(True)
        QTest.mousePress(button, Qt.LeftButton)
        QTest.qWait(100)
        QTest.mouseRelease(button, Qt.LeftButton)
        wait_until(app, lambda: window.client.pending is None)
        assert window.connected and window.tool_applied
        assert np.linalg.norm(np.array(window.state['tcp'][:3])-stopped) > 0
    finally:
        window.close()
        app.processEvents()


def test_static_pick_place_recipe_runs_serially_in_simulator_and_persists(tmp_path):
    from fr3_cartesian.geometry import calibrate_pivot
    from fr3_cartesian.persistence import read_calibration
    from fr3_cartesian.worker import Session
    app = QApplication.instance() or QApplication([])
    settings = QSettings(str(tmp_path/'static-task.ini'), QSettings.IniFormat)
    session_path = tmp_path/'calibration-session.json'
    window = MainWindow(settings=settings, session_path=session_path)
    window.mode.setCurrentIndex(0)
    window.show()
    try:
        simulator = Session()
        window.tool_samples = [simulator.demo_pose('pivot', i)['flange'] for i in range(4)]
        window.tool_result = asdict(calibrate_pivot(window.tool_samples))
        window.source = 'simulation'
        window.planes = [Plane.teach('my plane', [[300, 0, 200], [400, 20, 200], [280, 100, 210]])]
        window.sync_planes()
        window.connect_robot()
        wait_until(app, lambda: window.connected and not window.client.busy)
        window.timer.stop()
        window.apply_tool()
        wait_until(app, lambda: window.tool_applied and not window.client.busy)
        for index in range(3):
            window.demo_plane()
            wait_until(app, lambda: len(window.plane_points) == index+1 and not window.client.busy)
        window.add_plane()
        # Return to base and create an active tool orientation before arming.
        window.base_frame_button.click()
        wait_until(app, lambda: window.state['user'] == 0 and not window.client.busy)
        window.tabs.setCurrentIndex(2)
        window.lock_button.click()
        wait_until(app, lambda: window.orientation is not None and not window.client.busy)
        window.axes_verified.setChecked(True)
        window.arm.setChecked(True)
        window.tabs.setCurrentIndex(3)
        window.task_rows.setValue(1)
        window.task_columns.setValue(2)
        for row, point in enumerate(([0, 5, 0], [5, 5, 0])):
            for column, value in enumerate(point, start=2):
                window.task_points.item(row, column).setText(str(value))
        for widget, value in zip(window.task_destination, [320, 30, 210, *window.orientation]):
            widget.setValue(value)
        assert window.task_waypoints.rowCount() == 0
        assert window.task_block_reason() is not None  # Operator review is required.
        window.task_ack.setChecked(True)
        assert window.task_block_reason() is None
        assert window.task_start.isEnabled()
        assert 'Pick 1' in window.task_review.toPlainText()
        assert 'Place 2' in window.task_review.toPlainText()
        window.task_start.click()
        assert window.task_running
        wait_until(app, lambda: not window.task_running and not window.client.busy, seconds=8)
        assert window.connected and window.tool_applied and window.arm.isChecked()
        assert window.state['tcp'] == pytest.approx([window.task_destination[i].value() for i in range(6)], abs=.02)
        assert 'Pick/place sequence completed' in window.messages.toPlainText()
        window.schedule_autosave()
        window.autosave()
        saved = read_calibration(session_path)
        assert saved['task']['rows'] == 1 and saved['task']['columns'] == 2
        assert saved['task']['points'] == [[0, 5, 0], [5, 5, 0]]
        assert saved['task']['destination'] == [320, 30, 210, *window.orientation]
        assert 'task_ack' not in json.loads(session_path.read_text())  # execution permission is never persisted
    finally:
        window.close()
        app.processEvents()


def test_capture_live_tcp_teaches_selected_array_row_in_plane_coordinates(tmp_path):
    from fr3_cartesian.worker import Session
    app = QApplication.instance() or QApplication([])
    window = MainWindow(settings=QSettings(str(tmp_path/'capture-task.ini'), QSettings.IniFormat))
    plane = Plane.teach('pick plane', [[100, 200, 300], [200, 200, 300], [100, 300, 310]])
    try:
        window.planes = [plane]
        window.sync_planes()
        window.connected = True
        window.tool_applied = True
        window.state = Session().read()
        window.state.update(tcp=[*plane.to_base([42.5, 27.25, 6.0]), 180, 0, 0], user=0,
                             safe=True, errors=[], stopped=True, program_stopped=True)
        window.state_time = time.monotonic()
        window.task_points.setCurrentCell(2, 2)
        assert window.task_capture_block_reason() is None
        window.refresh()
        assert window.task_capture_button.isEnabled()
        window.capture_task_point()
        captured = [float(window.task_points.item(2, column).text()) for column in (2, 3, 4)]
        assert captured == pytest.approx([42.5, 27.25, 6.0])
        assert 'Captured current TCP into array row 3' in window.messages.toPlainText()
    finally:
        window.connected = False
        window.close()
        app.processEvents()


def test_capture_live_tcp_teaches_fixed_destination_in_robot_base(tmp_path):
    from fr3_cartesian.worker import Session
    app = QApplication.instance() or QApplication([])
    window = MainWindow(settings=QSettings(str(tmp_path/'capture-destination.ini'), QSettings.IniFormat))
    pose = [450.25, -123.5, 678.75, 179.5, -12.25, 88.0]
    try:
        window.connected = True
        window.tool_applied = True
        window.state = Session().read()
        window.state.update(tcp=pose, user=0, safe=True, errors=[], stopped=True, program_stopped=True)
        window.state_time = time.monotonic()
        window.task_points.clearSelection()  # Destination teaching does not depend on an array row.
        assert window.task_capture_block_reason(require_array_row=False, require_calibrated_plane=False) is None
        window.refresh()
        assert window.task_destination_capture.isEnabled()
        window.task_destination_capture.click()
        assert [widget.value() for widget in window.task_destination] == pytest.approx(pose)
        assert 'Captured current TCP as fixed base destination' in window.messages.toPlainText()
    finally:
        window.connected = False
        window.close()
        app.processEvents()


def test_calibration_file_dialogs_use_dark_theme(tmp_path):
    app = QApplication.instance() or QApplication([])
    window = MainWindow(settings=QSettings(str(tmp_path/'file-dialog-theme.ini'), QSettings.IniFormat))
    try:
        for mode, initial in ((QFileDialog.AcceptSave, tmp_path/'calibration.json'),
                              (QFileDialog.AcceptOpen, tmp_path)):
            dialog = window.calibration_file_dialog(mode, str(initial))
            assert dialog.testOption(QFileDialog.DontUseNativeDialog)
            assert '#101820' in dialog.styleSheet()
            assert '#182732' in dialog.styleSheet()
            assert dialog.nameFilters() == ['JSON (*.json)']
            dialog.close()
    finally:
        window.close()
        app.processEvents()


@pytest.mark.parametrize('language', ['en', 'zh_CN'])
def test_capture_independent_transit_routes_switch_resize_and_restore(tmp_path, language):
    from fr3_cartesian.worker import Session
    from fr3_cartesian.persistence import read_calibration
    app = QApplication.instance() or QApplication([])
    path = tmp_path/'route-session.json'
    window = MainWindow(settings=QSettings(str(tmp_path/'route-ui.ini'), QSettings.IniFormat),
                        session_path=path, language=language)
    pose_a = [301.123, 32.456, 220.789, 180, 0, 0]
    pose_b = [350, 40, 230, 180, 0, 0]
    try:
        window.connected = True
        window.tool_applied = True
        window.state = Session().read()
        window.state.update(tcp=pose_a, user=0)
        window.state_time = time.monotonic()
        window.refresh()
        assert window.task_add_waypoint.isEnabled()
        window.task_add_waypoint.click()
        assert window.task_waypoint_values() == [pose_a]
        assert window.task_data()['routes'][0] == [pose_a]
        assert window.task_data()['routes'][1] == []
        window.task_points.setCurrentCell(1, 2)
        assert window.task_waypoints.rowCount() == 0
        window.state['tcp'] = pose_b
        window.task_add_waypoint.click()
        window.task_waypoints.item(0, 0).setText('351.125')
        expected_b = [351.125, *pose_b[1:]]
        window.task_points.setCurrentCell(0, 2)
        assert window.task_waypoint_values() == [pose_a]
        window.task_columns.setValue(3)
        assert window.task_data()['routes'][:3] == [[pose_a], [expected_b], []]
        window.task_points.setCurrentCell(1, 2)
        assert window.task_waypoint_values() == [expected_b]
        # An uncompleted edit is retained across selection, and cannot be saved
        # or executed until it becomes a finite six-axis pose again.
        window.task_waypoints.item(0, 0).setText('-')
        window.task_points.setCurrentCell(0, 2)
        window.task_points.setCurrentCell(1, 2)
        assert window.task_waypoints.item(0, 0).text() == '-'
        window.task_waypoints.item(0, 0).setText(str(expected_b[0]))
        window.schedule_autosave()
        window.autosave()
        saved = read_calibration(path)
        assert saved['task']['routes'][:3] == [[pose_a], [expected_b], []]
        window.restore_calibration(saved)
        window.task_points.setCurrentCell(1, 2)
        assert window.task_waypoint_values() == [expected_b]
        window.task_points.setCurrentCell(0, 2)
        assert window.task_waypoint_values() == [pose_a]
    finally:
        window.connected = False
        window.close()
        app.processEvents()


@pytest.mark.parametrize('state_change', [dict(stopped=False), dict(program_stopped=False),
                                          dict(user=1), dict(safe=False), dict(errors=[1, 2])])
def test_waypoint_capture_rejects_unsafe_or_nonbase_feedback(tmp_path, state_change):
    from fr3_cartesian.worker import Session
    app = QApplication.instance() or QApplication([])
    window = MainWindow(settings=QSettings(str(tmp_path/'blocked-route.ini'), QSettings.IniFormat))
    try:
        window.connected = True
        window.tool_applied = True
        window.state = Session().read()
        window.state.update(state_change)
        window.state_time = time.monotonic()
        window.add_task_waypoint()
        assert window.task_waypoints.rowCount() == 0
        assert 'Waypoint capture blocked' in window.messages.toPlainText()
    finally:
        window.connected = False
        window.close()
        app.processEvents()
