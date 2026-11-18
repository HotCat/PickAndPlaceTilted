"""Calibration recovery across restarts, without robot communication."""
import copy
import json
import os
import time
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from dataclasses import asdict
import numpy as np
import pytest
from PySide6.QtCore import QSettings, QSignalBlocker
from PySide6.QtTest import QTest
from PySide6.QtWidgets import QApplication

from fr3_cartesian.app import MainWindow
from fr3_cartesian.geometry import Plane, calibrate_pivot
from fr3_cartesian.persistence import decode_calibration, read_calibration, write_calibration
from fr3_cartesian.worker import Session


@pytest.fixture
def document():
    session = Session()
    contacts = [session.demo_pose('pivot', i)['flange'] for i in range(4)]
    tool = asdict(calibrate_pivot(contacts))
    points = [[300, 0, 200], [400, 0, 200], [300, 100, 200]]
    return dict(schema=3, source='robot', samples=contacts, samples_reference='robot_base', tolerance=.5, tool=tool,
                slot=2, tool_id=1, load=3, workpiece_slot=4,
                planes=[asdict(Plane.teach(name, points)) for name in ('Pick plane', 'Place / 放料')],
                bounds=[[-100, 150], [-50, 100], [-5, 120]], selected_plane='Place / 放料',
                plane_draft=dict(name='Next / 下一个', points=points[:2]),
                xyz=dict(step=2.5, speed=4, hold=True, targets=[10, 20, 30],
                         orientation_reference=[180, 0, 25]), active_tab=2)


def test_legacy_file_recomputes_calibration_and_supplies_xyz_defaults(document):
    document['schema'] = 1
    for key in ('selected_plane', 'plane_draft', 'xyz', 'active_tab'):
        del document[key]
    document['tool']['tool'][2] = 999  # Never trust stored fitted values.
    restored = decode_calibration(document)
    assert restored['tool']['tool'][2] == pytest.approx(120)
    assert restored['selected_plane'] == 'Pick plane'
    assert restored['xyz']['step'] == 1
    assert restored['xyz']['orientation_reference'] is None


def test_legacy_robot_calibration_is_preserved_but_never_silently_relabelled_as_base(document):
    document['schema'] = 2
    document.pop('samples_reference')
    for plane in document['planes']:
        plane.pop('reference')
    decoded = decode_calibration(document)
    assert decoded['samples_reference'] == 'legacy_workpiece_unknown'
    assert decoded['plane_draft']['reference'] == 'legacy_workpiece_unknown'
    assert all(p.reference == 'legacy_workpiece_unknown' for p in decoded['planes'])
    assert decoded['planes'][0].points == document['planes'][0]['points']
    np.testing.assert_allclose(decoded['tool']['tool'], document['tool']['tool'], atol=1e-9)


@pytest.mark.parametrize('language', ['en', 'zh_CN'])
def test_legacy_plane_is_blocked_until_reteach_and_frame_provenance_survives_save(tmp_path, monkeypatch, document, language):
    app = QApplication.instance() or QApplication([])
    document['schema'] = 2
    document.pop('samples_reference')
    for plane in document['planes']:
        plane.pop('reference')
    settings = QSettings(str(tmp_path/'legacy.ini'), QSettings.IniFormat)
    window = MainWindow(settings=settings, language=language)
    calls = []
    monkeypatch.setattr(window.client, 'call', lambda *args, **kwargs: calls.append((args, kwargs)))
    try:
        window.restore_calibration(decode_calibration(document))
        window.connected = True
        window.tool_applied = True
        window.client.simulation = False
        window.state = Session().read()
        window.state_time = time.monotonic()
        window.plane_replace_confirm.setChecked(True)
        window.refresh()
        assert not window.apply_plane_button.isEnabled()
        assert not window.plane_replace_confirm.isEnabled()
        assert window.current_plane().name in window.plane_apply_guidance.text()
        assert ('Reteach' if language == 'en' else '重新示教') in window.apply_plane_button.text()
        assert ('Remove plane' if language == 'en' else '删除平面') in window.plane_apply_guidance.text()
        assert ('Clear points' if language == 'en' else '清空点位') in window.plane_apply_guidance.text()
        assert not window.lock_button.isEnabled()
        window.apply_plane()
        window.capture('tool')
        window.capture('plane1')
        assert not calls
        window.axes_verified.setChecked(True)
        window.orientation = [180, 0, 0]
        window.arm.setChecked(True)
        assert not window.arm.isChecked()
        window.schedule_autosave()
        window.autosave()
        saved = json.loads(window.session_path.read_text())
        assert saved['schema'] == 3
        assert saved['planes'][0]['reference'] == 'legacy_workpiece_unknown'
        assert saved['samples_reference'] == 'legacy_workpiece_unknown'
        assert read_calibration(window.session_path)['planes'][0].reference == 'legacy_workpiece_unknown'
        # Follow the displayed recovery steps without recalibrating the TCP.
        old_tool = copy.deepcopy(window.tool_result)
        old_name = window.current_plane().name
        window.remove_plane()
        window.clear_plane_points()
        state = dict(Session().read(), tool=window.slot.value(), tcp_verified=True,
                     tool_offset=old_tool['tool'])
        for i, point in enumerate([[300, 0, 200], [400, 0, 200], [300, 100, 200]]):
            window.capture(f'plane{i}')
            assert calls[-1][0] == ('capture',)
            window.completed('capture', dict(state, tcp=[*point, 180, 0, 0]))
        window.plane_name.setText(old_name)
        window.add_plane()
        assert window.current_plane().name == old_name
        assert window.current_plane().reference == 'robot_base'
        assert window.tool_applied and window.tool_result == old_tool
        assert window.plane_replace_confirm.isEnabled()
        assert not window.apply_plane_button.isEnabled()  # New points need a new review.
        window.plane_replace_confirm.setChecked(True)
        assert window.apply_plane_button.isEnabled()
        assert window.apply_plane_button.text() == window.tr('Apply plane to controller')
        window.apply_plane_button.click()
        assert [args[0] for args, _ in calls] == ['capture']*3 + ['apply_plane']
        assert calls[-1][0][1]['plane']['reference'] == 'robot_base'
    finally:
        window.connected = False
        window.close()


@pytest.mark.parametrize('key,value', [
    ('bounds', [[-1, 1], [-1, 1], [float('nan'), 10]]),
    ('selected_plane', 'missing'),
    ('slot', 99),
    ('workpiece_slot', 0),
    ('workpiece_slot', 15),
    ('xyz', {'speed':100}),
    ('plane_draft', {'points': [[0, 0, 0]]*4}),
])
def test_invalid_saved_data_is_rejected(document, key, value):
    document[key] = value
    with pytest.raises(ValueError):
        decode_calibration(document)


def test_failed_serialization_preserves_previous_file(tmp_path, document):
    path = tmp_path/'session.json'
    write_calibration(path, document)
    before = path.read_bytes()
    broken = copy.deepcopy(document)
    broken['xyz']['speed'] = float('nan')
    with pytest.raises(ValueError):
        write_calibration(path, broken)
    assert path.read_bytes() == before


@pytest.mark.parametrize('language', ['en', 'zh_CN'])
def test_autosave_restart_keeps_planes_draft_and_xyz_without_robot_commands(tmp_path, monkeypatch, document, language):
    app = QApplication.instance() or QApplication([])
    def unexpected_command(*args, **kwargs):
        raise AssertionError('Restoring calibration must never send robot commands')
    monkeypatch.setattr('fr3_cartesian.client.RobotClient.call', unexpected_command)
    settings = QSettings(str(tmp_path/'preferences.ini'), QSettings.IniFormat)
    window = MainWindow(settings=settings, language=language)
    try:
        window.restore_calibration(decode_calibration(document))
        # Real operator edits trigger autosave without an explicit Save or close.
        window.targets[0].setValue(12.34)
        window.plane_name.setText('Draft / 待标定')
        QTest.qWait(450)
        app.processEvents()
        assert window.session_path.exists()
        saved = read_calibration(window.session_path)
        assert saved['xyz']['targets'][0] == 12.34
        assert len(saved['plane_draft']['points']) == 2
        assert saved['plane_draft']['name'] == 'Draft / 待标定'
        # Close immediately after editing: the final edit must also be saved.
        window.speed.setValue(6)
        window.tool_applied = True
        with QSignalBlocker(window.arm):
            window.arm.setChecked(True)
    finally:
        window.close()

    restored = MainWindow(settings=settings, language=language)
    try:
        assert [p.name for p in restored.planes] == ['Pick plane', 'Place / 放料']
        assert restored.current_plane().name == 'Place / 放料'
        assert restored.plane_list.currentRow() == 1
        assert restored.plane_points == document['plane_draft']['points']
        assert restored.plane_name.text() == 'Draft / 待标定'
        assert restored.limits() == document['bounds']
        assert restored.workpiece_slot.value() == 4
        assert restored.last_plane_application is None
        assert not restored.plane_replace_confirm.isChecked()
        assert restored.step.value() == 2.5 and restored.speed.value() == 6
        assert restored.hold.isChecked()
        assert [w.value() for w in restored.targets] == [12.34, 20, 30]
        assert restored.tabs.currentIndex() == 2
        assert restored.saved_orientation == [180, 0, 25]
        assert restored.orientation is None
        assert ('Saved orientation' if language == 'en' else '已保存姿态') in restored.orientation_label.text()
        assert restored.slot.value() == 2 and restored.load_number.value() == 3
        assert not restored.tool_applied and not restored.axes_verified.isChecked()
        assert not restored.arm.isChecked() and not restored.teach_enabled.isChecked()
        assert not restored.connected and restored.client.process is None
        # Saving again must retain the reference, without making it a live lock.
        restored.step.setValue(3)
        restored.autosave()
        assert read_calibration(restored.session_path)['xyz']['orientation_reference'] == [180, 0, 25]
    finally:
        restored.close()


def test_corrupt_autosave_is_reported_and_not_overwritten_on_close(tmp_path):
    app = QApplication.instance() or QApplication([])
    path = tmp_path/'calibration-session.json'
    path.write_text('{broken file', encoding='utf-8')
    window = MainWindow(settings=QSettings(str(tmp_path/'preferences.ini'), QSettings.IniFormat), language='en')
    try:
        assert not window.planes and not window.connected
        assert 'Cannot restore saved calibration' in window.messages.toPlainText()
    finally:
        window.close()
    assert path.read_text() == '{broken file'


def test_partial_tcp_contacts_can_be_recovered(tmp_path, document):
    document['samples'] = document['samples'][:2]
    document['tool'] = None
    document['planes'] = []
    document['selected_plane'] = None
    document['plane_draft']['points'] = []
    path = tmp_path/'partial.json'
    write_calibration(path, document)
    recovered = read_calibration(path)
    assert len(recovered['samples']) == 2
    assert recovered['tool'] is None


def test_loaded_tcp_can_be_applied_from_xyz_controls_and_external_change_revokes_it(tmp_path, document):
    app = QApplication.instance() or QApplication([])
    document['source'] = 'simulation'
    path = tmp_path/'calibration-session.json'
    write_calibration(path, document)
    settings = QSettings(str(tmp_path/'preferences.ini'), QSettings.IniFormat)
    window = MainWindow(settings=settings, language='en')
    def wait_for(condition):
        deadline = time.monotonic()+5
        while time.monotonic() < deadline:
            app.processEvents()
            if condition():
                return
            QTest.qWait(10)
        raise AssertionError('Timed out waiting for SDK worker')
    try:
        window.mode.setCurrentIndex(0)
        window.connect_robot()
        wait_for(lambda: window.connected and window.client.pending is None)
        window.timer.stop()
        assert not window.tool_applied and not window.lock_button.isEnabled()
        assert window.apply_saved_tcp_button.isEnabled()
        assert 'Apply saved TCP' in window.orientation_guidance.text()
        window.apply_saved_tcp_button.click()
        wait_for(lambda: window.tool_applied and window.client.pending is None)
        assert window.lock_button.isEnabled()
        window.lock_button.click()
        wait_for(lambda: window.orientation is not None and window.client.pending is None)
        window.axes_verified.setChecked(True)
        window.arm.setChecked(True)
        assert window.arm.isChecked()
        # Represent a controller tool change independently of the saved settings.
        window.client.call('apply_tool', dict(tool=[0, 0, 80, 0, 0, 0], slot=3, tool_id=1, load=0))
        wait_for(lambda: window.client.pending is None)
        assert not window.tool_applied and not window.arm.isChecked()
        assert window.orientation is None and not window.axes_verified.isChecked()
    finally:
        window.close()
        app.processEvents()


@pytest.mark.parametrize('language',['en','zh_CN'])
def test_jog_speeds_restore_with_legacy_joystick_tab_and_no_input_profile_loading(tmp_path,document,language):
    app=QApplication.instance() or QApplication([])
    settings=QSettings(str(tmp_path/'jogs.ini'),QSettings.IniFormat)
    profile=tmp_path/'joystick-profile.json';profile.write_text('{retired profile: do not parse}')
    document['active_tab']=3
    document['jog']=dict(joint_speed=2,tcp_speed=7,joint_limit=1.5,linear_limit=12,rotation_limit=2.5)
    write_calibration(tmp_path/'calibration-session.json',document)
    window=MainWindow(settings=settings,language=language)
    try:
        assert window.tabs.count()==3 and window.tabs.currentIndex()==2
        assert not hasattr(window,'pendant')
        assert window.base_jog_speed.value()==window.base_jog_speed_slider.value()==7
        assert window.jog_speed.value()==2
        assert window.base_linear_limit.value()==12 and window.base_rotation_limit.value()==2.5
        assert window.joint_limit.value()==1.5
        assert window.client.process is None and not window.connected
        assert not window.teach_enabled.isChecked() and not window.arm.isChecked()
        assert 'Cannot restore' not in window.messages.toPlainText()
        window.base_jog_speed_slider.setValue(6)
    finally:
        window.close();app.processEvents()
    assert profile.read_text()=='{retired profile: do not parse}'
    restored=MainWindow(settings=settings,language=language)
    try:
        assert restored.base_jog_speed.value()==restored.base_jog_speed_slider.value()==6
        assert restored.jog_speed.value()==2 and restored.tabs.currentIndex()==2
        assert not restored.teach_enabled.isChecked() and restored.client.process is None
        assert len(restored.planes)==2 and restored.tool_result is not None
    finally:
        restored.close();app.processEvents()
