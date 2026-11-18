"""Retired joystick UI experiment; not imported or instantiated by the application."""
import json
from pathlib import Path
import shutil
import statistics
import time

from PySide6.QtCore import Qt, QTimer, QSignalBlocker, QProcess
from PySide6.QtWidgets import (QApplication, QWidget, QVBoxLayout, QHBoxLayout,
    QFormLayout, QGridLayout, QComboBox, QCheckBox, QDoubleSpinBox, QPushButton)

from .joystick import AXES, LinuxJoystick, StrokeGate, default_profile, devices, normalized, throttle_fraction, validate_profile
from .messages import Message, OperatorValueError, error_message
from .persistence import write_calibration


class ScrollSafeComboBox(QComboBox):
    def wheelEvent(self, event):
        if self.view().isVisible():
            super().wheelEvent(event)
        else:
            event.ignore()


class ScrollSafeDoubleSpinBox(QDoubleSpinBox):
    def wheelEvent(self, event):
        event.ignore()


class PendantPanel(QWidget):
    def __init__(self, window, profile_path):
        super().__init__()
        self.window = window
        self.profile_path = Path(profile_path)
        self.profile = default_profile()
        self.reader = None
        self.gate = StrokeGate()
        self.sent = False
        self.plane_owned = False
        self.plane_remaining = 0.
        self.sampling = None
        self.last_axes, self.last_buttons = {}, set()
        self.last_blocked_input = None
        self.vector_started = False
        self.last_paint_at = 0.
        self.marker = QPushButton(self)  # Identifies joystick-owned jogs in the existing arbiter.
        self.marker.hide()
        self.input_status = Message('Joystick closed. Scan and open a device to monitor inputs without motion.')
        w = window
        box = QVBoxLayout(self)
        box.addWidget(w.paragraph('4  /  JOYSTICK TEACHING PENDANT'))
        box.addWidget(w.paragraph('Base XYZ moves all three axes together with live proportional stick velocity and fixed tool orientation. Joint/rotation modes use one native jog axis at a time. Hold the deadman and deflect; center or release to stop. Recenter after the shared travel cap.'))
        row = QHBoxLayout()
        self.device_combo = ScrollSafeComboBox()
        row.addWidget(self.device_combo, 1)
        self.scan_button = w.button('Scan joysticks', self.scan, row)
        self.open_button = w.button('Open input monitor', self.toggle_open, row)
        box.addLayout(row)
        control = w.group('JOYSTICK CONTROL')
        control.setStyleSheet('QGroupBox { border: 2px solid #89a8b8; }')
        control_box = QVBoxLayout(control)
        self.mode = ScrollSafeComboBox()
        for source in ('Joints J1 / J2 / J3', 'Joints J4 / J5 / J6', 'Base X / Y / Z', 'Base RX / RY / RZ'):
            self.mode.addItem(w.tr(source), source)
        self.mode.addItem(w.tr('Calibrated plane X / Y / Z (bounded steps)'), 'Calibrated plane X / Y / Z (bounded steps)')
        self.mode.currentIndexChanged.connect(self.mode_changed)
        form = QFormLayout()
        form.addRow(w.label('Joystick motion mode'), self.mode)
        control_box.addLayout(form)
        shared = QFormLayout()
        self.linear_limit = ScrollSafeDoubleSpinBox()
        self.linear_limit.setRange(.1, 50.)
        self.linear_limit.setDecimals(1)
        self.linear_limit.setSuffix(' mm')
        self.linear_limit.setValue(w.base_linear_limit.value())
        self.linear_limit.valueChanged.connect(w.base_linear_limit.setValue)
        w.base_linear_limit.valueChanged.connect(self.linear_limit.setValue)
        shared.addRow(w.label('Shared Base XYZ linear travel limit'), self.linear_limit)
        control_box.addLayout(shared)
        self.enable_state = w.paragraph('JOYSTICK DISABLED')
        self.enable_state.setStyleSheet('font-size: 17px; font-weight: 700; color: #ffd089;')
        control_box.addWidget(self.enable_state)
        self.enabled = w.pushbutton('Enable joystick teaching')
        self.enabled.setCheckable(True)
        self.enabled.setMinimumHeight(58)
        self.enabled.setStyleSheet('''
            QPushButton { background: #563d1f; border: 2px solid #e7b36d;
                          color: #fff2d5; font-size: 17px; font-weight: 700; padding: 12px; }
            QPushButton:hover { background: #71502a; }
            QPushButton:checked { background: #194b45; border-color: #62dec5; color: #e5fff8; }
            QPushButton:checked:hover { background: #236157; }
        ''')
        self.enabled.toggled.connect(self.enable_changed)
        control_box.addWidget(self.enabled)
        self.readiness = w.paragraph('Click Enable joystick teaching after choosing the motion mode.')
        control_box.addWidget(self.readiness)
        self.mapping_summary = w.paragraph(Message('Active mapping: {axes} · hold {deadman} · STOP {stop}',
            dict(axes='—', deadman='—', stop='—')))
        control_box.addWidget(self.mapping_summary)
        self.velocity_readout = w.paragraph('TCP velocity: —')
        self.velocity_readout.setMinimumHeight(90)
        control_box.addWidget(self.velocity_readout)
        box.addWidget(control)
        self.identity = w.paragraph('No joystick open.')
        box.addWidget(self.identity)
        self.monitor = w.paragraph('Axes/buttons: —')
        self.monitor.setMinimumHeight(70)
        box.addWidget(self.monitor)
        self.status = w.paragraph('Joystick closed. Scan and open a device to monitor inputs without motion.')
        box.addWidget(self.status)
        box.addWidget(w.paragraph('Opening the monitor never enables robot motion. First release the trigger and center the mapped axes. Focus loss, input loss, mode changes or a delayed input poll disable joystick teaching; release and center before enabling again.'))
        self.config = w.group('JOYSTICK MAPPING AND CENTER FILTER')
        config_box = QVBoxLayout(self.config)
        grid = QGridLayout()
        grid.addWidget(w.label('Target axis in selected mode'), 0, 0)
        grid.addWidget(w.label('Linux input axis'), 0, 1)
        grid.addWidget(w.label('Invert'), 0, 2)
        self.axis_combos, self.inverts = [], []
        for i in range(3):
            grid.addWidget(w.label(f'{i+1}'), i+1, 0)
            combo, invert = ScrollSafeComboBox(), QCheckBox()
            self.axis_combos.append(combo)
            self.inverts.append(invert)
            grid.addWidget(combo, i+1, 1)
            grid.addWidget(invert, i+1, 2)
        config_box.addLayout(grid)
        form = QFormLayout()
        self.button_combos = {}
        for key, label in [('deadman', 'Held deadman button'), ('stop', 'Software STOP button'),
                           ('capture', 'Capture button (trigger released, stick centered)')]:
            combo = ScrollSafeComboBox()
            self.button_combos[key] = combo
            form.addRow(w.label(label), combo)
        self.deadzone = ScrollSafeDoubleSpinBox()
        self.deadzone.setRange(3, 40)
        self.deadzone.setSuffix(' %')
        form.addRow(w.label('Center dead zone'), self.deadzone)
        self.response = ScrollSafeDoubleSpinBox()
        self.response.setRange(1, 3)
        self.response.setSingleStep(.1)
        form.addRow(w.label('Joint/rotation response exponent'), self.response)
        self.max_speed = ScrollSafeDoubleSpinBox()
        self.max_speed.setRange(1, 10)
        self.max_speed.setSuffix(' %')
        form.addRow(w.label('Joint / rotation / plane speed ceiling'), self.max_speed)
        self.tcp_speed = ScrollSafeDoubleSpinBox()
        self.tcp_speed.setRange(.1, 25.)
        self.tcp_speed.setDecimals(1)
        self.tcp_speed.setValue(10.)
        self.tcp_speed.setSuffix(' mm/s')
        form.addRow(w.label('Base XYZ maximum TCP speed'), self.tcp_speed)
        form.addRow(w.paragraph('Base XYZ speed is a ceiling. It reduces automatically when the robot falls behind the target.'))
        self.throttle_combo = ScrollSafeComboBox()
        self.throttle_invert = w.checkbox('Invert throttle (swap slow/fast ends)')
        form.addRow(w.label('Throttle axis for TCP speed limit'), self.throttle_combo)
        form.addRow(self.throttle_invert)
        config_box.addLayout(form)
        row = QHBoxLayout()
        self.center_button = w.button('Measure released center (1 second)', self.measure_center, row)
        self.save_button = w.button('Save joystick profile', self.save_profile, row)
        config_box.addLayout(row)
        box.addWidget(self.config)
        box.addWidget(w.paragraph('Button codes are shown live (trigger is BTN_TRIGGER / 288 on the tested PXN). Capture adds a TCP contact on the Tool tab, or the next O/X/Y point on the Workpiece planes tab. Other tabs do not capture. The measured PXN profile maps twist ABS_RY to target axis 3; confirm directions in the monitor. Generic profiles leave input 3 disabled until identified.'))
        box.addWidget(w.paragraph('Base XYZ speed is in mm/s; the percentage Jog speed applies to native joint/rotation modes. The throttle scales TCP speed from zero to its configured maximum. Diagonal speed is capped as a vector. Linear travel limit caps total path length per stroke. This USB input is not a safety-rated enabling device.'))
        box.addWidget(w.paragraph('Plane mode requires Enable Cartesian motion in XYZ controls. It uses verified steps of at most 1 mm, preserves locked orientation and plane limits, and caps total travel per stroke at Linear travel limit. Releasing finishes the current step; focus/input loss stops and disconnects an in-flight plane move.'))
        w.button('Open standard jstest-gtk tool', self.open_standard_tool, box)
        box.addWidget(w.paragraph('jstest-gtk tests/calibrates Linux js devices. This app reads raw evdev input and uses the mapping and dead zone saved here; js calibration does not replace this profile.'))
        box.addStretch()
        try:
            if self.profile_path.exists():
                self.profile = validate_profile(json.loads(self.profile_path.read_text()))
        except (OSError, ValueError, TypeError, KeyError) as exc:
            self.input_status = Message('Cannot load joystick profile: {detail}', dict(detail=error_message(exc)))
        with QSignalBlocker(self.mode):
            self.mode.setCurrentIndex(self.profile['motion_mode'])
        self.populate_mapping()
        self.scan()
        self.timer = QTimer(self)
        self.timer.setInterval(20)
        self.timer.setTimerType(Qt.PreciseTimer)
        self.timer.timeout.connect(self.poll_input)
        self.timer.start()

    @property
    def owns_jog(self):
        return bool(self.window.active_jog and self.window.active_jog[1] is self.marker)

    def populate_mapping(self):
        axes = self.reader.ranges if self.reader else AXES
        buttons = self.reader.buttons if self.reader else range(288, 300)
        for i, combo in enumerate(self.axis_combos):
            combo.clear()
            combo.addItem(self.window.tr('Disabled'), None)
            for code in axes:
                combo.addItem(f'{AXES.get(code, "ABS_"+str(code))} ({code})', code)
            combo.setCurrentIndex(max(0, combo.findData(self.profile['axes'][i])))
            self.inverts[i].setChecked(self.profile['invert'][i])
        for key, combo in self.button_combos.items():
            combo.clear()
            combo.addItem(self.window.tr('Disabled'), None)
            for code in buttons:
                combo.addItem(f'{code} / button {code-287}', code)
            combo.setCurrentIndex(max(0, combo.findData(self.profile[key])))
        self.throttle_combo.clear()
        self.throttle_combo.addItem(self.window.tr('Disabled'), None)
        for code in axes:
            self.throttle_combo.addItem(f'{AXES.get(code, "ABS_"+str(code))} ({code})', code)
        self.throttle_combo.setCurrentIndex(max(0, self.throttle_combo.findData(self.profile['throttle_axis'])))
        self.throttle_invert.setChecked(self.profile['throttle_invert'])
        self.tcp_speed.setValue(self.profile['tcp_speed'])
        self.deadzone.setValue(self.profile['deadzone']*100)
        self.response.setValue(self.profile['response'])
        self.max_speed.setValue(self.profile['max_speed'])

    def scan(self):
        if self.reader:
            return
        self.device_combo.clear()
        for device in devices():
            self.device_combo.addItem(f'{device.name} · {device.vendor}:{device.product} · {device.path}', device)
        for i in range(self.device_combo.count()):
            if self.device_combo.itemData(i).identity == self.profile['device']:
                self.device_combo.setCurrentIndex(i)
                break
        if self.device_combo.count() == 0:
            self.input_status = Message('No Linux joystick found. Reconnect USB and scan again; the sandbox may hide /dev/input.')
        self.render_labels()

    def toggle_open(self):
        self.suspend()
        if self.reader:
            self.close_input()
            return
        device = self.device_combo.currentData()
        if device is None:
            return
        try:
            self.reader = LinuxJoystick(device)
            if device.identity != self.profile['device']:
                self.profile['centers'] = {}
            self.profile['device'] = device.identity
            self.populate_mapping()
            self.gate.reset()
            self.input_status = Message('Input monitor open. Verify mappings and measure released center before teaching.')
        except (OSError, ValueError) as exc:
            self.input_status = Message('Joystick input error: {detail}', dict(detail=error_message(exc)))
        self.render_labels()

    def close_input(self):
        self.suspend()
        if self.reader:
            self.reader.close()
            self.reader = None
        self.sampling = None
        self.input_status = Message('Joystick closed. Scan and open a device to monitor inputs without motion.')
        self.render_labels()

    def suspend(self, *_, stop_plane=True, reason=None):
        was_enabled = hasattr(self, 'enabled') and self.enabled.isChecked()
        if hasattr(self, 'enabled'):
            with QSignalBlocker(self.enabled):
                self.enabled.setChecked(False)
        if self.owns_jog:
            self.window.end_jog(self.window.active_jog[0]+1)
        if self.window.client.stream_token is not None:
            self.window.client.end_velocity()
        self.vector_started = False
        pending = self.window.client.pending
        queued = self.window.client.queued_command
        stop_plane = stop_plane and self.plane_owned and ((pending and pending[1] == 'move') or (queued and queued[0] == 'move'))
        self.plane_owned = False
        self.gate.reset()
        self.sent = False
        if hasattr(self, 'config'):
            self.config.setEnabled(True)
        if was_enabled and reason is not None:
            self.input_status = Message('Joystick teaching suspended: {reason}. Release, center, and enable again.', dict(reason=reason))
            self.window.log(self.input_status)
        if stop_plane:
            self.window.stop(reason=Message('Joystick teaching suspended during a plane move'))

    def mode_changed(self, *_):
        was_enabled = self.enabled.isChecked()
        self.suspend()
        self.input_status = Message('Joystick mode changed to {mode}. Click Enable joystick teaching again, then release all buttons and center the stick.',
                                    dict(mode=Message(self.mode.currentData())))
        if was_enabled:
            self.window.log(self.input_status)
        self.profile['motion_mode'] = self.mode.currentIndex()
        try:
            write_calibration(self.profile_path, self.profile)
        except OSError as exc:
            self.window.log('Cannot save joystick mode: {detail}', detail=error_message(exc))
        self.render_labels()

    def enable_changed(self, checked):
        self.gate.reset()
        self.sent = False
        if checked and not self.reader:
            self.enabled.setChecked(False)
            self.input_status = Message('Open a joystick input monitor first.')
        elif checked:
            try:
                self.collect_profile()
                for code in self.profile['axes']:
                    if code is not None and (code not in self.reader.ranges or str(code) not in self.profile['centers']):
                        raise OperatorValueError('Measure centers for mapped axes first')
                if self.reader.device.identity != self.profile['device']:
                    raise OperatorValueError('Joystick identity changed')
                self.input_status = Message('Release the deadman and center all mapped axes to prepare a stroke.')
                self.window.log('Joystick teaching enabled: {mode}; deadman {deadman}, STOP {stop}',
                    mode=Message(self.mode.currentData()), deadman=self.profile['deadman'], stop=self.profile['stop'])
                self.window.log('Joystick speed ceilings: joint/rotation/plane {native:.1f}%, Base XYZ {tcp:.1f} mm/s before throttle.',
                    native=self.profile['max_speed'], tcp=self.profile['tcp_speed'])
            except (ValueError, TypeError, KeyError) as exc:
                self.enabled.setChecked(False)
                self.input_status = Message('Joystick setup error: {detail}', dict(detail=error_message(exc)))
        else:
            self.suspend()
        self.config.setEnabled(not self.enabled.isChecked())
        self.render_labels()

    def collect_profile(self):
        data = self.profile | dict(motion_mode=self.mode.currentIndex(), axes=[c.currentData() for c in self.axis_combos],
            invert=[c.isChecked() for c in self.inverts],
            deadzone=self.deadzone.value()/100, response=self.response.value(), max_speed=self.max_speed.value(),
            tcp_speed=self.tcp_speed.value(), throttle_axis=self.throttle_combo.currentData(),
            throttle_invert=self.throttle_invert.isChecked(),
            **{k: c.currentData() for k, c in self.button_combos.items()})
        self.profile = validate_profile(data)

    def save_profile(self):
        self.suspend()
        try:
            self.collect_profile()
            write_calibration(self.profile_path, self.profile)
            self.input_status = Message('Joystick profile saved to {path}. Motion remains disabled.', dict(path=str(self.profile_path)))
        except (OSError, ValueError, TypeError) as exc:
            self.input_status = Message('Joystick setup error: {detail}', dict(detail=error_message(exc)))
        self.render_labels()

    def measure_center(self):
        self.suspend()
        if not self.reader:
            return
        self.sampling = []
        self.input_status = Message('Leave the stick and twist released. Sampling center; press no buttons.')
        self.render_labels()

    def capture(self):
        w = self.window
        if w.client.busy or w.active_jog or w.state is None or not w.state['stopped']:
            return
        tab = w.tabs.currentIndex()
        if tab == 0:
            w.capture('tool')
        elif tab == 1 and len(w.plane_points) < 3:
            w.capture(f'plane{len(w.plane_points)}')

    def open_standard_tool(self):
        self.suspend()
        root = Path(__file__).resolve().parents[1]
        if (root/'.tools/linux-joystick/root/usr/libexec/jstest-gtk').exists():
            started, _ = QProcess.startDetached(str(root/'tools/joystick-tools'), ['jstest-gtk'])
        elif shutil.which('jstest-gtk'):
            started, _ = QProcess.startDetached(shutil.which('jstest-gtk'), [])
        else:
            started = False
        if not started:
            self.input_status = Message('jstest-gtk is unavailable. Run tools/install_joystick_tools.py to install a local copy.')
        self.render_labels()

    def permission_block_reason(self):
        w = self.window
        state = w.state
        if not self.reader:
            return Message('Open a joystick input monitor first.')
        if not self.enabled.isChecked():
            return Message('Click Enable joystick teaching after choosing the motion mode.')
        if not w.connected:
            return Message('Connect to a robot or simulator to jog.')
        if QApplication.activeWindow() is not w:
            return Message('Focus the robot application window to use the joystick.')
        if self.mode.currentIndex() == 4:
            if not w.arm.isChecked():
                return Message('Enable Cartesian motion in XYZ controls before using joystick plane mode.')
        elif not w.teach_enabled.isChecked():
            return Message('Enable teaching jog controls')
        if not state or time.monotonic()-w.state_time >= 1.2:
            return Message('Waiting for fresh robot feedback.')
        if not state['safe'] or any(state['errors']):
            return Message('Robot reports a stop, collision, or fault')
        if not state['enabled']:
            return Message('Enable the robot using the controller first')
        if state['mode'] != 1:
            return Message('Use controller manual mode for teaching jog')
        owned_velocity = (self.mode.currentIndex() == 2 and w.client.stream_token is not None
                          and state.get('servo_active') and state.get('servo_program_running'))
        if not state['program_stopped'] and not owned_velocity:
            return Message("Stop the controller's Lua program before using this application")
        if self.mode.currentIndex() == 2 and (state['user'] != 0 or any(abs(v) > .001 for v in state.get('workpiece_offset', [0]*6))):
            return Message('Select base workpiece 0 before simultaneous joystick XYZ motion')
        return None

    def permitted(self):
        return self.permission_block_reason() is None

    def input_block_reason(self):
        reason = self.permission_block_reason()
        if reason:
            return reason
        values = []
        for code, invert in zip(self.profile['axes'], self.profile['invert']):
            if code is None:
                continue
            if code not in self.last_axes or code not in self.reader.ranges or str(code) not in self.profile['centers']:
                return Message('Measure centers for mapped axes first')
            r = self.reader.ranges[code]
            value = normalized(self.last_axes[code], r['min'], r['max'], self.profile['centers'][str(code)], invert)
            values.append((code, value))
        if not self.gate.ready:
            offcenter = [(code,v) for code,v in values if abs(v) > self.profile['deadzone']]
            if offcenter:
                return Message('Release all buttons and center {axis}: deflection {value:.0f}%, dead zone {zone:.0f}%. Remeasure center if the stick is already released.',
                    dict(axis=AXES.get(offcenter[0][0], str(offcenter[0][0])), value=abs(offcenter[0][1])*100,
                         zone=self.profile['deadzone']*100))
            return Message('Release the deadman and center all mapped axes to prepare a stroke.')
        if self.gate.consumed and not self.gate.active:
            return Message('This joystick stroke has ended. Recenter before starting another stroke.')
        if self.last_buttons and self.profile['deadman'] not in self.last_buttons:
            return Message('Buttons {buttons} are pressed, but the configured deadman is {deadman}. Check the button mapping.',
                dict(buttons=', '.join(map(str, sorted(self.last_buttons))), deadman=self.profile['deadman']))
        if self.mode.currentIndex() == 2 and throttle_fraction(self.last_axes, self.reader.ranges, self.profile) <= 1e-6:
            return Message('Throttle is at zero. Increase throttle for TCP movement.')
        return None

    def poll_input(self):
        if not self.reader:
            return
        try:
            pending, queued = self.window.client.pending, self.window.client.queued_command
            if not ((pending and pending[1] == 'move') or (queued and queued[0] == 'move')):
                self.plane_owned = False
            if self.enabled.isChecked() and self.gate.last_time is not None and time.monotonic()-self.gate.last_time > .15:
                self.suspend()
                self.input_status = Message('Joystick input polling was delayed. Teaching disabled; release, center, and enable again.')
                self.window.log(self.input_status)
            axes, buttons = self.reader.snapshot()
            self.last_axes, self.last_buttons = axes, buttons
            if self.sampling is not None:
                if buttons:
                    self.sampling = None
                    self.input_status = Message('Center measurement canceled: buttons were pressed.')
                else:
                    self.sampling.append(axes)
                    if len(self.sampling) >= 50:
                        samples, self.sampling = self.sampling, None
                        # Ignore unassigned throttle drift; reject a moving mapped stick.
                        for code in (c.currentData() for c in self.axis_combos):
                            if code is not None:
                                span = self.reader.ranges[code]['max']-self.reader.ranges[code]['min']
                                if max(s[code] for s in samples)-min(s[code] for s in samples) > .08*span:
                                    raise OperatorValueError('Stick moved during center measurement; try again')
                        self.profile['centers'] = {str(code): statistics.median(s[code] for s in samples) for code in axes}
                        self.save_profile()
            permission = self.permitted()
            throttle = throttle_fraction(axes, self.reader.ranges, self.profile)
            demand, stop, capture = self.gate.update(axes, buttons, self.reader.ranges, self.profile, permission,
                                                    vector_mode=self.mode.currentIndex() == 2)
            if stop and self.enabled.isChecked():
                self.suspend()
                self.window.stop(reason=Message('Joystick STOP button {button} pressed; raw buttons: {buttons}',
                    dict(button=self.profile['stop'], buttons=', '.join(map(str, sorted(buttons))))))
            elif demand is None:
                if self.window.client.stream_token is not None:
                    self.window.client.end_velocity()
                self.vector_started = False
                if self.owns_jog:
                    self.window.end_jog(self.window.active_jog[0]+1)
                self.sent = False
                if capture:
                    self.capture()
            elif self.mode.currentIndex() == 2:
                self.vector_input(demand)
            elif self.mode.currentIndex() == 4:
                w = self.window
                if not self.sent:
                    self.sent = True
                    self.plane_remaining = w.base_linear_limit.value()
                    # A stroke that starts during another command is consumed.
                    if w.client.busy or w.active_jog or not w.state['stopped']:
                        self.plane_remaining = 0.
                if self.plane_remaining > 1e-6 and not w.client.busy and not w.active_jog:
                    plane = w.current_plane()
                    if plane is not None:
                        step = min(1., self.plane_remaining)
                        target = plane.from_base(w.state['tcp'][:3])
                        target[demand.axis-1] += step*(1 if demand.direction else -1)
                        speed = min(demand.speed, w.jog_speed.value(), w.speed.value())*throttle
                        if speed <= 1e-6:
                            self.render_labels()
                            return
                        if w.send_move(target, speed=speed):
                            self.plane_owned = True
                            self.plane_remaining -= step
                        else:
                            self.plane_remaining = 0.
            elif not self.sent:
                self.sent = True  # Consume even blocked strokes; never start later from a held input.
                w = self.window
                fresh = w.state is not None and time.monotonic()-w.state_time < 1.2
                start_reason = (Message('Wait for the current controller command to finish before starting another joystick stroke.')
                                if w.client.busy or w.jog_request_active() else
                                Message(w.jog_block_reason(fresh)) if w.jog_block_reason(fresh) else None)
                if start_reason is None:
                    mode = self.mode.currentIndex()
                    ref = 0 if mode < 2 else 2
                    axis = demand.axis + (3 if mode in (1, 3) else 0)
                    distance = w.joint_limit.value() if ref == 0 else (w.base_rotation_limit.value() if axis >= 4 else w.base_linear_limit.value())
                    self.marker.setDown(True)
                    w.begin_jog(ref, axis, demand.direction, distance, self.marker,
                                speed=min(demand.speed, w.jog_speed.value()))
                    self.input_status = Message('Joystick stroke: {mode}, axis {axis}, speed {speed:.1f}%. Recenter after its travel cap.',
                                                dict(mode=Message(self.mode.currentData()), axis=axis,
                                                     speed=min(demand.speed, w.jog_speed.value())))
                    self.window.log(self.input_status)
                else:
                    self.input_status = Message('Joystick command not sent: {reason}. Recenter and try again.', dict(reason=start_reason))
                    self.window.log(self.input_status)
            if self.enabled.isChecked() and not permission:
                if self.owns_jog:
                    self.window.end_jog(self.window.active_jog[0]+1)
                self.input_status = Message('Joystick blocked: {reason}', dict(reason=self.permission_block_reason()))
            elif self.enabled.isChecked() and not self.gate.ready:
                self.input_status = Message('Release the deadman and center all mapped axes to prepare a stroke.')
            blocker = self.input_block_reason()
            attempted = bool(buttons) and any(
                code is not None and code in axes and str(code) in self.profile['centers']
                and abs(normalized(axes[code], self.reader.ranges[code]['min'], self.reader.ranges[code]['max'],
                                   self.profile['centers'][str(code)])) > self.profile['deadzone']
                for code in self.profile['axes'])
            if attempted and blocker:
                if blocker.source != self.last_blocked_input:
                    self.window.log('Joystick blocked: {reason}', reason=blocker)
                self.last_blocked_input = blocker.source
            elif blocker is None:
                self.last_blocked_input = None
            self.render_live()
        except (OSError, ValueError, KeyError) as exc:
            self.close_input()
            self.input_status = Message('Joystick input lost or invalid: {detail}. Teaching disabled.', dict(detail=error_message(exc)))
            self.window.log(self.input_status)
            self.render_labels()

    def render_live(self):
        # Input/heartbeats go first. Reformatting text and relaying out the whole
        # panel at 50 Hz can starve the very input timer it is displaying.
        self.render_status()
        now = time.monotonic()
        if now-self.last_paint_at < .1:
            return
        self.last_paint_at = now
        self.monitor.setText(self.window.tr('Raw axes: {axes}\nPressed button codes: {buttons}',
            axes=' · '.join(f'{AXES.get(a, str(a))}={v}' for a,v in self.last_axes.items()),
            buttons=', '.join(map(str, sorted(self.last_buttons))) or '—'))
        throttle = throttle_fraction(self.last_axes, self.reader.ranges, self.profile)
        self.velocity_readout.setText(self.window.tr('Throttle {throttle:.0f}% · Base XYZ limit {speed:.2f} mm/s · shared travel {travel:.1f} mm',
            throttle=throttle*100, speed=self.profile['tcp_speed']*throttle,
            travel=self.window.base_linear_limit.value()))
        if self.window.client.stream_token is not None and self.window.state:
            velocity = self.window.state.get('servo_velocity', [0., 0., 0.])
            self.velocity_readout.setText(self.velocity_readout.text()+'\n'+self.window.tr(
                'Commanded TCP velocity / mm/s: X {x:.2f} · Y {y:.2f} · Z {z:.2f}',
                x=velocity[0], y=velocity[1], z=velocity[2]))
            if 'servo_distance' in self.window.state:
                self.velocity_readout.setText(self.velocity_readout.text()+'\n'+self.window.tr(
                    'Stroke travel: {distance:.2f} / {limit:.2f} mm',
                    distance=self.window.state['servo_distance'], limit=self.window.state['servo_limit']))
            if 'servo_tracking_error' in self.window.state:
                self.velocity_readout.setText(self.velocity_readout.text()+'\n'+self.window.tr(
                    'Tracking lag {error:.3f} mm · trajectory speed {scale:.0f}% of stick request',
                    error=self.window.state['servo_tracking_error'],
                    scale=self.window.state['servo_tracking_scale']*100))
        self.render_labels()

    def render_labels(self):
        self.render_status()
        w = self.window
        self.open_button.setText(w.tr('Close input monitor' if self.reader else 'Open input monitor'))
        self.device_combo.setEnabled(self.reader is None)
        self.scan_button.setEnabled(self.reader is None)
        self.center_button.setEnabled(self.reader is not None and self.sampling is None)
        self.identity.setText(w.tr('Joystick {name} · USB {vendor}:{product} · {path}',
            name=self.reader.device.name, vendor=self.reader.device.vendor, product=self.reader.device.product,
            path=self.reader.device.path) if self.reader else w.tr('No joystick open.'))
        for i in range(self.mode.count()):
            self.mode.setItemText(i, w.tr(self.mode.itemData(i)))
        for combo in [*self.axis_combos, *self.button_combos.values(), self.throttle_combo]:
            combo.setItemText(0, w.tr('Disabled'))

    def render_status(self):
        w = self.window
        self.mapping_summary.setText(w.tr('Active mapping: {axes} · hold {deadman} · STOP {stop}',
            axes=' / '.join(AXES.get(code, str(code)) if code is not None else '—' for code in self.profile['axes']),
            deadman=self.profile['deadman'], stop=self.profile['stop']))
        self.status.setText(w.tr(self.input_status))
        blocker = self.input_block_reason()
        self.enabled.setText(w.tr('Disable joystick teaching' if self.enabled.isChecked() else 'Enable joystick teaching'))
        self.enable_state.setText(w.tr('JOYSTICK DISABLED' if not self.enabled.isChecked() else
            'JOYSTICK ENABLED — action required below' if blocker else 'JOYSTICK READY — hold trigger to jog'))
        self.readiness.setText(w.tr(Message('Joystick blocked: {reason}', dict(reason=blocker)) if blocker else
            Message('Joystick ready. Hold button {button} and deflect a mapped axis to jog.', dict(button=self.profile['deadman']))))
        self.linear_limit.setEnabled(not w.client.busy and w.active_jog is None)

    def vector_input(self, demand):
        w = self.window
        if not self.sent and any(abs(v) > 1e-9 for v in demand.velocity):
            self.sent = True
            reason = (Message('Wait for the current controller command to finish before starting another joystick stroke.')
                      if w.client.busy or w.jog_request_active() else
                      Message(w.jog_block_reason(True)) if w.jog_block_reason(True) else None)
            if reason is None:
                self.vector_started = w.client.start_velocity(w.base_linear_limit.value(), self.profile['tcp_speed'])
                if self.vector_started:
                    w.orientation = None
                    w.saved_orientation = None
                    self.input_status = Message('Simultaneous Base XYZ velocity active. Release or center to stop; throttle changes the live speed limit.')
            else:
                self.input_status = Message('Joystick command not sent: {reason}. Recenter and try again.', dict(reason=reason))
                w.log(self.input_status)
        if self.vector_started and w.client.stream_token is not None:
            w.client.update_velocity(list(demand.velocity))

    def velocity_finished(self, state):
        self.vector_started = False
        if self.gate.active is not None:
            self.gate.active = None
            self.gate.consumed = True
        self.input_status = Message('Base XYZ velocity stopped: {reason}. Travel {distance:.2f} / {limit:.2f} mm. Recenter before another stroke.',
            dict(reason=Message({'released':'Trigger released or stick centered', 'travel_limit':'Linear travel limit reached',
                 'input_timeout':'Joystick heartbeat expired', 'timing_delay':'Servo cycle delayed'}.get(state['servo_reason'], 'Velocity session ended')),
                 distance=state['servo_distance'], limit=state['servo_limit']))
        if state['servo_reason'] in ('input_timeout', 'timing_delay'):
            self.suspend()
        self.window.log(self.input_status)
        self.render_labels()

    def velocity_rejected(self, reason):
        self.vector_started = False
        self.gate.active = None
        self.gate.consumed = True
        self.input_status = Message('Joystick command not sent: {reason}. Recenter and try again.', dict(reason=reason))
        self.render_labels()

    def shutdown(self):
        self.timer.stop()
        self.close_input()
