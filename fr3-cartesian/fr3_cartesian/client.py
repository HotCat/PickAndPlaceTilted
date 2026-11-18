"""Qt process bridge: single-flight commands, bounded waits, independent stop channel."""
import json
import sys
import time
import uuid
from PySide6.QtCore import QObject, QProcess, QTimer, Signal
from .messages import Message


class RobotClient(QObject):
    completed = Signal(str, object)
    failed = Signal(object)
    rejected = Signal(object)
    diagnostic = Signal(object)
    stopped = Signal(bool)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.process = None
        self.pending = None
        self.queued_command = None
        self.counter = 0
        self.buffer = b''
        self.simulation = True
        self.ip = ''
        self.stop_process = None
        self.stream_token = None
        self.stream_state_at = 0.
        self.stream_stopping = False
        self.servo_may_be_active = False
        self.timer = QTimer(self)
        self.timer.setInterval(100)
        self.timer.timeout.connect(self.watchdog)
        self.timer.start()

    @property
    def busy(self):
        return self.stream_token is not None or (self.pending is not None and self.pending[1] != 'read') or self.queued_command is not None or self.stop_process is not None

    @property
    def reading(self):
        return self.pending is not None and self.pending[1] == 'read'

    def connect_robot(self, **args):
        self.shutdown()
        self.simulation, self.ip = args['simulation'], args['ip']
        self.process = QProcess(self)
        self.process.setProgram(sys.executable)
        self.process.setArguments(['-m', 'fr3_cartesian.worker'])
        self.process.readyReadStandardOutput.connect(self.receive)
        self.process.readyReadStandardError.connect(self.stderr)
        self.process.finished.connect(self.exited)
        self.process.start()
        self.call('connect', args)

    def call(self, method, args=None):
        if method == 'stop_jog':
            if self.queued_command and self.queued_command[0] == 'start_jog':
                self.queued_command = None
                return True
            if self.pending and self.pending[1] == 'start_jog':
                self.queued_command = (method, args)
                return True
            if self.reading:
                self.queued_command = (method, args)
                return True
        if self.busy or self.process is None:
            return False
        if self.reading:
            if method == 'read':
                return False
            # A user command takes priority over the next telemetry poll. Keep
            # exactly one request queued; never overlap use of the SDK proxy.
            self.queued_command = (method, args)
            return True
        return self.send(method, args)

    def send(self, method, args=None):
        if method == 'servo_start':
            self.servo_may_be_active = True
            self.stream_stopping = False
        self.counter += 1
        self.pending = (self.counter, method, time.monotonic())
        self.process.write((json.dumps(dict(id=self.counter, method=method, args=args or {}))+'\n').encode())
        return True

    def start_velocity(self, travel, speed):
        token = uuid.uuid4().hex
        if not self.call('servo_start', dict(token=token, travel=travel, speed=speed, sent_at=time.monotonic())):
            return False
        self.stream_token = token
        self.stream_state_at = time.monotonic()
        return True

    def update_velocity(self, velocity):
        if self.stream_token is None or self.process is None or self.stream_stopping:
            return False
        self.process.write((json.dumps(dict(method='servo_update', args=dict(
            token=self.stream_token, velocity=velocity, sent_at=time.monotonic())))+'\n').encode())
        return True

    def end_velocity(self):
        token = self.stream_token
        if token is None or self.stream_stopping:
            return True
        if self.process is None:
            self.stream_token = None
            return False
        if ((self.pending and self.pending[1] == 'servo_end') or
                (self.queued_command and self.queued_command[0] == 'servo_end')):
            return True
        self.update_velocity([0., 0., 0.])
        if self.queued_command and self.queued_command[0] == 'servo_start':
            self.queued_command = None
            self.stream_token = None
            return True
        if self.pending:
            self.queued_command = ('servo_end', dict(token=token))
            return True
        return self.send('servo_end', dict(token=token))

    def receive(self):
        self.buffer += bytes(self.process.readAllStandardOutput())
        while b'\n' in self.buffer:
            line, self.buffer = self.buffer.split(b'\n', 1)
            try:
                response = json.loads(line)
                if 'event' in response:
                    if response['event'] == 'servo_stopping':
                        if response['token'] == self.stream_token:
                            self.stream_stopping = True
                            self.stream_state_at = time.monotonic()
                    elif response['event'] == 'servo_error':
                        self.stream_token = None
                        self.failed.emit(Message.from_payload(response['error']))
                    elif response['event'] in ('servo_state', 'servo_end'):
                        self.stream_state_at = time.monotonic()
                        if response['event'] == 'servo_end':
                            self.stream_token = None
                            self.stream_stopping = False
                            self.servo_may_be_active = False
                        self.completed.emit(response['event'], response['result'])
                    continue
                if self.pending and response['id'] == self.pending[0]:
                    method = self.pending[1]
                    self.pending = None
                    if method == 'servo_start':
                        self.stream_state_at = time.monotonic()
                    if method == 'servo_end' or method == 'servo_start' and 'error' in response:
                        self.stream_token = None
                        self.stream_stopping = False
                    if method == 'servo_end' and 'error' not in response:
                        self.servo_may_be_active = False
                    if method == 'servo_start' and response.get('recoverable') is True:
                        self.servo_may_be_active = False
                    if 'error' in response:
                        self.queued_command = None
                        message = Message.from_payload(response['error'])
                        if method in ('move', 'servo_start') and response.get('recoverable') is True:
                            self.rejected.emit(message)
                        else:
                            self.failed.emit(message)
                    else:
                        queued, self.queued_command = self.queued_command, None
                        if queued:
                            self.send(*queued)
                        if response['result'] is not None:
                            self.completed.emit(method, response['result'])
            except (ValueError, KeyError) as exc:
                self.failed.emit(Message('Invalid worker reply: {detail}', {'detail': str(exc)}))

    def stderr(self):
        self.diagnostic.emit(bytes(self.process.readAllStandardError()).decode(errors='replace').strip())

    def exited(self, code, status):
        if self.pending or self.stream_token is not None:
            self.pending = None
            self.stream_token = None
            self.failed.emit(Message('SDK worker exited ({code})', {'code': code}))

    def watchdog(self):
        stream_limit = 4. if self.stream_stopping else .5
        if self.stream_token is not None and self.pending is None and time.monotonic()-self.stream_state_at > stream_limit:
            self.shutdown()
            self.stop()
            self.failed.emit(Message('Velocity feedback timed out. Motion disabled; reconnect required.'))
            return
        if self.pending:
            limit = 20 if self.pending[1] in ('connect', 'move') else 4
            if time.monotonic()-self.pending[2] > limit:
                connecting = self.pending[1] == 'connect'
                self.shutdown()
                if connecting:
                    self.failed.emit(Message('Connection timed out. No motion commands were sent. Check controller services and network, then reconnect.'))
                else:
                    self.stop()
                    self.failed.emit(Message('Controller request timed out. Worker terminated; reconnect required. Use physical stop if needed.'))

    def stop(self):
        if self.stop_process:
            return
        if self.simulation:
            self.stopped.emit(True)
            return
        proc = QProcess(self)
        self.stop_process = proc
        proc.setProgram(sys.executable)
        proc.setArguments(['-m', 'fr3_cartesian.worker', '--stop-servo' if self.servo_may_be_active else '--stop', self.ip])
        proc.finished.connect(self.stop_finished)
        proc.start()
        QTimer.singleShot(5500, lambda: proc.kill() if self.stop_process is proc and proc.state() != QProcess.NotRunning else None)

    def stop_finished(self, code, status):
        proc = self.stop_process
        self.stop_process = None
        if code != 0:
            self.diagnostic.emit(Message('Stop was not confirmed: {detail}', {'detail': bytes(proc.readAllStandardError()).decode(errors='replace')}))
        else:
            self.servo_may_be_active = False
        self.stopped.emit(code == 0)
        proc.deleteLater()

    def shutdown(self):
        self.stream_token = None
        self.stream_stopping = False
        self.pending = None
        self.queued_command = None
        self.buffer = b''
        if self.process:
            self.process.finished.disconnect(self.exited)
            self.process.kill()
            self.process.waitForFinished(1000)
            self.process.deleteLater()
            self.process = None
