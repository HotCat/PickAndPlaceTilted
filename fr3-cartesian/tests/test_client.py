"""Exercise telemetry/command arbitration without a robot or network."""
import json
import os
import time
import pytest
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
from PySide6.QtWidgets import QApplication
from fr3_cartesian.client import RobotClient
from fr3_cartesian.messages import Message


class Process:
    def __init__(self):
        self.sent = []
        self.output = b''

    def write(self, data):
        self.sent.append(json.loads(data))

    def readAllStandardOutput(self):
        data, self.output = self.output, b''
        return data

    def reply(self, request, **fields):
        self.output = (json.dumps({'id': request['id'], **fields})+'\n').encode()


def bridge():
    app = QApplication.instance() or QApplication([])
    client = RobotClient()
    client.timer.stop()
    client.process = Process()
    return app, client


def test_poll_does_not_mark_controls_busy_and_user_command_is_not_dropped():
    app, client = bridge()
    process = client.process
    completed = []
    client.completed.connect(lambda method, result: completed.append(method))
    assert client.call('read')
    assert client.reading and not client.busy
    assert client.call('start_jog', {'ref':0, 'axis':1, 'direction':1, 'distance':.5, 'speed':3})
    assert client.busy
    assert len(process.sent) == 1  # SDK remains single-flight.
    assert not client.call('start_jog', {'ref':0, 'axis':2})  # No duplicate jog queue.
    assert not client.call('read')
    process.reply(process.sent[0], result={'stopped':True})
    client.receive()
    assert process.sent[1]['method'] == 'start_jog'
    assert client.pending[1] == 'start_jog' and client.busy
    assert completed == ['read']
    assert client.call('stop_jog', {'stop_ref':1})
    process.reply(process.sent[1], result={'stopped':True})
    client.receive()
    assert process.sent[2]['method'] == 'stop_jog'
    process.reply(process.sent[2], result={'stopped':True})
    client.receive()
    assert not client.busy and client.pending is None
    assert completed == ['read', 'start_jog', 'stop_jog']


def test_failed_poll_discards_waiting_motion():
    app, client = bridge()
    errors = []
    client.failed.connect(errors.append)
    assert client.call('read')
    assert client.call('start_jog', {'ref':0, 'axis':1})
    client.process.reply(client.process.sent[0], error=Message('Robot state stream is stale; reconnect before moving').payload())
    client.receive()
    assert len(client.process.sent) == 1
    assert client.queued_command is None
    assert errors[0].source.startswith('Robot state stream is stale')


def test_release_cancels_queued_start_or_stops_after_dispatched_start():
    app, client = bridge()
    process = client.process
    assert client.call('read')
    assert client.call('start_jog', {'ref':0, 'axis':1})
    assert client.call('stop_jog', {'stop_ref':1})
    assert client.queued_command is None
    process.reply(process.sent[0], result={'stopped':True})
    client.receive()
    assert len(process.sent) == 1

    assert client.call('start_jog', {'ref':0, 'axis':1})
    assert client.call('stop_jog', {'stop_ref':1})
    assert client.queued_command == ('stop_jog', {'stop_ref':1})
    process.reply(process.sent[-1], result={'stopped':True})
    client.receive()
    assert process.sent[-1]['method'] == 'stop_jog'


@pytest.mark.parametrize('recoverable,signal_name', [(True, 'rejected'), (False, 'failed')])
def test_move_rejection_is_distinguished_from_uncertain_motion_failure(recoverable, signal_name):
    app, client = bridge()
    process = client.process
    signals = []
    client.rejected.connect(lambda message: signals.append('rejected'))
    client.failed.connect(lambda message: signals.append('failed'))
    assert client.call('move', {})
    process.reply(process.sent[0], error=Message('Apply a calibrated tool before moving').payload(), recoverable=recoverable)
    client.receive()
    assert signals == [signal_name]
    assert client.process is process and client.pending is None
    assert not client.busy


@pytest.mark.parametrize('method,should_stop', [('connect', False), ('move', True), ('read', True)])
def test_timeout_only_requests_stop_for_an_existing_session(monkeypatch, method, should_stop):
    app, client = bridge()
    events = []
    client.pending = (1, method, time.monotonic() - 30)
    monkeypatch.setattr(client, 'shutdown', lambda: events.append('shutdown'))
    monkeypatch.setattr(client, 'stop', lambda: events.append('stop'))
    client.failed.connect(lambda message: events.append(message.source))
    client.watchdog()
    assert ('stop' in events) is should_stop
    assert events[0] == 'shutdown'
    assert events[-1].startswith('Connection timed out' if method == 'connect' else 'Controller request timed out')


def test_velocity_heartbeats_bypass_command_queue_and_release_cancels_waiting_start():
    app,client=bridge();process=client.process
    assert client.call('read')
    assert client.start_velocity(5,10)
    assert client.busy and client.queued_command[0]=='servo_start'
    assert client.update_velocity([1,2,3])
    assert process.sent[-1]['method']=='servo_update'
    assert client.end_velocity()
    assert client.stream_token is None and client.queued_command is None
    process.reply(process.sent[0],result={'stopped':True})
    client.receive()
    assert all(command['method']!='servo_start' for command in process.sent)


def test_release_during_servo_start_ends_once_after_ack_and_streaming_is_exclusive():
    app,client=bridge();process=client.process
    assert client.start_velocity(5,10)
    token=client.stream_token
    assert client.update_velocity([3,4,0])
    assert not client.call('start_jog',{})
    assert not client.call('read')
    assert client.end_velocity()
    assert client.queued_command==('servo_end',{'token':token})
    assert client.end_velocity()
    process.reply(process.sent[0],result={'servo_active':True})
    client.receive()
    assert process.sent[-1]['method']=='servo_end'
    process.reply(process.sent[-1],result={'servo_active':False})
    client.receive()
    assert client.stream_token is None and not client.busy
    assert len([m for m in process.sent if m['method']=='servo_end'])==1


def test_automatic_stream_stop_event_and_live_state_do_not_require_poll_commands():
    app,client=bridge();process=client.process;events=[]
    client.completed.connect(lambda method,state:events.append((method,state)))
    assert client.start_velocity(5,10)
    process.reply(process.sent[0],result={'servo_active':True})
    client.receive()
    process.output=(json.dumps(dict(event='servo_state',result={'servo_velocity':[1,2,3]}))+'\n').encode()
    client.receive()
    assert client.busy and events[-1][0]=='servo_state'
    process.output=(json.dumps(dict(event='servo_end',result={'servo_reason':'travel_limit'}))+'\n').encode()
    client.receive()
    assert not client.busy and client.stream_token is None
    assert events[-1][0]=='servo_end'


def test_streaming_worker_loss_and_stalled_feedback_are_not_silent(monkeypatch):
    app,client=bridge();errors=[];actions=[]
    client.failed.connect(errors.append)
    client.stream_token='active'
    client.stream_state_at=time.monotonic()-1
    monkeypatch.setattr(client,'shutdown',lambda:actions.append('shutdown'))
    monkeypatch.setattr(client,'stop',lambda:actions.append('stop'))
    client.watchdog()
    assert actions==['shutdown','stop'] and errors[-1].source.startswith('Velocity feedback')
    client.pending=None
    client.exited(1,None)
    assert errors[-1].source.startswith('SDK worker exited')


def test_uncertain_stream_shutdown_preserves_servo_cleanup_for_independent_stop(monkeypatch):
    app,client=bridge()
    client.simulation=False
    client.ip='192.0.2.2'
    assert client.start_velocity(5,10)
    assert client.servo_may_be_active
    # Shutdown kills the owner but cannot confirm controller servo mode ended.
    client.process=None
    client.shutdown()
    assert client.servo_may_be_active and client.stream_token is None
    class StopProcess:
        def __init__(self,parent):
            self.finished=type('Signal',(),{'connect':lambda *args: None})()
        def setProgram(self,value): pass
        def setArguments(self,args): self.arguments=args
        def start(self): pass
    monkeypatch.setattr('fr3_cartesian.client.QProcess',StopProcess)
    monkeypatch.setattr('fr3_cartesian.client.QTimer.singleShot',lambda *args: None)
    client.stop()
    assert client.stop_process.arguments==['-m','fr3_cartesian.worker','--stop-servo','192.0.2.2']


def test_rejected_velocity_start_does_not_leave_an_active_servo_marker():
    app,client=bridge()
    assert client.start_velocity(5,10)
    process=client.process
    process.reply(process.sent[0],error=Message('Joystick start expired; release and recenter before retrying').payload(),recoverable=True)
    client.receive()
    assert not client.servo_may_be_active and client.stream_token is None and not client.busy


def test_auto_stop_settling_is_bounded_and_does_not_trigger_active_feedback_timeout(monkeypatch):
    app,client=bridge();process=client.process;errors=[];actions=[]
    client.failed.connect(errors.append)
    assert client.start_velocity(5,10)
    process.reply(process.sent[0],result={'servo_active':True})
    client.receive()
    process.output=(json.dumps(dict(event='servo_stopping',token=client.stream_token))+'\n').encode()
    client.receive()
    assert client.stream_stopping and client.busy
    assert not client.update_velocity([3,4,0])
    assert client.end_velocity()  # Already stopping, no duplicate end request.
    client.stream_state_at=time.monotonic()-1
    client.watchdog()
    assert not errors
    monkeypatch.setattr(client,'shutdown',lambda:actions.append('shutdown'))
    monkeypatch.setattr(client,'stop',lambda:actions.append('stop'))
    client.stream_state_at=time.monotonic()-5
    client.watchdog()
    assert actions==['shutdown','stop'] and errors


def test_old_session_cannot_mark_the_new_stream_as_stopping():
    app,client=bridge();process=client.process
    assert client.start_velocity(5,10)
    process.output=(json.dumps(dict(event='servo_stopping',token='old-session'))+'\n').encode()
    client.receive()
    assert not client.stream_stopping


def test_plane_arrow_release_cancels_queued_move_before_dispatch():
    app, client = bridge()
    process = client.process
    assert client.call('read')
    assert client.start_plane_jog({'target': [1, 0, 0]})
    assert client.plane_jog_token and client.queued_command[0] == 'move'
    client.end_plane_jog()
    assert client.queued_command is None and client.plane_jog_token is None
    process.reply(process.sent[0], result={'stopped': True})
    client.receive()
    assert [command['method'] for command in process.sent] == ['read']


def test_plane_release_bypasses_pending_move_once_and_absolute_move_is_unaffected():
    app, client = bridge()
    process = client.process
    assert client.start_plane_jog({'target': [10, 0, 0]})
    token = client.plane_jog_token
    client.end_plane_jog()
    client.end_plane_jog()
    assert process.sent[1] == dict(method='cancel_move', token=token)
    assert len(process.sent) == 2 and client.queued_command is None
    assert client.busy and not client.call('move', {})
    process.reply(process.sent[0], result={'stopped': True, 'plane_jog_stopped': True})
    client.receive()
    assert client.plane_jog_token is None and client.plane_jog_cancel_at is None
    assert not client.busy
    assert client.call('move', {'target': [0, 0, 0]})
    client.end_plane_jog()
    assert process.sent[-1]['method'] == 'move' and len(process.sent) == 3


def test_plane_release_timeout_discards_owner_before_independent_stop(monkeypatch):
    app, client = bridge()
    events = []
    assert client.start_plane_jog({})
    client.end_plane_jog()
    client.plane_jog_cancel_at = time.monotonic()-3
    monkeypatch.setattr(client, 'shutdown', lambda: events.append('shutdown'))
    monkeypatch.setattr(client, 'stop', lambda: events.append('stop'))
    client.failed.connect(lambda message: events.append(message.source))
    client.watchdog()
    assert events == ['shutdown', 'stop', 'Plane jog stop timed out. Motion disabled; reconnect required.']
