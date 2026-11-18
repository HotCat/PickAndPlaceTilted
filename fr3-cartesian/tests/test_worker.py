from dataclasses import asdict
import time
import sys
from types import SimpleNamespace
import pytest
from fr3_cartesian.geometry import Plane, calibrate_pivot, pose_to_base, rotation, rpy
import numpy as np
from fr3_cartesian.worker import CommandInput, Session, NoRetryTransport, stop_robot
from fr3_cartesian.messages import MoveRejected, OperatorError


@pytest.mark.parametrize('feedback_open,command_open,port', [(False, False, 20004), (True, False, 20003)])
def test_connection_failure_identifies_unavailable_controller_channel(monkeypatch, tmp_path, feedback_open, command_open, port):
    class Robot:
        is_conect = command_open
        sock_cli_state_state = feedback_open
        def __init__(self, ip): pass
        def GetSDKVersion(self): return 0, ['offline test SDK']
    monkeypatch.setattr(sys, 'path', list(sys.path))
    monkeypatch.setitem(sys.modules, 'fairino', SimpleNamespace(Robot=SimpleNamespace(RPC=Robot, __file__='offline-test.py')))
    session = Session()
    monkeypatch.setattr(session, 'read', lambda: pytest.fail('Must not read an unconnected SDK'))
    with pytest.raises(OperatorError, match=f'192.168.58.2:{port}'):
        session.connect(simulation=False, sdk_path=str(tmp_path))


def calibrated_session():
    s = Session()
    s.connect()
    contacts = [s.demo_pose('pivot', i)['flange'] for i in range(4)]
    tool = calibrate_pivot(contacts).tool
    s.apply_tool(tool, 1, 1, 0)
    points = [s.demo_pose('plane', i)['tcp'][:3] for i in range(3)]
    p = Plane.teach('pick', points)
    return s, p


def test_simulated_calibration_to_plane_motion():
    s, p = calibrated_session()
    current = p.from_base(s.read()['tcp'][:3])
    target = [current[0]+5, current[1], current[2]+2]
    result = s.move(asdict(p), target, [180,0,0], [[-200,200],[-200,200],[-.1,200]], 3)
    assert p.from_base(result['tcp'][:3]) == pytest.approx(target)
    assert result['tcp'][3:] == pytest.approx([180,0,0])


def test_simulated_plane_arrow_release_stops_short_of_target_and_can_move_again():
    s, p = calibrated_session()
    before = np.array(s.read()['tcp'][:3])
    origin = p.from_base(before)
    started = time.monotonic()
    s.move_cancelled = lambda token: time.monotonic()-started > .06
    state = s.move(asdict(p), [origin[0]+10, origin[1], origin[2]], [180, 0, 0],
                   [[-200, 200], [-200, 200], [-1, 200]], 3, jog_token='stroke')
    assert state['plane_jog_stopped'] and state['stopped']
    distance = np.linalg.norm(np.array(state['tcp'][:3])-before)
    assert 0 < distance < 5
    # An ordinary absolute move is independent of the released stroke's token.
    target = p.from_base(state['tcp'][:3])
    target[0] += 1
    state = s.move(asdict(p), target, [180, 0, 0], [[-200, 200], [-200, 200], [-1, 200]], 3)
    assert p.from_base(state['tcp'][:3]) == pytest.approx(target)


def test_command_input_framing_token_isolation_and_eof_release():
    import json
    import os
    read_fd, write_fd = os.pipe()
    inbox = CommandInput(read_fd)
    try:
        normal = dict(id=2, method='read')
        os.write(write_fd, (json.dumps(normal)+'\n'+json.dumps(dict(method='cancel_move', token='old'))+'\n').encode())
        assert not inbox.cancelled('new')
        assert list(inbox.requests) == [normal]
        os.write(write_fd, b'{"method":"cancel_move",')
        assert not inbox.cancelled('new')
        os.write(write_fd, b'"token":"new"}\n')
        assert inbox.cancelled('new')
        assert list(inbox.requests) == [normal]
        os.close(write_fd)
        write_fd = None
        assert inbox.cancelled('next')
    finally:
        os.close(read_fd)
        if write_fd is not None:
            os.close(write_fd)


def test_worker_rejects_motion_before_tool_and_out_of_range_jog():
    s = Session()
    p = Plane.teach('xy', [[0,0,0],[100,0,0],[0,100,0]])
    with pytest.raises(RuntimeError, match='Apply'):
        s.move(asdict(p), [0,0,0], [180,0,0], [[-500,500]]*3, 3)
    with pytest.raises(ValueError):
        s.start_jog(0, 1, 1, 11, 3)


def test_simulated_joint_and_base_jogs_stop_on_release_within_limits(monkeypatch):
    s = Session()
    s.connect()
    start_joint = s.read()['joints'][0]
    s.start_jog(0, 1, 1, .5, 3)
    time.sleep(.05)
    joint = s.stop_jog(1)
    assert start_joint < joint['joints'][0] < start_joint + .5
    assert joint['stopped']

    start_x = joint['tcp'][0]
    s.start_jog(2, 1, 1, 5, 3)
    time.sleep(.05)
    base = s.stop_jog(3)
    assert start_x < base['tcp'][0] < start_x + 5
    assert base['stopped']


def test_fairino_jog_refs_and_release_stop_refs_are_mapped():
    class Robot:
        def __init__(self): self.calls = []
        def StartJOG(self, *args, **kwargs): self.calls.append(('start', args, kwargs)); return 0
        def StopJOG(self, ref): self.calls.append(('stop', ref)); return 0

    s = Session()
    s.sim = False
    s.robot = Robot()
    state = dict(safe=True, errors=[0, 0], stopped=True, program_stopped=True, enabled=True, mode=1)
    s.ready = lambda motion=False: state
    s.read = lambda: state
    s.start_jog(0, 2, 1, 1.5, 4)
    s.stop_jog(1)
    s.start_jog(2, 3, 0, 12, 5)
    s.stop_jog(3)
    assert s.robot.calls == [
        ('start', (0, 2, 1, 1.5), {'vel':4, 'acc':10}),
        ('stop', 1),
        ('start', (2, 3, 0, 12), {'vel':5, 'acc':10}),
        ('stop', 3),
    ]


@pytest.mark.parametrize('active_slot,offset,matched', [
    (1, [0, 0, 64, 0, 0, 0], True),
    (2, [0, 0, 64, 0, 0, 0], False),
    (1, [0, 0, 63, 0, 0, 0], False),
    (1, [0, 0, 64, 0, 0, 1], False),
])
def test_read_only_tcp_verification_checks_slot_and_all_offsets(active_slot, offset, matched):
    class Robot:
        reconnect_flag = False
        def __init__(self):
            self.calls = []
            self.robot = self
            self.robot_state_pkg = SimpleNamespace(frame_cnt=1, tool=active_slot, user=0,
                jt_cur_pos=[0]*6, main_code=0, sub_code=0, robot_state=1, motion_done=1,
                program_state=1, rbtEnableState=1, robot_mode=1, EmergencyStop=0,
                collisionState=0, safety_stop0_state=0, safety_stop1_state=0)
        def GetControllerIP(self):
            self.calls.append('ip')
            return 0, '192.168.58.2'
        def GetActualToolFlangePose(self, flag):
            self.calls.append('flange')
            return [0, 300, 0, 350, 180, 0, 0]
        def GetActualTCPNum(self, flag): return [0, self.robot_state_pkg.tool]
        def GetActualWObjNum(self, flag): return [0, self.robot_state_pkg.user]
        def GetWObjOffset(self): return 0, [0]*6
        def GetTCPOffset(self):
            self.calls.append('offset')
            return 0, offset
    s = Session()
    s.sim = False
    s.robot = Robot()
    state = s.verify_tool([0, 0, 64, 0, 0, 0], 1)
    assert state['tcp_verified'] is matched
    assert s.robot.calls == ['ip', 'offset', 'flange', 'offset']  # Only read APIs exist on this fake.
    assert state['tcp'][2] == pytest.approx(350-offset[2])
    if matched:
        assert s.read()['tcp_verified']
        s.robot.robot_state_pkg.tool = 2
        assert not s.read()['tcp_verified']  # Changing the active tool revokes confirmation.


def test_transport_does_not_retry_uncertain_command(monkeypatch):
    transport = NoRetryTransport()
    calls = []
    def lost(*args):
        calls.append(args)
        raise ConnectionResetError('lost after write')
    monkeypatch.setattr(transport, 'single_request', lost)
    with pytest.raises(RuntimeError, match='communication'):
        transport.request('host', '/', b'command')
    assert len(calls) == 1


class FrameFeedback:
    """Controller semantics observed on FR3: flange pose is Wobj-relative."""
    reconnect_flag = False

    def __init__(self):
        self.robot = self
        self.base_flange = [252, 440, 55, 179.93, 1.209, -89.007]
        self.workpiece = [0.]*6
        self.tool = [-.45, -.949, 63.886, 0, 0, 0]
        self.robot_state_pkg = SimpleNamespace(frame_cnt=1, jt_cur_pos=[0]*6,
            main_code=0, sub_code=0, robot_state=1, motion_done=1, program_state=1,
            rbtEnableState=1, robot_mode=1, EmergencyStop=0, collisionState=0,
            safety_stop0_state=0, safety_stop1_state=0)
    def GetControllerIP(self): return 0, '192.168.58.2'
    def GetActualTCPNum(self, flag): return 0, 1
    def GetActualWObjNum(self, flag): return 0, 0 if self.workpiece == [0.]*6 else 1
    def GetTCPOffset(self): return 0, self.tool
    def GetWObjOffset(self): return 0, list(self.workpiece)
    def GetActualToolFlangePose(self, flag):
        r = rotation(self.workpiece[3:])
        return [0, *(r.T @ (np.array(self.base_flange[:3])-self.workpiece[:3])),
                *rpy(r.T @ rotation(self.base_flange[3:]))]

def test_real_feedback_is_invariant_when_active_wobj_changes():
    session = Session()
    session.sim = False
    robot = FrameFeedback()
    session.robot = robot
    session.slot, session.tool = 1, list(robot.tool)
    first = session.read()
    for frame in ([57.148, -356.306, 134.138, 18.649, 2.301, -133.972],
                  [-707.107, -450.282, -29.552, -162.821, 7.713, 163.7]):
        robot.workpiece = frame
        state = session.capture()
        np.testing.assert_allclose(state['flange'][:3], first['flange'][:3], atol=1e-9)
        np.testing.assert_allclose(rotation(state['flange'][3:]), rotation(first['flange'][3:]), atol=1e-9)
        np.testing.assert_allclose(state['tcp'][:3], first['tcp'][:3], atol=1e-9)
        assert state['tcp_verified'] and state['pose_reference'] == 'robot_base'
        assert state['user'] == 1 and state['workpiece_offset'] == frame


def test_pivot_and_plane_capture_survive_different_workpieces_at_each_sample():
    session = Session()
    session.sim = False
    robot = FrameFeedback()
    session.robot = robot
    simulator = Session()
    contacts = []
    frames = [[0.]*6, [57, -356, 134, 19, 2, -134], [-707, -450, -29, -163, 8, 164]]
    for i in range(6):
        robot.base_flange = simulator.demo_pose('pivot', i)['flange']
        robot.workpiece = frames[i % 3]
        contacts.append(session.capture()['flange'])
    tool = calibrate_pivot(contacts).tool
    np.testing.assert_allclose(tool, [0, 0, 120, 0, 0, 0], atol=1e-9)
    robot.tool = tool
    simulator.apply_tool(tool, 1, 1, 0)
    points, expected = [], []
    for i in range(3):
        state = simulator.demo_pose('plane', i)
        robot.base_flange, robot.workpiece = state['flange'], frames[i]
        points.append(session.capture()['tcp'][:3])
        expected.append(state['tcp'][:3])
    np.testing.assert_allclose(Plane.teach('pick', points).pose(), Plane.teach('pick', expected).pose(), atol=1e-9)


@pytest.mark.parametrize('settles', [True, False])
def test_feedback_retries_only_reads_on_coordinate_switch_and_never_mixes_frames(settles):
    session = Session()
    session.sim = False
    robot = FrameFeedback()
    session.robot = robot
    original = robot.GetActualToolFlangePose
    count = 0
    def changing(flag):
        nonlocal count
        count += 1
        pose = original(flag)
        if not settles or count == 1:
            robot.workpiece = [count*10, 20, 30, 0, 0, 15]
        return pose
    robot.GetActualToolFlangePose = changing
    if settles:
        np.testing.assert_allclose(session.read()['flange'][:3], robot.base_flange[:3], atol=1e-9)
    else:
        with pytest.raises(OperatorError, match='coordinate reference changed'):
            session.read()
    assert count == 2


def test_plane_move_is_sent_in_base_with_matching_controller_frame():
    session, plane, state, calls = fake_plane_move(0)
    result = session.move(asdict(plane), [4, 3, 2], [180, 0, 0], [[-100, 100]]*3, 3)
    pose, kwargs = calls[0]
    assert pose[:3] == pytest.approx(plane.to_base([4, 3, 2]))
    assert pose[3:] == [180, 0, 0]
    assert kwargs == dict(tool=1, user=0, vel=3, ovl=100, blendR=0, offset_flag=0)
    assert result['user'] == 0


def fake_plane_move(selected_workpiece=0, sdk_error=0):
    plane = Plane.teach('tilted', [[300, 0, 200], [400, 20, 200], [280, 100, 210]])
    state = dict(tcp=[300, 0, 200, 180, 0, 0], tool=1, user=selected_workpiece,
                 safe=True, errors=[0, 0], enabled=True, stopped=True, program_stopped=True)
    calls = []
    class Robot:
        def GetTCPOffset(self): return 0, [0, 0, 64, 0, 0, 0]
        def GetWObjOffset(self): return 0, [0]*6
        def MoveL(self, pose, **kwargs):
            calls.append((pose, kwargs))
            state['tcp'] = pose
            return sdk_error
    session = Session()
    session.sim = False
    session.tool = [0, 0, 64, 0, 0, 0]
    session.slot = 1
    session.robot = Robot()
    session.ready = lambda motion=False: state
    session.read = lambda: state
    return session, plane, state, calls


@pytest.mark.parametrize('selected_workpiece', [1, 7])
def test_non_base_workpiece_rejects_before_sending_motion(selected_workpiece):
    session, plane, state, calls = fake_plane_move(selected_workpiece)
    with pytest.raises(MoveRejected, match=f'currently {selected_workpiece}'):
        session.move(asdict(plane), [4, 3, 2], [180, 0, 0], [[-100, 100]]*3, 3)
    assert not calls


def test_non_identity_base_workpiece_rejects_before_motion():
    session, plane, state, calls = fake_plane_move()
    session.robot.GetWObjOffset = lambda: (0, [1, 0, 0, 0, 0, 0])
    with pytest.raises(MoveRejected, match='nonzero offset'):
        session.move(asdict(plane), [4, 3, 2], [180, 0, 0], [[-100, 100]]*3, 3)
    assert not calls


@pytest.mark.parametrize('ack_user,ack_offset,confirmed', [
    (0, [0]*6, True), (1, [0]*6, False), (0, [1, 0, 0, 0, 0, 0], False),
])
def test_select_base_frame_changes_only_slot_zero_and_requires_readback(monkeypatch, ack_user, ack_offset, confirmed):
    calls = []
    class Robot:
        def SetWObjCoord(self, slot, pose, ref):
            calls.append(('apply', slot, pose, ref))
            return 0
        def GetWObjOffset(self):
            calls.append(('offset',))
            return 0, ack_offset
    session = Session()
    session.sim = False
    session.robot = Robot()
    session.ready = lambda: dict(user=1, stopped=True)
    session.read = lambda: dict(user=ack_user, stopped=True)
    ticks = iter([0, 0, 3])
    monkeypatch.setattr('fr3_cartesian.worker.time.monotonic', lambda: next(ticks))
    monkeypatch.setattr('fr3_cartesian.worker.time.sleep', lambda delay: None)
    if confirmed:
        assert session.select_base_frame()['user'] == 0
    else:
        with pytest.raises(OperatorError, match='did not confirm'):
            session.select_base_frame()
    assert calls == [('apply', 0, [0]*6, 0), ('offset',)]


def test_linear_point_error_is_decoded_and_never_retried(capsys):
    session, plane, state, calls = fake_plane_move(sdk_error=74)
    with pytest.raises(OperatorError, match='linear target point error') as error:
        session.move(asdict(plane), [4, 3, 2], [180, 0, 0], [[-100, 100]]*3, 3)
    assert not isinstance(error.value, MoveRejected)  # A real controller fault still stops the session.
    assert len(calls) == 1
    assert 'target_base=' in capsys.readouterr().err


@pytest.mark.parametrize('slot', [0, 15, 1.5, True])
def test_plane_upload_rejects_invalid_slot_without_writing(slot):
    session, plane = calibrated_session()
    with pytest.raises(ValueError, match='between 1 and 14'):
        session.apply_plane(asdict(plane), slot)
    assert session.read()['user'] == 0


@pytest.mark.parametrize('ack_user,position_error,angle_error', [(2, 0, 0), (3, 1, 0), (3, 0, 1)])
def test_plane_upload_requires_both_selected_slot_and_matching_pose(monkeypatch, ack_user, position_error, angle_error):
    session, plane, state, calls = fake_plane_move()
    state['tcp_verified'] = True
    state['user'] = ack_user
    class Robot:
        def SetWObjList(self, slot, pose, ref): calls.append(('save', slot, pose, ref)); return 0
        def SetWObjCoord(self, slot, pose, ref): calls.append(('apply', slot, pose, ref)); return 0
        def GetWObjOffset(self):
            pose = plane.pose()
            pose[0] += position_error
            pose[5] += angle_error
            return 0, pose
    session.robot = Robot()
    ticks = iter([0, 0, 3])
    monkeypatch.setattr('fr3_cartesian.worker.time.monotonic', lambda: next(ticks))
    monkeypatch.setattr('fr3_cartesian.worker.time.sleep', lambda delay: None)
    with pytest.raises(OperatorError, match='did not confirm'):
        session.apply_plane(asdict(plane), 3)
    assert [call[0] for call in calls] == ['save', 'apply']


def test_plane_upload_failure_is_not_replayed_or_followed_by_selection():
    session, plane, state, calls = fake_plane_move()
    state['tcp_verified'] = True
    class Robot:
        def SetWObjList(self, *args):
            calls.append('save')
            raise OperatorError('communication lost after write')
        def SetWObjCoord(self, *args): calls.append('apply'); return 0
    session.robot = Robot()
    with pytest.raises(OperatorError, match='communication lost'):
        session.apply_plane(asdict(plane), 3)
    assert calls == ['save']


@pytest.mark.parametrize('tool,verified,slot', [(2, True, 1), (1, False, 1), (1, True, 0)])
def test_plane_upload_requires_matching_calibrated_tcp_before_writes(tool, verified, slot):
    session, plane, state, calls = fake_plane_move()
    state.update(tool=tool, tcp_verified=verified)
    session.slot = slot
    with pytest.raises(OperatorError, match='Verify the calibrated TCP'):
        session.apply_plane(asdict(plane), 1)
    assert not calls


def test_worker_refuses_legacy_plane_upload_and_motion_without_rpc_writes():
    session, plane, state, calls = fake_plane_move()
    plane.reference = 'legacy_workpiece_unknown'
    with pytest.raises(OperatorError, match='old coordinate-frame bug'):
        session.apply_plane(asdict(plane), 1)
    with pytest.raises(MoveRejected, match='old coordinate-frame bug'):
        session.move(asdict(plane), [0,0,0], [180,0,0], [[-100,100]]*3, 3)
    assert not calls


def test_tool_mismatch_is_recoverable_and_never_sends_a_move():
    session, plane, state, calls = fake_plane_move()
    state['tool'] = 2
    with pytest.raises(MoveRejected, match='Active controller tool 2'):
        session.move(asdict(plane), [4, 3, 2], [180, 0, 0], [[-100, 100]]*3, 3)
    assert not calls


def test_errors_after_sending_move_are_fatal():
    session, plane, state, calls = fake_plane_move(sdk_error=99)
    with pytest.raises(OperatorError, match='FAIRINO error 99') as error:
        session.move(asdict(plane), [4, 3, 2], [180, 0, 0], [[-100, 100]]*3, 3)
    assert len(calls) == 1
    assert not isinstance(error.value, MoveRejected)


def test_stop_attempts_both_commands_even_if_jog_stop_fails(monkeypatch):
    calls = []
    class Proxy:
        def __enter__(self): return self
        def __exit__(self, *args): pass
        def ImmStopJOG(self):
            calls.append('jog')
            raise RuntimeError('jog failed')
        def StopMotion(self):
            calls.append('motion')
            return 0
    monkeypatch.setattr('fr3_cartesian.worker.proxy', lambda ip: Proxy())
    with pytest.raises(RuntimeError, match='jog failed'):
        stop_robot('192.168.58.2')
    assert calls == ['jog', 'motion']
