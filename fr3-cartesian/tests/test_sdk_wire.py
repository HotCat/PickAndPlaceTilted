"""Exercise the installed official SDK through XML-RPC serialization, without sockets."""
from dataclasses import asdict
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import xmlrpc.client

import numpy as np
import pytest

from fr3_cartesian.geometry import Plane, pose_to_base, rotation
from fr3_cartesian.messages import OperatorError
from fr3_cartesian.worker import NoRetryTransport, Session


@pytest.fixture(scope='module')
def sdk():
    source = Path(__file__).resolve().parents[1] / 'vendor/fairino/Robot.py'
    if not source.exists():
        pytest.skip('Install the pinned official SDK with tools/install_sdk.py for wire tests')
    spec = importlib.util.spec_from_file_location('fairino_wire_test', source)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def wire_session(sdk, monkeypatch, failure=None):
    session = Session()
    session.sim = False
    session.slot = 1
    session.tool = [-.45, -.949, 63.886, 0, 0, 0]
    state = dict(tcp=[300, 0, 200, 180, 0, 0], tool=1, user=0,
                 safe=True, errors=[0, 0], enabled=True, stopped=True, program_stopped=True, tcp_verified=True)
    session.ready = lambda motion=False: state
    session.read = lambda: state
    # Bypass the SDK constructor: no robot connections or feedback threads.
    robot = sdk.RPC.__new__(sdk.RPC)
    robot.reconnect_flag = False
    robot.robot_state_pkg = SimpleNamespace(safety_stop0_state=0, safety_stop1_state=0)
    transport = NoRetryTransport()
    calls = []
    workpiece = [0.]*6

    def respond(host, handler, request_body, verbose=False):
        nonlocal workpiece
        params, method = xmlrpc.client.loads(request_body)
        calls.append((method, params))
        if method == 'GetTCPOffset':
            response = [0, *session.tool]
        elif method == 'GetWObjOffset':
            response = [0, *workpiece]
        elif method == 'SetWObjList':
            response = 0
        elif method == 'SetWObjCoord':
            state['user'], workpiece = params[0], params[1]
            response = 0
        elif method == 'GetInverseKin':
            response = [0, 96.497, -70, 100, -120, -90, 10]
        elif method == 'MoveL':
            if failure is not None:
                raise failure
            # Model feedback arriving at the transmitted target.
            state['tcp'] = params[0][6:12]
            response = 0
        else:
            pytest.fail(f'Unexpected RPC: {method}')
        return xmlrpc.client.loads(xmlrpc.client.dumps((response,), methodresponse=True))[0]

    monkeypatch.setattr(transport, 'single_request', respond)
    robot.robot = xmlrpc.client.ServerProxy('http://offline.invalid', transport=transport)
    session.robot = robot
    plane = Plane.teach('tilted', [[300, 0, 200], [400, 20, 200], [280, 100, 210]])
    return session, plane, calls


def test_plane_move_uses_controller_385_single_array_wire_format(sdk, monkeypatch):
    session, plane, calls = wire_session(sdk, monkeypatch)
    result = session.move(asdict(plane), [4, 3, 2], [180, 0, 0], [[-100, 100]]*3, 3)
    assert [method for method, _ in calls] == ['GetTCPOffset', 'GetWObjOffset', 'GetInverseKin', 'MoveL']
    pose = [*plane.to_base([4, 3, 2]), 180, 0, 0]
    assert calls[2][1][0] == 0  # IK in robot base
    assert calls[2][1][1] == pytest.approx(pose)
    assert calls[2][1][2] == -1
    params = calls[3][1]
    assert len(params) == 1  # The old SDK sent 13 RPC arguments, causing Fault -502.
    assert params[0] == pytest.approx([
        96.497, -70, 100, -120, -90, 10, *pose,
        1, 0, 3, 0, 100, 0, 0,  # tool, base user, speed, acc, override, blend, mode
        0, 0, 0, 0, 0, 0,       # external axes, search, offset flag
        0, 0, 0, 0, 0, 0, 100, 0,  # offset, multiplier, percent speed mode
    ])
    assert result['tcp'] == pytest.approx(pose)


def test_select_base_frame_official_sdk_wire_has_no_motion_or_taught_slot_write(sdk, monkeypatch):
    session, plane, calls = wire_session(sdk, monkeypatch)
    assert session.select_base_frame()['user'] == 0
    assert calls == [('SetWObjCoord', (0, [0.]*6, 0)), ('GetWObjOffset', (1,))]


def test_plane_upload_official_sdk_wire_writes_and_verifies_only_chosen_slot(sdk, monkeypatch):
    session, plane, calls = wire_session(sdk, monkeypatch)
    result = session.apply_plane(asdict(plane), 3)
    assert result['user'] == 3
    assert result['applied_workpiece'] == dict(name=plane.name, slot=3, pose=plane.pose())
    assert calls == [('SetWObjList', (3, plane.pose(), 0)),
                     ('SetWObjCoord', (3, plane.pose(), 0)), ('GetWObjOffset', (1,))]


@pytest.mark.parametrize('failure,expected', [
    (xmlrpc.client.Fault(-502, 'argument format rejected'), xmlrpc.client.Fault),
    (ConnectionResetError('lost after write'), OperatorError),
])
def test_official_sdk_does_not_replay_failed_move(sdk, monkeypatch, failure, expected):
    session, plane, calls = wire_session(sdk, monkeypatch, failure)
    with pytest.raises(expected):
        session.move(asdict(plane), [4, 3, 2], [180, 0, 0], [[-100, 100]]*3, 3)
    assert sum(method == 'MoveL' for method, _ in calls) == 1


def test_feedback_normalization_with_official_sdk_and_nonzero_wobj_wire_responses(sdk, monkeypatch):
    session = Session()
    session.sim = False
    session.slot = 1
    session.tool = [0., 0., 100., 0., 0., 0.]
    robot = sdk.RPC.__new__(sdk.RPC)
    robot.reconnect_flag = False
    robot.robot_state_pkg = SimpleNamespace(frame_cnt=1, jt_cur_pos=[0]*6,
        main_code=0, sub_code=0, robot_state=1, motion_done=1, program_state=1,
        rbtEnableState=1, robot_mode=1, EmergencyStop=0, collisionState=0,
        safety_stop0_state=0, safety_stop1_state=0)
    # Synthetic regression for the observed active-Wobj convention. A +90°
    # Wobj rotation maps local (20, 30, 40) to base (70, 220, 340).
    # With Rx=180°, a +100 mm tool Z offset points down to base Z=240.
    # Keep this independent expected result instead of publishing cell captures.
    responses = {
        'GetControllerIP': [0, '192.0.2.2'], 'GetActualTCPNum': [0, 1],
        'GetActualWObjNum': [0, 1],
        'GetTCPOffset': [0, *session.tool],
        'GetWObjOffset': [0, 100., 200., 300., 0., 0., 90.],
        'GetActualToolFlangePose': [0, 20., 30., 40., 180., 0., 0.],
        'GetActualTCPPose': [0, 20., 30., -60., 180., 0., 0.],
    }
    calls = []
    transport = NoRetryTransport()

    def respond(host, handler, request_body, verbose=False):
        params, method = xmlrpc.client.loads(request_body)
        calls.append((method, params))
        assert method in responses, f'Unexpected controller operation: {method}'
        assert params == (() if method == 'GetControllerIP' else (1,))
        return xmlrpc.client.loads(xmlrpc.client.dumps((responses[method],), methodresponse=True))[0]

    monkeypatch.setattr(transport, 'single_request', respond)
    robot.robot = xmlrpc.client.ServerProxy('http://offline.invalid', transport=transport)
    session.robot = robot
    state = session.read()
    assert [method for method, _ in calls] == ['GetControllerIP',
        'GetActualTCPNum', 'GetActualWObjNum', 'GetTCPOffset', 'GetWObjOffset',
        'GetActualToolFlangePose',
        'GetActualTCPNum', 'GetActualWObjNum', 'GetTCPOffset', 'GetWObjOffset']
    assert state['pose_reference'] == 'robot_base' and state['tcp_verified']
    expected = pose_to_base(responses['GetActualTCPPose'][1:], responses['GetWObjOffset'][1:])
    np.testing.assert_allclose(state['tcp'][:3], expected[:3], atol=1e-9)
    np.testing.assert_allclose(rotation(state['tcp'][3:]), rotation(expected[3:]), atol=1e-5)
    assert state['tcp'] == pytest.approx([70., 220., 240., 180., 0., 90.], abs=1e-9)


@pytest.mark.parametrize('tool,attitude', [
    ([0.]*6,[180.,0.,0.]),
    ([-.45,-.949,63.886,0.,0.,0.],[164.894,12.649,43.509]),
])
def test_servo_velocity_uses_pinned_sdk_absolute_base_tcp_wire_format_without_replay(sdk, monkeypatch, tool, attitude):
    import time
    session=Session();session.tool=tool;session.slot=1 if any(tool) else 0
    session.flange[3:]=attitude
    state=session.read();session.sim=False
    robot=sdk.RPC.__new__(sdk.RPC)
    robot.reconnect_flag=False
    pkg=SimpleNamespace(frame_cnt=1,tool=state['tool'],user=0,tl_cur_pos=list(state['tcp']),jt_cur_pos=[0]*6,
        main_code=0,sub_code=0, EmergencyStop=0,collisionState=0,safety_stop0_state=0,safety_stop1_state=0,
        rbtEnableState=1,robot_mode=1,program_state=1,robot_state=1)
    robot.robot_state_pkg=pkg
    transport=NoRetryTransport();calls=[]
    def respond(host,handler,request_body,verbose=False):
        params,method=xmlrpc.client.loads(request_body)
        calls.append((method,params))
        assert method in ('ServoMoveStart','ServoCart','StopMotion','ServoMoveEnd')
        if method=='ServoCart':
            pkg.tl_cur_pos=list(params[1])
            pkg.frame_cnt+=1
        return (0,)
    monkeypatch.setattr(transport,'single_request',respond)
    robot.robot=xmlrpc.client.ServerProxy('http://offline.invalid',transport=transport)
    session.robot=robot
    session.ready=lambda motion=False:state
    session.read=lambda:session.servo_state() if session.servo else state
    session.servo_start('wire',5,10,time.monotonic())
    session.servo_update('wire',[3,4,5],time.monotonic())
    session.servo_tick()
    session.servo_end('wire')
    assert [m for m,_ in calls]==['ServoMoveStart','ServoCart','StopMotion','ServoMoveEnd']
    params=calls[1][1]
    assert len(params)==8
    assert params[0]==0  # absolute TCP in base Wobj0
    assert all(v>origin for v,origin in zip(params[1][:3],state['tcp'][:3]))
    assert np.linalg.norm(np.array(params[1][:3])-state['tcp'][:3])<=10*.008
    assert params[1][3:]==state['tcp'][3:]
    assert params[2]==[1.]*6
    assert params[3:]==(0.,0.,.008,0.,0.)
