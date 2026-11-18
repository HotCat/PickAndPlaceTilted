"""TCP segmentation and SDK read timeouts; never use a robot connection."""
import ctypes
import importlib.util
from pathlib import Path
import socket
import struct
import threading
import time
from types import SimpleNamespace

import pytest

from fr3_cartesian.feedback import StateFrames, receive_feedback
from fr3_cartesian.worker import Session
from fr3_cartesian.messages import OperatorError


class State(ctypes.LittleEndianStructure):
    _pack_ = 1
    _fields_ = [('head',ctypes.c_uint16),('frame',ctypes.c_uint8),('length',ctypes.c_uint16),
                ('joints',ctypes.c_double*3)]


def packet(frame=1):
    state=State(0x5a5a,frame,ctypes.sizeof(State)-5,(96.497,-70,100))
    body=bytes(state)
    return body+struct.pack('<H',sum(body)&65535)


@pytest.mark.parametrize('cut',range(1,ctypes.sizeof(State)+2))
def test_state_frames_accept_every_tcp_split_including_header_and_checksum(cut):
    parser=StateFrames(ctypes.sizeof(State))
    data=packet()
    assert parser.feed(data[:cut])==[]
    frames=parser.feed(data[cut:])
    assert len(frames)==1
    assert list(State.from_buffer_copy(frames[0]).joints)==[96.497,-70,100]


def test_multiple_frames_one_byte_reads_and_bad_checksum_resynchronize():
    parser=StateFrames(ctypes.sizeof(State))
    bad=bytearray(packet(2));bad[-1]^=1
    data=b'junk'+packet(1)+bad+packet(3)+packet(4)
    frames=[]
    for value in data: frames.extend(parser.feed(bytes([value])))
    assert [State.from_buffer_copy(body).frame for body in frames]==[1,3,4]
    assert parser.buffer==b''
    assert len(StateFrames(ctypes.sizeof(State)).feed(packet(5)+packet(6)))==2


def test_checksum_overflow_is_a_uint16_sum():
    body=b'ZZ'+bytes([7])+struct.pack('<H',300)+bytes([255])*300
    assert sum(body)>65535
    wire=body+struct.pack('<H',sum(body)&65535)
    assert StateFrames(ctypes.sizeof(State)).feed(wire)==[wire]


@pytest.mark.parametrize('extra_fields', [b'', b'\x11\x22'])
def test_pinned_sdk_struct_with_checksum_and_newer_firmware_prefix(extra_fields):
    source=Path(__file__).resolve().parents[1]/'vendor/fairino/Robot.py'
    if not source.exists(): pytest.skip('Pinned SDK not installed')
    spec=importlib.util.spec_from_file_location('fairino_feedback_test',source)
    sdk=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(sdk)  # Import only; never construct RPC or open sockets.
    state=sdk.RobotStatePkg()
    state.frame_head=0x5a5a;state.frame_cnt=12
    state.program_state=1;state.robot_state=1;state.robot_mode=1;state.rbtEnableState=1
    state.jt_cur_pos[:]=[96.497,-70.,100.,-120.,-90.,10.]
    state.tl_cur_pos[:]=[300.,20.,350.,180.,0.,0.]
    state.tool=1;state.user=0
    # An exact SDK packet includes check_sum inside sizeof(State); the
    # observed controller packet also has two newer bytes before that checksum.
    state.data_len=ctypes.sizeof(state)-7+len(extra_fields)
    body=bytes(state)[:-2]+extra_fields
    wire=body+struct.pack('<H',sum(body)&65535)
    assert len(wire)==ctypes.sizeof(state)+len(extra_fields)
    robot=SimpleNamespace(closeRPC_state=False,robot_realstate_exit=False,stop_event=threading.Event(),
        sock_cli_state_state=True,SDK_state=True,reconnect_flag=False,robot_state_pkg=None)
    chunks=[wire[:4],wire[4:-1],wire[-1:]]
    class Socket:
        def recv(self,size):
            value=chunks.pop(0)
            if not chunks: robot.stop_event.set()
            return value
    robot.sock_cli_state=Socket()
    receive_feedback(robot,sdk.RobotStatePkg)
    pkg=robot.robot_state_pkg
    assert pkg is not None and list(pkg.jt_cur_pos)==list(state.jt_cur_pos)
    assert list(pkg.tl_cur_pos)==list(state.tl_cur_pos)
    assert (pkg.tool,pkg.user,pkg.rbtEnableState)==(1,0,1)
    assert robot.sock_cli_state_state and not robot.reconnect_flag
    if not extra_fields: assert pkg.check_sum==sum(body)&65535


def test_receive_timeout_keeps_socket_and_last_valid_state_without_reconnecting():
    robot=SimpleNamespace(closeRPC_state=False,robot_realstate_exit=False,stop_event=threading.Event(),
        sock_cli_state_state=True,SDK_state=True,reconnect_flag=False,robot_state_pkg=None)
    data=packet()
    bad=bytearray(packet(2));bad[-1]^=1
    chunks=[socket.timeout(),data[:1],socket.timeout(),data[1:8],data[8:],socket.timeout(),bytes(bad)]
    class Socket:
        def recv(self,size):
            value=chunks.pop(0)
            if not chunks: robot.stop_event.set()
            if isinstance(value,Exception): raise value
            return value
    robot.sock_cli_state=Socket()
    receive_feedback(robot,State)
    assert robot.sock_cli_state_state and robot.SDK_state and not robot.reconnect_flag
    assert robot.robot_state_pkg.frame==1
    assert list(robot.robot_state_pkg.joints)==[96.497,-70,100]
    assert time.monotonic()-robot._fr3_feedback_at<.1


def test_closed_socket_marks_feedback_lost_without_reconnection():
    robot=SimpleNamespace(closeRPC_state=False,robot_realstate_exit=False,stop_event=threading.Event(),
        sock_cli_state_state=True,SDK_state=True,reconnect_flag=False,
        sock_cli_state=SimpleNamespace(recv=lambda size:b''))
    receive_feedback(robot,State)
    assert not robot.sock_cli_state_state and not robot.SDK_state and robot._fr3_feedback_error
    assert not robot.reconnect_flag  # SDK stop methods must not wait for a reconnect that never runs.


def test_receive_failure_rejects_read_before_entering_any_sdk_wait_loop():
    session=Session();session.sim=False
    session.robot=SimpleNamespace(_fr3_feedback_error='EOF',reconnect_flag=False,
        GetControllerIP=lambda:pytest.fail('Do not probe a failed feedback session'))
    with pytest.raises(OperatorError,match='state stream is stale'): session.read()


def test_real_frame_age_expires_even_if_frame_counter_changes():
    session=Session();session.sim=False
    session.robot=SimpleNamespace(GetControllerIP=lambda:(0,'192.0.2.2'),reconnect_flag=False,
        robot_state_pkg=SimpleNamespace(frame_cnt=1),_fr3_feedback_at=time.monotonic()-1.)
    with pytest.raises(OperatorError,match='state stream is stale'): session.read()
