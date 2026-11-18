"""Incremental FAIRINO feedback framing for the pinned SDK's receive thread."""
import ctypes
import socket
import struct
import time


class StateFrames:
    """Yield complete checksum-validated wire packets, including the checksum."""
    def __init__(self, minimum_size):
        self.minimum_size = minimum_size
        self.buffer = bytearray()

    def feed(self, chunk):
        self.buffer.extend(chunk)
        frames = []
        while True:
            head = self.buffer.find(b'ZZ')
            if head < 0:
                self.buffer[:] = b'Z' if self.buffer.endswith(b'Z') else b''
                break
            if head:
                del self.buffer[:head]
            if len(self.buffer) < 5:
                break
            size = struct.unpack_from('<H', self.buffer, 3)[0]+7
            if size < self.minimum_size:
                del self.buffer[0]
                continue
            if len(self.buffer) < size:
                break
            checksum = struct.unpack_from('<H', self.buffer, size-2)[0]
            if sum(self.buffer[:size-2]) & 65535 != checksum:
                # Keep the last valid state; corrupt data must never zero joints.
                del self.buffer[0]
                continue
            frames.append(bytes(self.buffer[:size]))
            del self.buffer[:size]
        return frames


def receive_feedback(robot, state_type):
    """A socket read timeout is not a disconnect; frame-age checks own expiry.

    Install before RPC construction so its receive thread never enters the
    vendor's automatic reconnect loop. No command channel is used here.
    """
    parser = StateFrames(ctypes.sizeof(state_type))
    try:
        while not (robot.closeRPC_state or robot.robot_realstate_exit or robot.stop_event.is_set()):
            try:
                chunk = robot.sock_cli_state.recv(65536)
            except socket.timeout:
                continue
            if not chunk:
                raise ConnectionError('Robot feedback socket closed')
            for packet in parser.feed(chunk):
                # The pinned ctypes structure includes check_sum. Newer
                # firmware can append fields before the wire checksum; copy
                # only the SDK's known prefix, just as its accessors expect.
                state = state_type.from_buffer_copy(packet)
                robot.robot_state_pkg = state
                robot._fr3_feedback_at = time.monotonic()
    except (OSError, ValueError) as exc:
        robot._fr3_feedback_error = str(exc)
        robot.sock_cli_state_state = False
        robot.SDK_state = False
        # reconnect_flag means "reconnection in progress" to the SDK: its
        # command methods wait on it indefinitely, including stop commands.
        # Report this permanent receive failure separately; never enter that
        # loop or prevent the worker from attempting its bounded stop calls.


def install_feedback_reader(sdk):
    if hasattr(sdk, 'RobotStatePkg'):
        sdk.RPC.robot_state_routine_thread = lambda robot: receive_feedback(robot, sdk.RobotStatePkg)
