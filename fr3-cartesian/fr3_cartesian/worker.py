"""Isolated SDK owner. Stdout is a JSON protocol; vendor prints go to stderr."""
import contextlib
from collections import deque
import ipaddress
import json
import os
from pathlib import Path
import sys
import select
import time
import xmlrpc.client
import numpy as np

from .geometry import Plane, checked_target, checked_sequence_target, check_pick_point, rotation, rpy, tcp_from_flange, vector, pose_to_base
from .messages import MoveRejected, OperatorError, OperatorValueError, error_message
from .velocity import TrackingGovernor, VelocityIntegrator, SERVO_PERIOD, INPUT_TIMEOUT
from .feedback import install_feedback_reader


class NoRetryTransport(xmlrpc.client.Transport):
    def make_connection(self, host):
        conn = super().make_connection(host)
        conn.timeout = 2.0
        return conn

    def request(self, host, handler, request_body, verbose=False):
        # The SDK retries socket errors indefinitely; surface them without replaying motion.
        try:
            return self.single_request(host, handler, request_body, verbose)
        except OSError as exc:
            raise OperatorError("Controller communication failed: {detail}", detail=str(exc)) from exc


class CommandInput:
    """Read stdin during a plane move without concurrent SDK access.

    Normal commands remain queued for the main dispatcher. Only cancellation
    matching the active arrow token can interrupt that move; EOF also releases
    it. A late cancellation can never affect another stroke or an absolute move.
    """
    def __init__(self, fd):
        self.fd = fd
        self.buffer = b''
        self.requests = deque()
        self.closed = False

    def poll(self, timeout):
        if self.closed:
            return
        readable, _, _ = select.select([self.fd], [], [], timeout)
        if readable:
            chunk = os.read(self.fd, 65536)
            if not chunk:
                self.closed = True
            self.buffer += chunk
            if len(self.buffer) > 131072 or len(self.requests) > 1024:
                raise OperatorError('Controller command input backlog exceeded its limit')
            while b'\n' in self.buffer:
                line, self.buffer = self.buffer.split(b'\n', 1)
                self.requests.append(json.loads(line))

    def cancelled(self, token):
        self.poll(0)
        cancelled = self.closed
        pending = deque()
        while self.requests:
            request = self.requests.popleft()
            if request['method'] == 'cancel_move':
                cancelled |= request.get('token') == token
            else:
                pending.append(request)
        self.requests = pending
        return cancelled


def proxy(ip):
    return xmlrpc.client.ServerProxy(f"http://{ipaddress.IPv4Address(ip)}:20003",
                                     transport=NoRetryTransport())


def check(result, name):
    code = result[0] if isinstance(result, (list, tuple)) else result
    if code != 0:
        raise OperatorError("{name}: FAIRINO error {code}", name=name, code=code)
    return result[1] if isinstance(result, (list, tuple)) and len(result) > 1 else None


def controller_flags(pkg):
    """Copy the exact readiness fields from one SDK packet for diagnostics."""
    return {key: int(getattr(pkg, key)) for key in (
        'robot_mode', 'program_state', 'robot_state', 'rbtEnableState',
        'main_code', 'sub_code', 'EmergencyStop', 'collisionState',
        'safety_stop0_state', 'safety_stop1_state')}


class Session:
    def __init__(self):
        self.robot = None
        self.sim = True
        self.tool = [0.] * 6
        self.slot = 0
        self.flange = [300., 0., 350., 180., 0., 0.]
        self.joints = [0., -70., 100., -120., -90., 0.]
        self.frame = None
        self.frame_time = 0.
        self.sim_jog = None
        self.workpiece_slot = 0
        self.workpiece_pose = [0.] * 6
        self.servo = None
        self.event_sink = None
        self.move_cancelled = lambda token: False

    def advance_sim_jog(self):
        jog = self.sim_jog
        if not jog:
            return
        elapsed = time.monotonic() - jog['started']
        rate = (90. if jog['ref'] == 0 or jog['axis'] >= 4 else 250.) * jog['speed'] / 100.
        traveled = min(jog['distance'], elapsed * rate)
        signed = traveled * (1 if jog['direction'] else -1)
        if jog['ref'] == 0:
            self.joints[jog['axis'] - 1] = jog['origin'] + signed
        elif jog['axis'] <= 3:
            self.flange[jog['axis'] - 1] = jog['origin'] + signed
        else:
            self.flange[jog['axis'] - 1] = jog['origin'] + signed
        if traveled >= jog['distance']:
            self.sim_jog = None

    def connect(self, simulation=True, ip="192.168.58.2", sdk_path=""):
        self.sim = simulation
        if not simulation:
            ipaddress.IPv4Address(ip)
            sys.path.insert(0, str(Path(sdk_path).resolve()))
            from fairino import Robot
            install_feedback_reader(Robot)
            self.robot = Robot.RPC(ip)
            print(f"Loaded FAIRINO SDK: {Robot.__file__}; {self.robot.GetSDKVersion()}", file=sys.stderr)
            self.robot.robot = proxy(ip)
            # Keep both the command and state channels from silently reconnecting.
            self.robot.reconnect = lambda: False
            if hasattr(self.robot, "SetReConnectParam"):
                self.robot.SetReConnectParam(False, 0, 1000)
            if not self.robot.sock_cli_state_state:
                raise OperatorError("Robot feedback connection failed at {ip}:20004. Check controller services and network.", ip=ip)
            if not Robot.RPC.is_conect:
                raise OperatorError("Robot command connection failed at {ip}:20003. Check controller services and network.", ip=ip)
            initial_frame = self.robot.robot_state_pkg.frame_cnt
            if not isinstance(initial_frame, int):
                raise OperatorError("No live state frames. Check SDK/firmware compatibility and port 20004")
            self.frame = initial_frame
            for _ in range(100):
                time.sleep(0.02)
                if int(self.robot.robot_state_pkg.frame_cnt) != self.frame:
                    break
            else:
                raise OperatorError("No live state frames. Check SDK/firmware compatibility and port 20004")
            self.frame_time = time.monotonic()
        return self.read()

    def read(self):
        if self.servo is not None:
            return self.servo_state()
        if self.sim:
            self.advance_sim_jog()
            return dict(flange=self.flange, tcp=tcp_from_flange(self.flange, self.tool),
                        pose_reference='robot_base', workpiece_offset=list(self.workpiece_pose),
                        joints=self.joints, tool=self.slot, user=self.workpiece_slot, errors=[0, 0],
                        tool_offset=list(self.tool), tcp_verified=self.slot != 0,
                        stopped=self.sim_jog is None, enabled=True, safe=True, mode=1, program_stopped=True)
        if getattr(self.robot, '_fr3_feedback_error', None) or self.robot.reconnect_flag:
            raise OperatorError("Robot state stream is stale; reconnect before moving")
        # Probe the command channel before taking the feedback snapshot.
        check(self.robot.GetControllerIP(), "GetControllerIP")
        pkg = self.robot.robot_state_pkg
        frame = int(pkg.frame_cnt)
        if frame != self.frame:
            self.frame, self.frame_time = frame, time.monotonic()
        self.frame_time = getattr(self.robot, '_fr3_feedback_at', self.frame_time)
        if time.monotonic() - self.frame_time > 0.8 or self.robot.reconnect_flag:
            raise OperatorError("Robot state stream is stale; reconnect before moving")
        # This controller expresses flange feedback in the applied Wobj frame.
        # Read a coherent reference around a live flange RPC (not the possibly
        # older SDK state packet), then compose it into base before calibration.
        def reference():
            rpc = self.robot.robot
            return (int(check(rpc.GetActualTCPNum(1), "GetActualTCPNum")),
                    int(check(rpc.GetActualWObjNum(1), "GetActualWObjNum")),
                    vector(check(self.robot.GetTCPOffset(), "GetTCPOffset"), 6),
                    vector(check(self.robot.GetWObjOffset(), "GetWObjOffset"), 6))
        for _ in range(2):
            before = reference()
            raw = self.robot.robot.GetActualToolFlangePose(1)
            check(raw[0], "GetActualToolFlangePose")
            local_flange = vector(raw[1:], 6)
            after = reference()
            if before[:2] == after[:2] and all(np.array_equal(a, b) for a, b in zip(before[2:], after[2:])):
                break
        else:
            raise OperatorError("Controller coordinate reference changed during feedback read; reconnect after frame selection settles")
        tool_number, workpiece_number, actual, workpiece = after
        flange = pose_to_base(local_flange, workpiece)
        verified = self.slot != 0 and tool_number == self.slot and np.allclose(actual, self.tool, atol=.001, rtol=0)
        return dict(flange=flange, tcp=tcp_from_flange(flange, actual),
                    tool_offset=actual.tolist(), tcp_verified=bool(verified),
                    pose_reference='robot_base', workpiece_offset=workpiece.tolist(),
                    joints=list(pkg.jt_cur_pos), tool=tool_number, user=workpiece_number,
                    errors=[int(pkg.main_code), int(pkg.sub_code)],
                    stopped=int(pkg.robot_state) == 1 and int(pkg.motion_done) == 1,
                    program_stopped=int(pkg.program_state) == 1,
                    enabled=bool(pkg.rbtEnableState), mode=int(pkg.robot_mode),
                    safe=not any([pkg.EmergencyStop, pkg.collisionState,
                                  pkg.safety_stop0_state, pkg.safety_stop1_state]))

    def ready(self, motion=False):
        state = self.read()
        if state['errors'] == [1, 2]:
            raise OperatorError("Controller fault 1/2: linear target point error (including tool mismatch). Review and reset the fault in WebUI, then reconnect with the calibrated tool and base workpiece 0 selected.")
        if not state['safe'] or any(state['errors']):
            raise OperatorError("Robot reports a stop, collision, or fault")
        if not state['stopped']:
            raise OperatorError("Robot must be stationary (exit drag mode before capturing)")
        if not state['program_stopped']:
            raise OperatorError("Stop the controller's Lua program before using this application")
        if motion and not state['enabled']:
            raise OperatorError("Enable the robot using the controller first")
        return state

    def capture(self):
        return self.ready()

    def servo_start(self, token, travel, speed, sent_at):
        if self.servo is not None:
            raise OperatorError('A Cartesian velocity session is already active')
        integrator = VelocityIntegrator(speed, travel)
        if not isinstance(token, str) or not token or len(token) > 64:
            raise OperatorValueError('Invalid velocity session identifier')
        if not 0 <= time.monotonic()-float(sent_at) < INPUT_TIMEOUT:
            raise MoveRejected('Joystick start expired; release and recenter before retrying')
        state = self.ready(motion=True)
        if state['mode'] != 1:
            raise OperatorError('Use controller manual mode for teaching jog')
        if state['user'] != 0 or not np.allclose(state.get('workpiece_offset', [0]*6), 0, atol=.001, rtol=0):
            raise MoveRejected('Select base workpiece 0 before simultaneous joystick XYZ motion')
        if self.sim_jog:
            raise OperatorError('Stop the current jog before starting velocity control')
        if time.monotonic()-float(sent_at) >= INPUT_TIMEOUT:
            raise MoveRejected('Joystick start expired; release and recenter before retrying')
        initial_flags = None
        if not self.sim:
            pkg = self.robot.robot_state_pkg
            # Confirm the packet pose uses the same reference as live RPC before
            # trusting it for 125 Hz servo health and following-error checks.
            if (int(pkg.tool) != state['tool'] or int(pkg.user) != 0
                    or np.linalg.norm(vector(pkg.tl_cur_pos, 6)[:3]-state['tcp'][:3]) > .1):
                raise OperatorError('Live TCP and servo feedback disagree; velocity motion was not started')
            initial_flags = controller_flags(pkg)
            if initial_flags['program_state'] != 1 or initial_flags['robot_state'] != 1:
                raise MoveRejected('Robot must be stationary with its program stopped before velocity motion')
            check(self.robot.ServoMoveStart(), 'ServoMoveStart')
        now = time.monotonic()
        self.servo = dict(token=token, integrator=integrator, state=state,
            velocity=[0.]*3, input_at=now, tick_at=now, origin=list(state['tcp']),
            target=list(state['tcp']), last_rpc_ms=0.,
            governor=TrackingGovernor(state['tcp'][:3], now),
            frame=self.frame, frame_at=now, initial_flags=initial_flags, writes=0)
        return self.servo_state()

    def servo_update(self, token, velocity, sent_at):
        stream = self.servo
        # Late updates cannot create/revive a stroke or change another stroke.
        if stream is None or stream['token'] != token:
            return
        age = time.monotonic()-float(sent_at)
        if not 0 <= age < INPUT_TIMEOUT:
            return
        value = vector(velocity, 3)
        if np.linalg.norm(value) > stream['integrator'].speed+1e-7:
            raise OperatorValueError('TCP velocity exceeds the configured maximum')
        stream['velocity'] = value.tolist()
        stream['input_at'] = float(sent_at)

    def servo_state(self):
        stream = self.servo
        state = dict(stream['state'])
        if self.sim:
            state.update(tcp=tcp_from_flange(self.flange, self.tool), flange=list(self.flange), stopped=False)
        else:
            pkg = self.robot.robot_state_pkg
            now = time.monotonic()
            frame = int(pkg.frame_cnt)
            if frame != stream['frame']:
                stream['frame'], stream['frame_at'] = frame, now
            stream['frame_at'] = getattr(self.robot, '_fr3_feedback_at', stream['frame_at'])
            if (now-stream['frame_at'] > .1 or self.robot.reconnect_flag
                    or getattr(self.robot, '_fr3_feedback_error', None)):
                raise OperatorError('Servo feedback is stale; velocity motion stopped')
            if int(pkg.tool) != state['tool'] or int(pkg.user) != 0:
                raise OperatorError('Controller tool or workpiece changed during velocity motion')
            safe = not any((pkg.EmergencyStop, pkg.collisionState, pkg.safety_stop0_state, pkg.safety_stop1_state))
            errors = [int(pkg.main_code), int(pkg.sub_code)]
            program, motion = int(pkg.program_state), int(pkg.robot_state)
            # FR3 V3.8.25-QX reports program=2/motion=2 while executing our
            # ServoCart stream. Require program=1 before starting; accept the
            # running marker only after this session has sent a servo command.
            servo_running = stream['writes'] > 0 and program == 2 and motion in (1, 2)
            if (not safe or any(errors) or not pkg.rbtEnableState or int(pkg.robot_mode) != 1
                    or motion not in (1, 2) or not (program == 1 or servo_running)):
                flags = controller_flags(pkg)
                details = ', '.join(f'{key}={value} (start={stream["initial_flags"][key]})'
                                    for key, value in flags.items())
                raise OperatorError('Controller readiness changed during velocity motion after {writes} ServoCart writes: {detail}',
                                    writes=stream['writes'], detail=details)
            tcp = vector(pkg.tl_cur_pos, 6)
            actual = tcp[:3]
            # Compare against the exact last accepted absolute target, rather
            # than assuming the controller perfectly accumulates relative jog
            # increments despite tracking lag.
            expected = np.array(stream['target'][:3])
            delta = actual-expected
            following_error = float(np.linalg.norm(delta))
            if following_error > 1.:
                raise OperatorError(
                    'Servo TCP following error {error:.3f} mm exceeds 1 mm after {writes} absolute ServoCart writes; '
                    'actual XYZ [{actual}], target XYZ [{target}], delta XYZ [{delta}], '
                    'feedback age {age:.1f} ms, last RPC {rpc:.1f} ms, '
                    'commanded speed {speed:.2f} mm/s, travel {distance:.2f}/{limit:.2f} mm. Velocity motion stopped',
                    error=following_error, writes=stream['writes'],
                    actual=', '.join(f'{v:.3f}' for v in actual),
                    target=', '.join(f'{v:.3f}' for v in expected),
                    delta=', '.join(f'{v:.3f}' for v in delta),
                    age=max(0., now-stream['frame_at'])*1000, rpc=stream['last_rpc_ms'],
                    speed=float(np.linalg.norm(stream['integrator'].velocity)),
                    distance=stream['integrator'].distance, limit=stream['integrator'].travel)
            if np.max(np.abs(rotation(tcp[3:])-rotation(stream['origin'][3:]))) > .02:
                raise OperatorError('Tool orientation changed during XYZ velocity motion')
            if np.linalg.norm(actual-stream['origin'][:3]) > stream['integrator'].travel+.25:
                raise OperatorError('Measured TCP exceeds the joystick travel limit')
            offset = state['tool_offset']
            flange_r = rotation(tcp[3:]) @ rotation(offset[3:]).T
            flange = [*(actual-flange_r @ np.array(offset[:3])).tolist(), *rpy(flange_r)]
            state.update(tcp=tcp.tolist(), flange=flange, joints=list(pkg.jt_cur_pos), stopped=False,
                         program_stopped=program == 1, servo_program_running=servo_running)
        state.update(servo_active=True, servo_distance=stream['integrator'].distance,
                     servo_limit=stream['integrator'].travel,
                     servo_tracking_error=float(np.linalg.norm(np.array(state['tcp'][:3])-stream['target'][:3])),
                     servo_tracking_scale=stream['governor'].scale,
                     servo_velocity=stream['integrator'].velocity.tolist())
        return state

    def servo_tick(self, now=None):
        stream = self.servo
        if stream is None:
            return None
        now = time.monotonic() if now is None else now
        if now-stream['input_at'] >= INPUT_TIMEOUT:
            return self.servo_end(stream['token'], 'input_timeout')
        if now-stream['tick_at'] > .04:
            return self.servo_end(stream['token'], 'timing_delay')
        feedback = self.servo_state()  # Live controller checks before each increment.
        stream['tick_at'] = now
        # Speed is a ceiling: do not let the software encoder outrun the
        # measured TCP. Keep discarded demand out of the path budget/backlog.
        requested = stream['governor'].limit(stream['velocity'], feedback['tcp'][:3],
                                             stream['target'][:3], now)
        increment = stream['integrator'].step(requested)
        target = [*(np.array(stream['origin'][:3])+stream['integrator'].position).tolist(),
                  *stream['origin'][3:]]
        if self.sim:
            self.flange[:3] = (np.array(self.flange[:3])+increment).tolist()
        else:
            # Absolute TCP target in base Wobj0, retaining the captured tool
            # attitude. The software encoder integrates mm/s at cmdT. Relative
            # mode must not be mixed with an accumulated absolute target check.
            rpc_started = time.monotonic()
            check(self.robot.ServoCart(0, target, cmdT=SERVO_PERIOD), 'ServoCart')
            stream['last_rpc_ms'] = (time.monotonic()-rpc_started)*1000
        stream['target'] = target
        stream['writes'] += 1
        if stream['integrator'].exhausted:
            return self.servo_end(stream['token'], 'travel_limit')
        return None

    def servo_end(self, token, reason='released'):
        if self.servo is None or self.servo['token'] != token:
            return None
        if self.event_sink is not None:
            self.event_sink(dict(event='servo_stopping', token=token))
        stream, self.servo = self.servo, None
        if not self.sim:
            # Both attempted once, even if the first fails. No motion replay.
            errors = []
            for name in ('StopMotion', 'ServoMoveEnd'):
                try:
                    check(getattr(self.robot, name)(), name)
                except Exception as exc:
                    errors.append(str(exc))
            if errors:
                raise OperatorError('Velocity stop was not confirmed: {detail}', detail='; '.join(errors))
        state = self.read()
        if not self.sim:
            # StopMotion/ServoMoveEnd can acknowledge before feedback leaves the
            # servo-running marker. Do not publish that transitional packet as
            # an unrelated Lua program or permit another start during settling.
            deadline = time.monotonic()+1.
            while not (state['stopped'] and state['program_stopped']):
                if not state['safe'] or any(state['errors']):
                    break
                if time.monotonic() >= deadline:
                    raise OperatorError('Velocity stop did not settle to stationary/program stopped; reconnect before moving')
                time.sleep(.008)
                state = self.read()
        return dict(state, servo_active=False, servo_reason=reason,
                    servo_distance=stream['integrator'].distance, servo_limit=stream['integrator'].travel)

    def select_base_frame(self):
        self.ready()
        if self.sim:
            self.workpiece_slot, self.workpiece_pose = 0, [0.] * 6
            return self.read()
        # Explicit operator action: apply the identity base frame in slot 0.
        # Leave all taught Wobj slots (1–14) intact. Never start motion here.
        check(self.robot.SetWObjCoord(0, [0.] * 6, 0), "SetWObjCoord")
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline:
            state = self.read()
            offset = vector(check(self.robot.GetWObjOffset(), "GetWObjOffset"), 6)
            if state['user'] == 0 and np.allclose(offset, 0, atol=.001, rtol=0):
                return state
            time.sleep(.03)
        raise OperatorError("Controller did not confirm base workpiece 0 with an identity offset")

    def apply_plane(self, plane, slot):
        state = self.ready()
        if plane.get('reference') != 'robot_base':
            raise OperatorError("This plane was captured with the old coordinate-frame bug. Load a recovered base-frame calibration or reteach the plane before using it.")
        if type(slot) is not int or not 1 <= slot <= 14:
            raise OperatorValueError("Controller workpiece slot must be between 1 and 14")
        if not self.slot or state['tool'] != self.slot or not state.get('tcp_verified'):
            raise OperatorError("Verify the calibrated TCP before applying a workpiece plane")
        frame = Plane.teach(plane['name'], plane['points'])
        pose = frame.pose()
        if self.sim:
            self.workpiece_slot, self.workpiece_pose = slot, pose
            state = self.read()
        else:
            # An explicit operator-approved replacement of exactly one taught
            # slot, referenced to robot base. Do not issue any motion or retry.
            check(self.robot.SetWObjList(slot, pose, 0), "SetWObjList")
            check(self.robot.SetWObjCoord(slot, pose, 0), "SetWObjCoord")
            deadline = time.monotonic() + 2
            while time.monotonic() < deadline:
                state = self.read()
                actual = vector(check(self.robot.GetWObjOffset(), "GetWObjOffset"), 6)
                if (state['user'] == slot and np.allclose(actual[:3], pose[:3], atol=.001, rtol=0)
                        and np.allclose(rotation(actual[3:]), rotation(pose[3:]), atol=2e-5, rtol=0)):
                    break
                time.sleep(.03)
            else:
                raise OperatorError("Controller did not confirm the applied workpiece slot and plane pose")
        return dict(state, applied_workpiece=dict(name=frame.name, slot=slot, pose=pose))

    def verify_tool(self, tool, slot):
        tool = vector(tool, 6).tolist()
        if not 1 <= slot <= 14:
            raise OperatorValueError("Invalid tool slot, physical tool ID, or load number")
        state = self.read()
        matched = state['tool'] == slot and np.allclose(state['tool_offset'], tool, atol=.001, rtol=0)
        if matched:
            self.tool, self.slot = tool, slot
        else:
            self.slot = 0
        state['tcp_verified'] = bool(matched)
        return state

    def apply_tool(self, tool, slot, tool_id, load):
        self.ready()
        tool = vector(tool, 6).tolist()
        if not 1 <= slot <= 14 or not 0 <= load <= 99 or not 0 <= tool_id <= 14:
            raise OperatorValueError("Invalid tool slot, physical tool ID, or load number")
        if not self.sim:
            check(self.robot.SetToolList(slot, tool, 0, 0, load), "SetToolList")
            check(self.robot.SetToolCoord(slot, tool, 0, 0, tool_id, load), "SetToolCoord")
        self.tool, self.slot = tool, slot
        if not self.sim:
            deadline = time.monotonic()+2
            while time.monotonic() < deadline:
                state = self.read()
                actual = vector(check(self.robot.GetTCPOffset(), "GetTCPOffset"), 6)
                if state['tool'] == slot and np.allclose(actual, tool, atol=.001, rtol=0):
                    return state
                time.sleep(.03)
            raise OperatorError('Controller did not confirm the applied TCP slot and offset')
        return self.read()

    def start_jog(self, ref, axis, direction, distance, speed):
        state = self.ready(motion=True)
        if ref not in (0, 2) or not 1 <= axis <= 6 or direction not in (0, 1):
            raise OperatorValueError("Invalid jog reference, axis, or direction")
        maximum = 10 if ref == 0 or axis >= 4 else 50
        if not 0 < distance <= maximum or not 0 < speed <= 10:
            raise OperatorValueError("Jog travel is out of range or speed exceeds 10%")
        if state['mode'] != 1:
            raise OperatorError("Use controller manual mode for teaching jog")
        if self.sim:
            origin = self.joints[axis - 1] if ref == 0 else self.flange[axis - 1]
            self.sim_jog = dict(ref=ref, axis=axis, direction=direction, distance=distance,
                                speed=speed, origin=origin, started=time.monotonic())
        else:
            check(self.robot.StartJOG(ref, axis, direction, distance, vel=speed, acc=10), "StartJOG")
        return self.read()

    def stop_jog(self, stop_ref):
        if stop_ref not in (1, 3):
            raise OperatorValueError("Invalid jog stop reference")
        if self.sim:
            self.advance_sim_jog()
            self.sim_jog = None
        else:
            check(self.robot.StopJOG(stop_ref), "StopJOG")
        return self.read()

    def move(self, plane, target, orientation, bounds, speed, jog_token=None, sequence=None):
        state = self.ready(motion=True)
        if plane.get('reference') != 'robot_base':
            raise MoveRejected("This plane was captured with the old coordinate-frame bug. Load a recovered base-frame calibration or reteach the plane before using it.")
        if self.slot == 0:
            raise MoveRejected("Apply a calibrated tool before moving")
        if not 0 < speed <= 10:
            raise MoveRejected("Cartesian speed must be in (0, 10]%")
        if state['tool'] != self.slot:
            raise MoveRejected("Active controller tool {actual} differs from calibrated tool {expected}. Apply the saved TCP before moving.",
                               actual=state['tool'], expected=self.slot)
        if state['user'] != 0:
            raise MoveRejected("Select base workpiece 0 in the controller before Cartesian motion (currently {actual}). The controller frame must match the base-frame target.", actual=state['user'])
        if not self.sim:
            actual = vector(check(self.robot.GetTCPOffset(), "GetTCPOffset"), 6)
            if not np.allclose(actual, self.tool, atol=0.001, rtol=0):
                raise MoveRejected("Controller tool offset changed; apply calibration again")
            workpiece = vector(check(self.robot.GetWObjOffset(), "GetWObjOffset"), 6)
            if not np.allclose(workpiece, 0, atol=.001, rtol=0):
                raise MoveRejected("Base workpiece 0 has a nonzero offset. Click “Use base workpiece 0” before moving.")
        try:
            frame = Plane.teach(plane['name'], plane['points'])
            if sequence not in (None, 'pick', 'transfer') or (sequence is not None and jog_token is not None):
                raise OperatorValueError('Invalid sequence motion request')
            if sequence is None:
                pose = checked_target(frame, state['tcp'], target, orientation, bounds)
            else:
                # A recipe explicitly teaches full base TCP poses. Retain those
                # orientations and check the pickup box only on source moves.
                # All readiness/tool/frame checks above still apply, and the
                # distance cap is rechecked against actual feedback per move.
                if sequence == 'pick':
                    check_pick_point(target, bounds)
                pose = checked_sequence_target(state['tcp'], [*frame.to_base(target), *vector(orientation, 3)])
        except OperatorValueError as exc:
            raise MoveRejected(exc.message.source, **exc.message.values) from exc
        if jog_token is not None and self.move_cancelled(jog_token):
            # Preflight may include several RPC reads. Check release again at
            # the dispatch boundary, before any motion can reach the controller.
            return dict(self.read(), plane_jog_stopped=True)
        if self.sim:
            if jog_token is None:
                self.flange = [*(np.array(pose[:3])-rotation(pose[3:]) @ np.array(self.tool[:3])), *pose[3:]]
                time.sleep(0.12)
            else:
                # Advance progressively so release tests can observe a partial
                # move, rather than teleporting to the target before sleeping.
                origin = np.array(state['tcp'][:3])
                delta = np.array(pose[:3])-origin
                duration = max(.12, np.linalg.norm(delta)/(250.*speed/100.))
                started = time.monotonic()
                while True:
                    fraction = min(1., (time.monotonic()-started)/duration)
                    tip = origin + fraction*delta
                    self.flange = [*(tip-rotation(pose[3:]) @ np.array(self.tool[:3])), *pose[3:]]
                    if self.move_cancelled(jog_token):
                        return dict(self.read(), plane_jog_stopped=True)
                    if fraction >= 1:
                        break
                    time.sleep(.01)
        else:
            jog_options = {}
            if jog_token is not None:
                # The SDK normally performs IK inside MoveL. Resolve it here
                # so a release arriving during that RPC is checked before the
                # motion write, instead of starting an already-released stroke.
                joints = vector(check(self.robot.GetInverseKin(0, pose), 'GetInverseKin'), 6).tolist()
                if self.move_cancelled(jog_token):
                    return dict(self.read(), plane_jog_stopped=True)
                jog_options['joint_pos'] = joints
            # The target and IK type 0 use robot base coordinates. MoveL's user=0
            # must agree with the controller's selected base workpiece.
            # blendR=0 returns after accepting the move. Do not submit
            # another point until live feedback confirms the target and stationary state.
            result = self.robot.MoveL(pose, tool=self.slot, user=0, vel=speed, ovl=100,
                                      blendR=0, offset_flag=0, **jog_options)
            code = result[0] if isinstance(result, (list, tuple)) else result
            if code == 74:
                print(f"MoveL point rejection: target_base={pose}; tool={self.slot}; user=0", file=sys.stderr)
                raise OperatorError("MoveL: FAIRINO error 74 — linear target point error (including tool mismatch). Check the target, selected tool and base workpiece 0. Review and reset the controller fault in WebUI before trying again.")
            check(result, "MoveL")
            deadline = time.monotonic()+15
            cancelled = False
            while time.monotonic() < deadline:
                if jog_token is not None and not cancelled and self.move_cancelled(jog_token):
                    # Normal arrow release uses StopMotion and retains this
                    # calibrated session only after fresh stationary feedback.
                    # A failed stop/read takes the existing stop/disconnect path.
                    check(self.robot.StopMotion(), 'StopMotion')
                    cancelled = True
                    deadline = min(deadline, time.monotonic()+1.5)
                feedback = self.read()
                if not feedback['safe'] or any(feedback['errors']) or not feedback['enabled']:
                    raise OperatorError('Controller stopped or faulted during motion')
                if feedback['stopped']:
                    if cancelled:
                        if feedback['program_stopped']:
                            return dict(feedback, plane_jog_stopped=True)
                    elif (np.linalg.norm(np.array(feedback['tcp'][:3])-pose[:3]) < 0.02
                          and (sequence is None or np.allclose(rotation(feedback['tcp'][3:]), rotation(pose[3:]), atol=.001, rtol=0))):
                        return feedback
                time.sleep(0.03)
            if cancelled:
                raise OperatorError('Plane jog stop was not confirmed by stationary feedback')
            raise OperatorError('Motion did not reach the target within 15 seconds')
        return self.read()

    def move_spline(self, plane, poses, bounds, speed, blend_radius=10.):
        """Execute one reviewed transit leg as a controller arc-transition spline.

        The caller batches only transit shaping points and one exact-stop pick
        or place endpoint. FAIRINO's NewSplinePoint ``lastFlag`` marks that
        endpoint; non-final points use a modest blend radius. The controller
        owns the IK interpolation between the taught TCP poses.
        """
        state = self.ready(motion=True)
        if plane.get('reference') != 'robot_base':
            raise MoveRejected("This plane was captured with the old coordinate-frame bug. Load a recovered base-frame calibration or reteach the plane before using it.")
        if self.slot == 0:
            raise MoveRejected("Apply a calibrated tool before moving")
        if not 0 < speed <= 10:
            raise MoveRejected("Cartesian speed must be in (0, 10]%")
        if state['tool'] != self.slot:
            raise MoveRejected("Active controller tool {actual} differs from calibrated tool {expected}. Apply the saved TCP before moving.",
                               actual=state['tool'], expected=self.slot)
        if state['user'] != 0:
            raise MoveRejected("Select base workpiece 0 in the controller before Cartesian motion (currently {actual}). The controller frame must match the base-frame target.", actual=state['user'])
        if not self.sim:
            actual = vector(check(self.robot.GetTCPOffset(), "GetTCPOffset"), 6)
            if not np.allclose(actual, self.tool, atol=.001, rtol=0):
                raise MoveRejected("Controller tool offset changed; apply calibration again")
            workpiece = vector(check(self.robot.GetWObjOffset(), "GetWObjOffset"), 6)
            if not np.allclose(workpiece, 0, atol=.001, rtol=0):
                raise MoveRejected("Base workpiece 0 has a nonzero offset. Click “Use base workpiece 0” before moving.")
        if not isinstance(poses, list) or not poses:
            raise OperatorValueError('A transit spline needs at least one taught point and a final pick/place point')
        blend_radius = float(blend_radius)
        if not np.isfinite(blend_radius) or not 0 <= blend_radius <= 25:
            raise OperatorValueError('Transit spline blend radius must be between 0 and 25 mm')
        checked, previous = [], state['tcp']
        for pose in poses:
            target_pose = checked_sequence_target(previous, pose)
            checked.append(target_pose)
            previous = target_pose
        if self.sim:
            final = checked[-1]
            self.flange = [*(np.array(final[:3])-rotation(final[3:]) @ np.array(self.tool[:3])), *final[3:]]
            time.sleep(.12)
            return self.read()

        # Solve IK as a chain: each via point uses the preceding solution as
        # its reference. Solving every point against the same live joints can
        # switch wrist/elbow branches even when adjacent TCP poses are close.
        reference_joints = vector(state['joints'], 6).tolist()
        planned_joints = []
        for pose in checked:
            joints = vector(check(self.robot.GetInverseKinRef(0, pose, reference_joints),
                                  'GetInverseKinRef'), 6).tolist()
            planned_joints.append(joints)
            reference_joints = joints
        # Do not open a controller spline context until every waypoint has a
        # validated, branch-continuous IK solution.
        check(self.robot.NewSplineStart(0, 2000), 'NewSplineStart')
        for index, (pose, joints) in enumerate(zip(checked, planned_joints)):
            last = index == len(checked)-1
            result = self.robot.NewSplinePoint(
                pose, self.slot, 0, int(last), joint_pos=joints,
                # FAIRINO documents spline vel/acc as not open; leave both at
                # their supported defaults. Apply the UI's conservative speed
                # percentage through the documented spline override instead.
                vel=0, acc=0, ovl=speed, blendR=0 if last else blend_radius)
            check(result, 'NewSplinePoint')
        result = self.robot.NewSplineEnd()
        code = result[0] if isinstance(result, (list, tuple)) else result
        if code == 14:
            plan = '; '.join(f'P{index+1} xyz=[{pose[0]:.1f},{pose[1]:.1f},{pose[2]:.1f}] '
                             f'rpy=[{pose[3]:.1f},{pose[4]:.1f},{pose[5]:.1f}] '
                             f'joints=[{",".join(f"{value:.1f}" for value in planned_joints[index])}]'
                             for index, pose in enumerate(checked))
            raise OperatorError('Transit spline rejected by the controller (FAIRINO error 14: Cartesian instruction speed exceeded; override={speed:g}%). Review/reset the controller alarm in WebUI. Planned poses/continuous IK: {plan}',
                                speed=speed, plan=plan)
        check(result, 'NewSplineEnd')

        deadline = time.monotonic()+60
        while time.monotonic() < deadline:
            feedback = self.read()
            if not feedback['safe'] or any(feedback['errors']) or not feedback['enabled']:
                raise OperatorError('Controller stopped or faulted during the transit spline')
            if feedback['stopped']:
                position_error = float(np.linalg.norm(np.array(feedback['tcp'][:3])-checked[-1][:3]))
                orientation_error = float(np.linalg.norm(rotation(feedback['tcp'][3:])-rotation(checked[-1][3:])))
                if position_error <= .02 and orientation_error <= .001:
                    return feedback
                raise OperatorError('Transit spline stopped before its final pick/place endpoint')
            time.sleep(.03)
        raise OperatorError('Transit spline did not reach its final pick/place endpoint within 60 seconds')

    def demo_pose(self, kind, index):
        if not self.sim:
            raise OperatorError("Demo poses are only available in simulation")
        if kind == 'pivot':
            angles = [[180, 0, 0], [150, 25, 0], [-155, -20, 30], [175, 35, -35], [145, -25, 20], [-160, 15, -20]]
            attitude = angles[index % len(angles)]
            position = np.array([300., 0., 200.])-rotation(attitude) @ np.array([0., 0., 120.])
            self.flange = [*position.tolist(), *attitude]
        else:
            tip = [[300., 0., 200.], [400., 20., 200.], [280., 100., 210.]][index]
            self.flange = [*(np.array(tip)-rotation([180, 0, 0]) @ np.array(self.tool[:3])), 180., 0., 0.]
        return self.read()


def stop_robot(ip, servo=False):
    # Independent of the SDK owner, including a blocked MoveL. Never retry a command.
    errors = []
    for method in (('ImmStopJOG', 'StopMotion', 'ServoMoveEnd') if servo else ('ImmStopJOG', 'StopMotion')):
        try:
            with proxy(ip) as rpc:
                check(getattr(rpc, method)(), method)
        except Exception as exc:
            errors.append(str(exc))
    if errors:
        raise OperatorError('; '.join(errors))


def main():
    if len(sys.argv) == 3 and sys.argv[1] in ('--stop', '--stop-servo'):
        try:
            stop_robot(sys.argv[2], servo=sys.argv[1] == '--stop-servo')
        except Exception as exc:
            print(str(exc), file=sys.stderr)
            sys.exit(1)
        return
    session = Session()
    allowed = {'connect', 'read', 'capture', 'select_base_frame', 'apply_plane', 'apply_tool', 'verify_tool', 'start_jog', 'stop_jog', 'move', 'move_spline', 'demo_pose', 'servo_start', 'servo_end'}
    inbox = CommandInput(sys.stdin.fileno())
    session.move_cancelled = inbox.cancelled
    next_tick, next_state = 0., 0.
    protocol_output = sys.stdout

    def reply(data):
        print(json.dumps(data), file=protocol_output, flush=True)

    session.event_sink = reply

    def fail_stream(exc):
        if session.servo:
            try:
                with contextlib.redirect_stdout(sys.stderr):
                    session.servo_end(session.servo['token'], 'fault')
            except Exception as stop_error:
                print(str(stop_error), file=sys.stderr)
        reply(dict(event='servo_error', error=error_message(exc).payload()))

    try:
        while True:
            timeout = max(0., next_tick-time.monotonic()) if session.servo else None
            inbox.poll(0 if inbox.requests else timeout)
            if inbox.requests:
                while inbox.requests:
                    request = inbox.requests.popleft()
                    if request['method'] == 'cancel_move':
                        continue  # An already-completed stroke needs no stop.
                    if request['method'] == 'servo_update':
                        try:
                            session.servo_update(**request.get('args', {}))
                        except Exception as exc:
                            fail_stream(exc)
                        continue
                    try:
                        if request['method'] not in allowed:
                            raise OperatorValueError('Unknown command')
                        if session.servo and request['method'] not in {'servo_end', 'read'}:
                            raise OperatorError('Finish joystick velocity motion before another controller command')
                        with contextlib.redirect_stdout(sys.stderr):
                            result = getattr(session, request['method'])(**request.get('args', {}))
                        reply(dict(id=request['id'], result=result))
                        if request['method'] == 'servo_start':
                            next_tick = time.monotonic()+SERVO_PERIOD
                            next_state = time.monotonic()+.1
                    except Exception as exc:
                        if session.servo:
                            fail_stream(exc)
                        reply(dict(id=request['id'], error=error_message(exc).payload(), recoverable=isinstance(exc, MoveRejected)))
            if inbox.closed:
                break
            now = time.monotonic()
            if session.servo and now >= next_tick:
                try:
                    with contextlib.redirect_stdout(sys.stderr):
                        ended = session.servo_tick(now)
                    # Never catch up missed cycles with a burst of increments.
                    next_tick = max(now+SERVO_PERIOD, time.monotonic())
                    if ended is not None:
                        reply(dict(event='servo_end', result=ended))
                    elif now >= next_state:
                        with contextlib.redirect_stdout(sys.stderr):
                            state = session.servo_state()
                        reply(dict(event='servo_state', result=state))
                        next_state = time.monotonic()+.1
                except Exception as exc:
                    fail_stream(exc)
    finally:
        if session.servo:
            with contextlib.redirect_stdout(sys.stderr):
                session.servo_end(session.servo['token'], 'input_closed')


if __name__ == '__main__':
    main()
