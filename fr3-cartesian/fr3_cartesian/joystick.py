"""Linux evdev snapshots and deterministic pendant input rules; no robot imports."""
from dataclasses import dataclass
import fcntl
import math
import os
from pathlib import Path
import struct
import time

from .messages import OperatorValueError


AXES = {0: 'ABS_X', 1: 'ABS_Y', 2: 'ABS_Z', 3: 'ABS_RX', 4: 'ABS_RY',
        5: 'ABS_RZ', 6: 'ABS_THROTTLE', 7: 'ABS_RUDDER', 8: 'ABS_WHEEL',
        16: 'ABS_HAT0X', 17: 'ABS_HAT0Y'}


def ioctl_read(fd, number, size):
    data = bytearray(size)
    fcntl.ioctl(fd, 0x80000000 | size << 16 | ord('E') << 8 | number, data)
    return data


def bits(data):
    return {i for i in range(len(data)*8) if data[i//8] & 1 << (i % 8)}


def sysfs_bitmap(text):
    return sum(int(word, 16) << (struct.calcsize('L')*8*i)
               for i, word in enumerate(reversed(text.split())))


@dataclass(frozen=True)
class Device:
    path: str
    name: str
    vendor: str
    product: str

    @property
    def identity(self):
        return f'{self.vendor}:{self.product}:{self.name}'


def devices(root=Path('/sys/class/input')):
    result = []
    for path in sorted(root.glob('event*')):
        try:
            dev = path/'device'
            # Only joystick/gamepad-class devices, never keyboard input.
            keys = sysfs_bitmap((dev/'capabilities/key').read_text())
            abs_bits = sysfs_bitmap((dev/'capabilities/abs').read_text())
            if not abs_bits & 3 == 3 or not any(keys & 1 << b for b in range(288, 320)):
                continue
            result.append(Device('/dev/input/'+path.name, (dev/'name').read_text().strip(),
                                 (dev/'id/vendor').read_text().strip(), (dev/'id/product').read_text().strip()))
        except (OSError, ValueError):
            continue
    return result


class LinuxJoystick:
    """Read current kernel state, including held keys at open and SYN_DROPPED recovery."""
    def __init__(self, device):
        self.device = device
        self.fd = os.open(device.path, os.O_RDONLY | os.O_NONBLOCK | os.O_CLOEXEC)
        try:
            codes = bits(ioctl_read(self.fd, 0x23, 8))
            self.ranges = {}
            for code in sorted(codes):
                value, low, high, fuzz, flat, resolution = struct.unpack('iiiiii', ioctl_read(self.fd, 0x40+code, 24))
                if high > low:
                    self.ranges[code] = dict(min=low, max=high, flat=flat, value=value)
            self.buttons = sorted(b for b in bits(ioctl_read(self.fd, 0x21, 96)) if 288 <= b < 768)
            if not {0, 1} <= self.ranges.keys() or not self.buttons:
                raise OperatorValueError('Selected device has no joystick axes/buttons')
        except Exception:
            self.close()
            raise

    def snapshot(self):
        # ioctls read current state rather than replaying queued input history.
        axes = {code: struct.unpack('iiiiii', ioctl_read(self.fd, 0x40+code, 24))[0]
                for code in self.ranges}
        buttons = bits(ioctl_read(self.fd, 0x18, 96))
        return axes, buttons

    def close(self):
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None


def default_profile():
    return dict(schema=1, device='11ff:3331:PC Game Controller', motion_mode=0,
                axes=[0, 1, None], invert=[False, True, False], centers={},
                deadzone=.15, hysteresis=.04, response=1.5,
                deadman=288, stop=289, capture=290, max_speed=3.,
                tcp_speed=10., throttle_axis=None, throttle_invert=False)


def validate_profile(data):
    d = default_profile() | data
    if d['schema'] != 1 or not isinstance(d['device'], str):
        raise OperatorValueError('Invalid joystick profile')
    if type(d['motion_mode']) is not int or not 0 <= d['motion_mode'] <= 4:
        raise OperatorValueError('Invalid joystick motion mode')
    if len(d['axes']) != 3 or len(d['invert']) != 3 or any(type(v) is not bool for v in d['invert']):
        raise OperatorValueError('Invalid joystick mapping')
    assigned = [v for v in d['axes'] if v is not None]
    throttle = d['throttle_axis']
    if throttle is not None:
        assigned.append(throttle)
    if type(d['throttle_invert']) is not bool:
        raise OperatorValueError('Invalid joystick mapping')
    if any(type(v) is not int or not 0 <= v < 64 for v in assigned) or len(set(assigned)) != len(assigned):
        raise OperatorValueError('Map each input axis only once')
    for key in ('deadman', 'stop', 'capture'):
        if d[key] is not None and (type(d[key]) is not int or not 288 <= d[key] < 768):
            raise OperatorValueError('Invalid joystick button')
    buttons = [d[k] for k in ('deadman', 'stop', 'capture') if d[k] is not None]
    if d['deadman'] is None or len(set(buttons)) != len(buttons):
        raise OperatorValueError('Deadman is required; map each button only once')
    for key, low, high in [('deadzone', .03, .4), ('hysteresis', .01, .1),
                           ('response', 1., 3.), ('max_speed', 1., 10.), ('tcp_speed', .1, 25.)]:
        value = float(d[key])
        if not math.isfinite(value) or not low <= value <= high:
            raise OperatorValueError('Invalid joystick filter or speed')
        d[key] = value
    d['centers'] = {str(int(k)): float(v) for k, v in d['centers'].items()}
    if any(not 0 <= int(k) < 64 or not math.isfinite(v) for k, v in d['centers'].items()):
        raise OperatorValueError('Invalid joystick center')
    return d


def normalized(raw, low, high, center, invert=False):
    if not low < center < high:
        raise OperatorValueError('Joystick center must be inside its axis range')
    value = (raw-center) / ((high-center) if raw >= center else (center-low))
    return max(-1., min(1., -value if invert else value))


@dataclass(frozen=True)
class Demand:
    axis: int  # target axis 1..3; mode chooses J1..3, J4..6, XYZ, RX/RY/RZ
    direction: int
    speed: float


@dataclass(frozen=True)
class VectorDemand:
    velocity: tuple


def throttle_fraction(axes, ranges, profile):
    code = profile['throttle_axis']
    if code is None:
        return 1.
    if code not in axes or code not in ranges:
        raise OperatorValueError('Configured throttle axis is unavailable')
    r = ranges[code]
    value = max(0., min(1., (axes[code]-r['min'])/(r['max']-r['min'])))
    if profile['throttle_invert']:
        value = 1-value
    # Endpoint margin gives a stable true zero and full scale.
    return max(0., min(1., (value-.02)/.96))


class StrokeGate:
    """A neutral/released interlock, dominant-axis selection and latched jog speed.

    Losing permission or a scheduling gap always requires neutral plus released
    deadman again. Returning into the raw deadzone stops without a filter tail.
    A travel-limited jog is never automatically reissued while the stick is held.
    """
    def __init__(self):
        self.reset()

    def reset(self):
        self.ready = False
        self.active = None
        self.consumed = False
        self.previous_buttons = set()
        self.last_time = None

    def update(self, axes, buttons, ranges, profile, permitted, now=None, vector_mode=False):
        now = time.monotonic() if now is None else now
        gap = self.last_time is not None and now-self.last_time > .15
        self.last_time = now
        edges = buttons-self.previous_buttons
        self.previous_buttons = set(buttons)
        stop = profile['stop'] in buttons
        capture = profile['capture'] in edges
        values = []
        for code, invert in zip(profile['axes'], profile['invert']):
            if code is None:
                values.append(0.)
                continue
            if code not in axes or code not in ranges or str(code) not in profile['centers']:
                self.ready, self.active = False, None
                return None, stop, False
            r = ranges[code]
            values.append(normalized(axes[code], r['min'], r['max'], profile['centers'][str(code)], invert))
        neutral = all(abs(v) <= profile['deadzone'] for v in values)
        held = profile['deadman'] in buttons
        if not permitted or gap or stop:
            self.ready, self.active, self.consumed = False, None, False
            return None, stop, False
        if neutral:
            if vector_mode and held and self.active is not None:
                # Zero velocity while centered; keep the same held-trigger
                # session and path budget when steering between XYZ axes.
                self.active = VectorDemand((0., 0., 0.))
                return self.active, stop, False
            was_ready = self.ready
            self.active, self.consumed = None, False
            if not buttons:
                self.ready = True
            return None, stop, capture and was_ready and not held
        if not held:
            self.consumed = self.consumed or self.active is not None
            self.active = None
            return None, stop, False
        if not self.ready:
            return None, stop, False
        if vector_mode:
            if self.consumed and self.active is None:
                return None, stop, False
            if self.active is None and max(abs(v) for v in values) < profile['deadzone']+profile['hysteresis']:
                return None, stop, False
            scaled = [math.copysign(max(0., (abs(v)-profile['deadzone'])/(1-profile['deadzone'])), v) for v in values]
            length = math.sqrt(sum(v*v for v in scaled))
            maximum = profile['tcp_speed']*throttle_fraction(axes, ranges, profile)
            self.active = VectorDemand(tuple(v/max(1.,length)*maximum for v in scaled))
            self.consumed = True
            return self.active, stop, False
        if self.active:
            value = values[self.active.axis-1]
            if abs(value) <= profile['deadzone'] or (value > 0) != bool(self.active.direction):
                self.active, self.consumed = None, True
            return self.active, stop, False
        if self.consumed:
            return None, stop, False
        index = max(range(3), key=lambda i: abs(values[i]))
        amplitude = abs(values[index])
        if amplitude < profile['deadzone']+profile['hysteresis']:
            return None, stop, False
        fraction = (amplitude-profile['deadzone'])/(1-profile['deadzone'])
        speed = max(1., profile['max_speed'] * fraction**profile['response'])
        self.active = Demand(index+1, int(values[index] > 0), speed)
        self.consumed = True
        return self.active, stop, False
