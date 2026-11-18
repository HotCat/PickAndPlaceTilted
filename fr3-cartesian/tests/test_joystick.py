"""Retained experimental HID and stroke-gate units; no application UI integration."""
import struct

import pytest

from fr3_cartesian.joystick import (Device, LinuxJoystick, StrokeGate, default_profile,
                                   normalized, sysfs_bitmap, validate_profile)


RANGES = {a: dict(min=0, max=255) for a in (0, 1, 3, 4)}
NEUTRAL = {0: 138, 1: 111, 3: 92, 4: 122}


@pytest.fixture
def profile():
    return validate_profile(default_profile() | dict(centers={str(k):v for k,v in NEUTRAL.items()}))


def test_sysfs_kernel_bitmap_word_boundaries():
    assert sysfs_bitmap('fff00000000 0 0 0 0') & 1 << 288
    assert not sysfs_bitmap('fff00000000 0 0 0 0') & 1 << 30


def test_evdev_queries_absolute_axis_capabilities_and_current_held_buttons(monkeypatch):
    closed, requests = [], []
    monkeypatch.setattr('fr3_cartesian.joystick.os.open', lambda *args: 17)
    monkeypatch.setattr('fr3_cartesian.joystick.os.close', closed.append)
    def ioctl(fd, request, data):
        assert fd == 17
        requests.append(request)
        number = request & 255
        if number == 0x23:
            data[:] = ((1 << 0) | (1 << 1) | (1 << 4) | (1 << 16)).to_bytes(8, 'little')
        elif number in (0x21, 0x18):
            data[:] = ((1 << 288) | (1 << 290)).to_bytes(96, 'little')
        elif number >= 0x40:
            code = number-0x40
            data[:] = struct.pack('iiiiii', 0 if code == 16 else 138, -1 if code == 16 else 0,
                                 1 if code == 16 else 255, 0, 15, 0)
        else:
            pytest.fail(f'Unexpected ioctl: {request:x}')
    monkeypatch.setattr('fr3_cartesian.joystick.fcntl.ioctl', ioctl)
    reader = LinuxJoystick(Device('/dev/input/offline', 'PXN', '11ff', '3331'))
    assert set(reader.ranges) == {0, 1, 4, 16}
    axes, buttons = reader.snapshot()
    assert axes[16] == 0 and buttons == {288, 290}
    assert 0x80084523 in requests  # EVIOCGBIT(EV_ABS), not event-type bits.
    reader.close()
    assert closed == [17]


def tick(gate, profile, axes=None, buttons=None, permitted=True, now=None):
    return gate.update(NEUTRAL if axes is None else axes, set() if buttons is None else buttons,
                       RANGES, profile, permitted, now)


def test_offset_center_piecewise_normalization_and_inversion():
    assert normalized(138, 0, 255, 138) == 0
    assert normalized(0, 0, 255, 138) == -1
    assert normalized(255, 0, 255, 138, True) == -1


def test_initial_held_input_and_buttons_cannot_start_or_capture(profile):
    g = StrokeGate()
    assert tick(g, profile, NEUTRAL | {0:255}, {288})[0] is None
    assert tick(g, profile, buttons={290}) == (None, False, False)
    tick(g, profile)
    demand, _, _ = tick(g, profile, NEUTRAL | {0:255}, {288})
    assert demand.axis == 1 and demand.direction == 1 and demand.speed == 3


def test_center_noise_and_activation_hysteresis_never_create_motion(profile):
    g = StrokeGate()
    tick(g, profile)
    for raw in (138, 141, 150, 156, 160, 158, 139):
        assert tick(g, profile, NEUTRAL | {0:raw}, {288})[0] is None
    assert tick(g, profile, NEUTRAL | {0:200}, {288})[0] is not None
    # Raw return into the deadzone stops immediately; there is no low-pass tail.
    assert tick(g, profile, NEUTRAL | {0:150}, {288})[0] is None


def test_dominant_axis_and_speed_latch_no_switch_or_reversal_without_center(profile):
    g = StrokeGate()
    tick(g, profile)
    demand = tick(g, profile, NEUTRAL | {0:230, 1:180}, {288})[0]
    assert demand.axis == 1
    assert tick(g, profile, NEUTRAL | {0:250, 1:240}, {288})[0] == demand
    assert tick(g, profile, NEUTRAL | {0:0}, {288})[0] is None
    assert tick(g, profile, NEUTRAL | {0:0}, {288})[0] is None
    tick(g, profile)
    assert tick(g, profile, NEUTRAL | {0:0}, {288})[0].direction == 0


def test_release_gap_and_permission_loss_require_fresh_neutral_interlock(profile):
    g = StrokeGate()
    tick(g, profile, now=0)
    assert tick(g, profile, NEUTRAL | {0:255}, {288}, now=.02)[0]
    assert tick(g, profile, NEUTRAL | {0:255}, {288}, now=.3)[0] is None
    assert tick(g, profile, buttons={288}, now=.32)[0] is None
    assert not g.ready
    tick(g, profile, now=.34)
    assert tick(g, profile, NEUTRAL | {0:255}, {288}, now=.36)[0]
    assert tick(g, profile, permitted=False, now=.38)[0] is None
    assert tick(g, profile, NEUTRAL | {0:255}, {288}, now=.4)[0] is None


def test_missing_center_is_inert_and_capture_has_no_repeat(profile):
    g = StrokeGate()
    missing = profile | dict(centers={})
    assert tick(g, missing)[0] is None and not g.ready
    tick(g, profile)
    assert tick(g, profile, buttons={290})[2]
    assert not tick(g, profile, buttons={290})[2]
    assert not tick(g, profile, buttons={288, 290})[2]


@pytest.mark.parametrize('change', [dict(axes=[0,0,None]), dict(deadman=289), dict(deadzone=float('nan')),
                                    dict(max_speed=11), dict(invert=[1,False,False])])
def test_invalid_profiles_rejected(profile, change):
    with pytest.raises(ValueError):
        validate_profile(profile | change)


def test_rest_noise_filters_drift_and_twist_selects_z(profile):
    profile=profile | dict(axes=[0,1,4])
    # Small synthetic offsets exercise drift filtering without a host HID dump.
    rest=NEUTRAL | {0:142, 1:118, 4:131}
    gate=StrokeGate()
    assert tick(gate,profile,rest)[0] is None and gate.ready
    # Throttle deflection does not participate in any target axis.
    assert tick(gate,profile,rest | {3:255}, {288})[0] is None
    demand=tick(gate,profile,rest | {4:255}, {288})[0]
    assert demand.axis==3 and demand.direction==1 and demand.speed==3
