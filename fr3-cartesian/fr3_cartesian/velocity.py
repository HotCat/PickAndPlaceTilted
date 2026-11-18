"""Bounded Cartesian velocity integration, independent of GUI and robot transport."""
import math
import numpy as np
from .geometry import vector
from .messages import OperatorError, OperatorValueError

SERVO_PERIOD = .008
INPUT_TIMEOUT = .15
MAX_TCP_SPEED = 25.  # mm/s, commissioning ceiling


class TrackingGovernor:
    """Slow the trajectory before the independent 1 mm hard tracking stop.

    Unserved velocity is discarded, never accumulated as catch-up motion.
    A stopped plant must still fail even though the target lead is bounded.
    """
    def __init__(self, actual, now):
        self.progress_position = vector(actual, 3)
        self.progress_at = now
        self.scale = 1.

    def limit(self, requested, actual, target, now):
        requested, actual, target = (vector(v, 3) for v in (requested, actual, target))
        error = float(np.linalg.norm(target-actual))
        self.scale = float(np.clip((.6-error)/.4, 0., 1.))
        if (np.linalg.norm(requested) <= .2 or error <= .2
                or np.linalg.norm(actual-self.progress_position) >= .05):
            self.progress_position = actual.copy()
            self.progress_at = now
        elif now-self.progress_at >= .75:
            raise OperatorError(
                'Servo TCP made less than 0.05 mm progress for {elapsed:.2f} s while motion was requested; '
                'tracking error {error:.3f} mm. Velocity motion stopped',
                elapsed=now-self.progress_at, error=error)
        return requested*self.scale


def positive(value, maximum, name):
    value = float(value)
    if not math.isfinite(value) or not 0 < value <= maximum:
        raise OperatorValueError('Invalid velocity control setting: {name}', name=name)
    return value


class VelocityIntegrator:
    def __init__(self, speed, travel, acceleration=100.):
        self.speed = positive(speed, MAX_TCP_SPEED, 'TCP speed')
        self.travel = positive(travel, 50., 'linear travel limit')
        self.acceleration = positive(acceleration, 250., 'acceleration')
        self.velocity = np.zeros(3)
        self.position = np.zeros(3)
        self.distance = 0.
        self.exhausted = False

    def step(self, requested, dt=SERVO_PERIOD):
        dt = positive(dt, .032, 'servo period')
        desired = vector(requested, 3)
        length = np.linalg.norm(desired)
        if length > self.speed+1e-7:
            raise OperatorValueError('TCP velocity exceeds the configured maximum')
        # Neutral/release/zero throttle has no filter tail.
        if length < 1e-10:
            self.velocity[:] = 0
            return np.zeros(3)
        delta = desired-self.velocity
        change = float(np.linalg.norm(delta))
        if change > self.acceleration*dt:
            delta *= self.acceleration*dt/change
        self.velocity += delta
        actual_speed = float(np.linalg.norm(self.velocity))
        if actual_speed > length:
            # A lowered throttle/deflection is an immediate command ceiling.
            self.velocity *= length/actual_speed
        increment = self.velocity*dt
        length = float(np.linalg.norm(increment))
        remaining = max(0., self.travel-self.distance)
        if length >= remaining-1e-10:
            increment *= remaining/length if length else 0.
            self.exhausted = True
        self.distance += float(np.linalg.norm(increment))
        self.position += increment
        return increment
