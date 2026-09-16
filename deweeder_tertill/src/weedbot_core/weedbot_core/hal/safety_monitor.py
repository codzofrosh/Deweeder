"""Pure fail-closed sensor monitor; gravity estimate needs sensor +Z upright.

Dynamic acceleration can bias tilt. Limits are provisional software settings,
not a calibrated overturn detector or an independent physical stop.
"""
from dataclasses import dataclass
import math
import time

SAFETY_NORMAL, SAFETY_SLOW, SAFETY_STOP, SAFETY_EMERGENCY = range(4)
DEFAULT_LIMITS = {
    'state_timeout': 0.5, 'imu_timeout': 0.5,
    'tilt_slow_deg': 10.0, 'tilt_stop_deg': 25.0, 'tilt_emergency_deg': 45.0,
    'rate_slow_rad_s': 0.3, 'rate_stop_rad_s': 0.7, 'rate_emergency_rad_s': 1.5,
    'gravity_min_m_s2': 5.0, 'gravity_max_m_s2': 15.0,
}


def _finite(value):
    try:
        return type(value) in (int, float) and math.isfinite(value)
    except OverflowError:
        return False


@dataclass(frozen=True)
class SafetyStatus:
    state: int
    reason: str
    tilt_degrees: float | None
    emergency_latched: bool


class SafetyMonitor:
    """Local receipt freshness, with an emergency latch and deliberate reset.

    This cannot detect stale data republished by an upstream node. Reset requires
    fresh NORMAL inputs and does not reset or arm the downstream HAL gate.
    """

    def __init__(self, *, clock=time.monotonic, **limits):
        unknown = set(limits) - set(DEFAULT_LIMITS)
        if unknown:
            raise ValueError(f'unknown safety limits: {sorted(unknown)}')
        self.limits = dict(DEFAULT_LIMITS, **limits)
        if not callable(clock):
            raise ValueError('clock must be callable')
        for name, value in self.limits.items():
            if not _finite(value) or value <= 0:
                raise ValueError(f'{name} must be finite and positive')
        for names in (('tilt_slow_deg', 'tilt_stop_deg', 'tilt_emergency_deg'),
                      ('rate_slow_rad_s', 'rate_stop_rad_s', 'rate_emergency_rad_s')):
            if not self.limits[names[0]] < self.limits[names[1]] < self.limits[names[2]]:
                raise ValueError('slow, stop and emergency thresholds must increase')
        if self.limits['tilt_emergency_deg'] > 180:
            raise ValueError('tilt emergency threshold cannot exceed 180 degrees')
        if self.limits['gravity_min_m_s2'] >= self.limits['gravity_max_m_s2']:
            raise ValueError('gravity magnitude bounds must increase')
        self._clock = clock
        self._last_clock = None
        self._state_received = self._imu_received = None
        self._contact = (SAFETY_STOP, 'contact input missing')
        self._imu = (SAFETY_STOP, 'IMU input missing')
        self._tilt = None
        self._emergency_latched = False

    def _now(self):
        try:
            now = self._clock()
        except Exception:
            self._state_received = self._imu_received = None
            return None
        if not _finite(now) or (self._last_clock is not None and now < self._last_clock):
            self._state_received = self._imu_received = None
            return None
        self._last_clock = now
        return now

    def update_contacts(self, *, side=False, rear=False, belly=False, valid=True):
        self._state_received = self._now()
        if not valid or any(type(value) is not bool for value in (side, rear, belly, valid)):
            self._contact = (SAFETY_STOP, 'invalid contact input')
        elif side or rear:
            self._contact = (SAFETY_STOP, 'side or rear contact')
        elif belly:
            self._contact = (SAFETY_SLOW, 'belly contact')
        else:
            self._contact = (SAFETY_NORMAL, 'contacts clear')

    def update_imu(self, ax, ay, az, wx, wy, wz, *, valid=True):
        self._imu_received = self._now()
        self._tilt = None
        if type(valid) is not bool or not valid or not all(
                _finite(value) for value in (ax, ay, az, wx, wy, wz)):
            self._imu = (SAFETY_STOP, 'invalid IMU input')
            return
        rate = max(abs(wx), abs(wy))
        if rate >= self.limits['rate_emergency_rad_s']:
            self._imu = (SAFETY_EMERGENCY, 'extreme roll/pitch angular rate')
            self._emergency_latched = True
            return
        gravity = math.hypot(ax, ay, az)
        if not self.limits['gravity_min_m_s2'] <= gravity <= self.limits['gravity_max_m_s2']:
            self._imu = (SAFETY_STOP, 'acceleration cannot support gravity tilt estimate')
            return
        self._tilt = math.degrees(math.atan2(math.hypot(ax, ay), az))
        if self._tilt >= self.limits['tilt_emergency_deg']:
            self._imu = (SAFETY_EMERGENCY, 'extreme static tilt estimate')
            self._emergency_latched = True
        elif self._tilt >= self.limits['tilt_stop_deg'] or rate >= self.limits['rate_stop_rad_s']:
            self._imu = (SAFETY_STOP, 'tilt or angular rate exceeds stop limit')
        elif self._tilt >= self.limits['tilt_slow_deg'] or rate >= self.limits['rate_slow_rad_s']:
            self._imu = (SAFETY_SLOW, 'tilt or angular rate exceeds slow limit')
        else:
            self._imu = (SAFETY_NORMAL, 'IMU within limits')

    def _current(self):
        now = self._now()
        faults = [self._contact, self._imu]
        if now is None:
            faults.append((SAFETY_STOP, 'monotonic clock invalid or regressed'))
        else:
            for name, received, timeout in (
                ('contact', self._state_received, self.limits['state_timeout']),
                ('IMU', self._imu_received, self.limits['imu_timeout']),
            ):
                if received is None or now - received >= timeout:
                    faults.append((SAFETY_STOP, f'{name} input missing or stale'))
        level = max(state for state, _ in faults)
        reason = '; '.join(reason for state, reason in faults if state == level)
        return level, reason

    def evaluate(self):
        level, reason = self._current()
        if self._emergency_latched:
            level = SAFETY_EMERGENCY
            reason = 'emergency latched; ' + reason
        return SafetyStatus(level, reason, self._tilt, self._emergency_latched)

    def reset(self):
        """Return (success, reason); reject reset while any current hazard remains."""
        level, reason = self._current()
        if level != SAFETY_NORMAL:
            return False, 'reset rejected: ' + reason
        self._emergency_latched = False
        return True, 'safety emergency reset; HAL must be reset and armed separately'
