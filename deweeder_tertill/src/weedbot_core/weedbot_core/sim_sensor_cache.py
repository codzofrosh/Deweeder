"""Pure simulation input assembly; never turn expired caches into fresh packets."""

import math
import time


def _finite(value):
    try:
        return type(value) in (int, float) and math.isfinite(value)
    except OverflowError:
        return False


class SimSensorCache:
    SOURCES = ('odom', 'imu', 'front_contact', 'belly_contact')

    def __init__(self, *, clock=time.monotonic, source_timeout=0.5):
        if not callable(clock) or not _finite(source_timeout) or source_timeout <= 0:
            raise ValueError('clock must be callable and source_timeout finite and positive')
        self.clock, self.timeout = clock, source_timeout
        self._received = {}
        self._values = {}
        self._last_clock = None
        self._odom_stamp = self._emitted_stamp = None
        self._revision = self._emitted_revision = 0
        self._encoders = [0.0] * 4
        self._discontinuous = True

    def _now(self):
        try:
            now = self.clock()
        except Exception:
            now = None
        if not _finite(now) or (self._last_clock is not None and now < self._last_clock):
            self._received.clear()
            self._discontinuous = True
            return None
        self._last_clock = now
        return now

    def _invalidate(self, source):
        self._received.pop(source, None)
        self._discontinuous = True

    def update_contact(self, source, contact):
        if source not in ('front_contact', 'belly_contact'):
            raise ValueError('unknown contact source')
        now = self._now()
        if now is None or type(contact) is not bool:
            self._invalidate(source)
            return False
        self._values[source] = contact
        self._received[source] = now
        return True

    def update_imu(self, linear, angular, *, valid=True):
        now = self._now()
        try:
            values = tuple(linear) + tuple(angular)
            good = len(linear) == len(angular) == 3 and all(_finite(v) for v in values)
        except (TypeError, ValueError):
            good = False
        if now is None or type(valid) is not bool or not valid or not good:
            self._invalidate('imu')
            return False
        self._values['imu'] = (list(linear), list(angular))
        self._received['imu'] = now
        return True

    def update_odom(self, stamp, linear, angular):
        now = self._now()
        if now is None or not all(_finite(v) for v in (stamp, linear, angular)) or stamp < 0:
            self._invalidate('odom')
            return False
        if self._odom_stamp is not None:
            dt = stamp - self._odom_stamp
            if dt <= 0 or dt >= self.timeout:
                # Rebase source time, but demand another progressing sample.
                self._odom_stamp = stamp
                self._invalidate('odom')
                return False
            ticks_per_meter = 2048 / (2 * math.pi * 0.035)
            left = (linear - angular * 0.14 / 2) * dt * ticks_per_meter
            right = (linear + angular * 0.14 / 2) * dt * ticks_per_meter
            encoders = [old + delta for old, delta in
                        zip(self._encoders, (left, right, left, right))]
            if not all(_finite(v) and -(2**31) <= v < 2**31 for v in encoders):
                self._odom_stamp = stamp
                self._invalidate('odom')
                return False
            self._encoders = encoders
        self._odom_stamp = stamp
        self._received['odom'] = now
        self._revision += 1
        return True

    def packet(self):
        """Return one snapshot per odom update, or None while anything is stale.

        tick_time is the source odom interval since the last emitted snapshot.
        First/recovery snapshots use 0 to invalidate velocity and seed a new
        baseline. Subscriber packet loss remains ambiguous without a sequence ID.
        """
        now = self._now()
        if now is None or any(source not in self._received or
                              now - self._received[source] >= self.timeout
                              for source in self.SOURCES):
            self._discontinuous = True
            return None
        if self._revision == self._emitted_revision:
            return None
        dt = 0.0 if self._discontinuous or self._emitted_stamp is None else (
            self._odom_stamp - self._emitted_stamp)
        linear, angular = self._values['imu']
        result = {'wheel_encoder': [int(value) for value in self._encoders],
                  'imu_linear': linear[:], 'imu_angular': angular[:],
                  'cap_front': self._values['front_contact'],
                  'belly_contact': self._values['belly_contact'],
                  'tick_time': dt}
        self._emitted_revision = self._revision
        self._emitted_stamp = self._odom_stamp
        self._discontinuous = False
        return result
