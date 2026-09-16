"""Fail-closed output gate. Software interlocks are not a hardware E-stop."""
import math
import time


def finite_number(value):
    try:
        return type(value) in (int, float) and math.isfinite(value)
    except OverflowError:
        return False


class SafetyGate:
    def __init__(self, driver, *, clock=time.monotonic, timeout=0.5):
        if not finite_number(timeout) or timeout <= 0:
            raise ValueError('timeout must be finite and positive')
        self.driver, self.clock, self.timeout = driver, clock, timeout
        self.armed = False
        self.reason = 'startup: explicit arm required'
        self.estop = False
        self.safety = 2
        self.motion = (0.0, 0.0)
        self.tool = (False, 0, False)
        self.velocity = 0.0
        self.yaw_rate = 0.0
        self.received = {}
        self.last_clock = -math.inf
        self.stop_errors = []
        self._stop()

    def _now(self):
        try:
            return self.clock()
        except Exception:
            return float('nan')

    def _stop(self):
        # Attempt every stop operation even when one transport operation fails.
        self.stop_errors = []
        operations = [self.driver.emergency_stop,
                      lambda: self.driver.apply_tool(False, 0, False)]
        operations += [lambda ch=ch: self.driver.set_pwm(ch, 0.0) for ch in range(4)]
        for operation in operations:
            try:
                operation()
            except Exception as exc:
                self.stop_errors.append(str(exc))

    def trip(self, reason):
        self.armed = False
        self.reason = reason
        self.motion, self.tool = (0.0, 0.0), (False, 0, False)
        # Old commands cannot satisfy a later arm request.
        self.received.pop('motion', None)
        self.received.pop('tool', None)
        self._stop()

    def _fault(self):
        if self.estop:
            return 'emergency-stop asserted'
        try:
            if not self.driver.is_healthy():
                return 'driver unhealthy'
        except Exception:
            return 'driver health unavailable'
        if not self.driver.supports_tool:
            return 'backend lacks verified tool support'
        if self.safety not in (0, 1):
            return 'safety STOP or EMERGENCY'
        now = self._now()
        if not finite_number(now) or now < self.last_clock:
            return 'invalid monotonic clock'
        self.last_clock = now
        for source in ('heartbeat', 'safety', 'feedback', 'motion', 'tool'):
            age = now - self.received.get(source, -math.inf)
            if not finite_number(age) or age < 0 or age >= self.timeout:
                return source + ' missing or stale'
        return None

    def tick(self):
        if self.armed:
            fault = self._fault()
            if fault:
                self.trip(fault)
        return self.armed

    def arm(self):
        self.tick()
        fault = self._fault()
        if fault or self.motion != (0.0, 0.0) or any(self.tool) or abs(self.velocity) > 0.01 or abs(self.yaw_rate) > 0.05:
            return False, fault or 'fresh neutral commands and measured stop required'
        self._stop()
        if self.stop_errors:
            return False, 'driver stop failed: ' + '; '.join(self.stop_errors)
        self.armed, self.reason = True, 'armed'
        return True, self.reason

    def heartbeat(self):
        self.tick()
        self.received['heartbeat'] = self._now()

    def update_safety(self, state):
        self.tick()
        self.safety = state if type(state) is int else 2
        self.received['safety'] = self._now()
        if self.safety not in (0, 1):
            self.trip('safety STOP or EMERGENCY')
        elif self.armed:
            self._apply()

    def emergency_stop(self, asserted):
        self.estop = bool(asserted)
        if self.estop:
            self.trip('emergency-stop asserted')

    def feedback(self, velocity, yaw_rate=0.0):
        self.tick()
        if not all(finite_number(v) for v in (velocity, yaw_rate)) or abs(velocity) > 2.0 or abs(yaw_rate) > 4.0:
            self.trip('invalid motion feedback')
            self.received.pop('feedback', None)
            return
        self.velocity, self.yaw_rate = velocity, yaw_rate
        self.received['feedback'] = self._now()
        if self.armed and any(self.tool) and (velocity <= 0.01 or abs(yaw_rate) > 0.05):
            self.trip('tool interlock: measured forward motion lost')

    def command_motion(self, linear, angular):
        self.tick()
        if not all(finite_number(v) for v in (linear, angular)) or abs(linear) > 1 or abs(angular) > 4:
            self.trip('invalid motion command')
            return
        self.motion = (linear, angular)
        self.received['motion'] = self._now()
        if self.armed:
            if any(self.tool) and (linear <= 0 or angular != 0):
                self.trip('tool interlock: forward command lost')
            else:
                self._apply()

    def command_tool(self, front, mode, agitator):
        self.tick()
        if type(front) is not bool or type(agitator) is not bool or type(mode) is not int or mode not in (0, 1, 2):
            self.trip('invalid tool command')
            return
        self.tool = (front, mode, agitator)
        self.received['tool'] = self._now()
        if self.armed:
            if any(self.tool) and (self.motion[0] <= 0 or self.motion[1] != 0 or self.velocity <= 0.01 or abs(self.yaw_rate) > 0.05):
                self.trip('tool interlock: forward motion required')
            else:
                self._apply()

    def _apply(self):
        if not self.tick():
            return
        linear, angular = self.motion
        scale = 0.5 if self.safety == 1 else 1.0
        left = max(-1.0, min(1.0, (200 * linear - 100 * angular) / 255)) * scale
        right = max(-1.0, min(1.0, (200 * linear + 100 * angular) / 255)) * scale
        try:
            for ch, duty in enumerate((left, right, left, right)):
                self.driver.set_pwm(ch, duty)
            self.driver.apply_tool(*self.tool)
        except Exception as exc:
            self.trip('driver output failed: ' + str(exc))

    def shutdown(self):
        self.trip('shutdown')
        self.driver.shutdown()
