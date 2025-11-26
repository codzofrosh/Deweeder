# mock_driver.py
"""
MockDriver: simple deterministic simulation driver for testing.
This is the default backend when hw_mode=False.
"""

import time
from typing import Optional
from weedbot_core.hal.driver_interface import MotorDriverInterface


class MockDriver(MotorDriverInterface):
    def __init__(self, config: Optional[dict] = None):
        self.config = config or {}
        self.encoders = [0, 0, 0, 0]  # Four wheel encoders
        self.currents = [0.0] * 6
        self._alive = False
        self._last_pwm = [0.0, 0.0, 0.0, 0.0]
        self._last_direction = [True] * 4
        self._last_heartbeat = time.time()

    def init(self) -> bool:
        self._alive = True
        # small deterministic startup delay
        time.sleep(0.01)
        return True

    def set_pwm(self, channel: int, duty: float) -> None:
        # clamp and store
        if channel < 0 or channel >= len(self._last_pwm):
            return
        duty_clamped = max(-1.0, min(1.0, float(duty)))
        self._last_pwm[channel] = duty_clamped
        # simulate current proportional to duty magnitude
        self.currents[channel] = abs(duty_clamped) * 0.5

    def set_direction(self, channel: int, forward: bool) -> None:
        if channel < 0 or channel >= len(self._last_direction):
            return
        self._last_direction[channel] = bool(forward)

    def read_encoder(self, channel: int) -> int:
        # simulate ticks: proportional to pwm and time
        pwm = self._last_pwm[channel] if 0 <= channel < len(self._last_pwm) else 0.0
        # increment by small deterministic amount
        inc = int(round(abs(pwm) * 2.0))  # 0,1,2 ticks per read
        self.encoders[channel] += inc
        return self.encoders[channel]

    def read_current(self, channel: int) -> float:
        if 0 <= channel < len(self.currents):
            return float(self.currents[channel])
        return 0.0

    def emergency_stop(self) -> None:
        # zero pwms and set currents to zero
        self._last_pwm = [0.0] * len(self._last_pwm)
        self.currents = [0.0] * len(self.currents)

    def heartbeat(self) -> None:
        self._last_heartbeat = time.time()

    def shutdown(self) -> None:
        self._alive = False
        # noop for mock
