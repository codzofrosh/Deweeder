# mock_driver.py
import time
from weedbot_core.hal.driver_interface import MotorDriverInterface

class MockDriver(MotorDriverInterface):
    def __init__(self, config=None):
        self.config = config or {}
        self.encoders = [0,0,0,0]
        self.currents = [0.0]*6
        self._alive = False

    def init(self) -> bool:
        self._alive = True
        return True

    def set_pwm(self, channel, duty):
        # clamp duty -1.0..1.0
        pass

    def set_direction(self, channel, forward):
        pass

    def read_encoder(self, channel):
        # sim: increment slightly
        self.encoders[channel] += int(1)
        return self.encoders[channel]

    def read_current(self, channel):
        return self.currents[channel]

    def emergency_stop(self):
        pass

    def heartbeat(self):
        pass

    def shutdown(self):
        self._alive = False
