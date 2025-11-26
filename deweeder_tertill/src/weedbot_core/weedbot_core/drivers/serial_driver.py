# serial_driver.py
from weedbot_core.hal.driver_interface import MotorDriverInterface

class SerialDriver(MotorDriverInterface):
    def __init__(self, port="/dev/ttyUSB0", baud=115200, config=None):
        self.port = port
        self.baud = baud

    def init(self):
        # Attempt import/pyserial only if used
        try:
            import serial
        except Exception:
            # Not available in simulation - return False or fallback to simulate
            return False
        # implement real init if desired
        return True

    def set_pwm(self, channel, duty): raise NotImplementedError("Serial backend not implemented yet")
    ...
