# serial_driver.py
"""
SerialDriver stub.
This file provides the interface and a safe fallback for environments without pyserial.
If you later choose an MCU + uart protocol, implement the serial read/write and parsing here.
"""

from typing import Optional
from weedbot_core.hal.driver_interface import MotorDriverInterface


class SerialDriver(MotorDriverInterface):
    def __init__(self, port: str = "/dev/ttyUSB0", baud: int = 115200, config: Optional[dict] = None):
        self.port = port
        self.baud = baud
        self.config = config or {}
        self._connected = False
        self._serial = None

    def init(self) -> bool:
        try:
            import serial  # optional
            self._serial = serial.Serial(self.port, self.baud, timeout=0.1)
            self._connected = True
            return True
        except Exception:
            # serial not available or device not present — return False so HAL can fallback
            self._connected = False
            return False

    def set_pwm(self, channel: int, duty: float) -> None:
        raise NotImplementedError("SerialDriver.set_pwm must be implemented for your MCU protocol")

    def set_direction(self, channel: int, forward: bool) -> None:
        raise NotImplementedError("SerialDriver.set_direction must be implemented for your MCU protocol")

    def read_encoder(self, channel: int) -> int:
        raise NotImplementedError("SerialDriver.read_encoder must be implemented for your MCU protocol")

    def read_current(self, channel: int) -> float:
        raise NotImplementedError("SerialDriver.read_current must be implemented for your MCU protocol")

    def emergency_stop(self) -> None:
        # best-effort: send stop frame if connected
        if self._connected and self._serial:
            try:
                self._serial.write(b"EMERGENCY\n")
            except Exception:
                pass

    def heartbeat(self) -> None:
        # optional: send heartbeat frame
        if self._connected and self._serial:
            try:
                self._serial.write(b"HB\n")
            except Exception:
                pass

    def shutdown(self) -> None:
        try:
            if self._serial:
                self._serial.close()
        except Exception:
            pass
        self._connected = False
