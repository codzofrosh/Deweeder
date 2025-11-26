# socketcan_driver.py
"""
SocketCAN driver stub.
If you pick CAN-capable motor controllers later, implement send/receive
using python-can or socketcan here.
"""

from typing import Optional
from weedbot_core.hal.driver_interface import MotorDriverInterface


class SocketCANDriver(MotorDriverInterface):
    def __init__(self, interface: str = "can0", config: Optional[dict] = None):
        self.interface = interface
        self.config = config or {}
        self._bus = None
        self._ready = False

    def init(self) -> bool:
        # attempt to import python-can; if not available, return False to fallback to mock
        try:
            import can  # type: ignore
            # not creating bus here; leave for future hardware implementation
            self._ready = True
            return True
        except Exception:
            self._ready = False
            return False

    def set_pwm(self, channel: int, duty: float) -> None:
        raise NotImplementedError("SocketCANDriver.set_pwm not implemented yet")

    def set_direction(self, channel: int, forward: bool) -> None:
        raise NotImplementedError("SocketCANDriver.set_direction not implemented yet")

    def read_encoder(self, channel: int) -> int:
        raise NotImplementedError("SocketCANDriver.read_encoder not implemented yet")

    def read_current(self, channel: int) -> float:
        raise NotImplementedError("SocketCANDriver.read_current not implemented yet")

    def emergency_stop(self) -> None:
        # send broadcast stop if bus present (future)
        pass

    def heartbeat(self) -> None:
        # send heartbeat frame (future)
        pass

    def shutdown(self) -> None:
        self._ready = False
