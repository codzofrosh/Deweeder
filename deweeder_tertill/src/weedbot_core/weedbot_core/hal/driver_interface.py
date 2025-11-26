# driver_interface.py
# HAL driver interface (stable API)
from typing import Protocol


class MotorDriverInterface(Protocol):
    """
    Protocol defining the minimal HAL driver API.
    Implementations must provide these methods.
    """

    def init(self) -> bool:
        """Initialize hardware/driver. Return True if ready."""
        ...

    def set_pwm(self, channel: int, duty: float) -> None:
        """
        Set PWM duty for a motor channel.
        duty expected in range -1.0 .. 1.0 (negative may indicate reverse).
        """
        ...

    def set_direction(self, channel: int, forward: bool) -> None:
        """Set direction pin/state for a motor channel (optional)."""
        ...

    def read_encoder(self, channel: int) -> int:
        """Return encoder tick count (integer)."""
        ...

    def read_current(self, channel: int) -> float:
        """Return measured motor current (amps) or 0.0 if not available."""
        ...

    def emergency_stop(self) -> None:
        """Immediately stop all outputs; leave driver in a safe state."""
        ...

    def heartbeat(self) -> None:
        """Called periodically by the controller to reset a hardware watchdog if present."""
        ...

    def shutdown(self) -> None:
        """Clean shutdown and release resources."""
        ...
