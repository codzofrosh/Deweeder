# socketcan_driver.py
"""
SocketCANDriver — python-can driver for CAN-capable motor controllers.

CAN frame layout (11-bit standard IDs)
───────────────────────────────────────
HOST → MCU:
  0x100 + channel   Set PWM
    Byte 0: direction (0=reverse, 1=forward)
    Bytes 1-2: duty magnitude as uint16, 0-10000 (representing 0.0-1.0)

  0x180            Emergency stop  (DLC=0, no data)
  0x181            Heartbeat       (DLC=0, no data)

MCU → HOST:
  0x200 + channel   Encoder count
    Bytes 0-3: int32, little-endian

  0x280 + channel   Motor current
    Bytes 0-1: uint16, milliamps, little-endian

Usage
─────
Install python-can (`pip install python-can`).
Bring up SocketCAN interface before running:
  sudo ip link set can0 type can bitrate 500000
  sudo ip link set can0 up
"""

import struct
import threading
from typing import Optional

from weedbot_core.hal.driver_interface import MotorDriverInterface

# CAN IDs
_CMD_PWM_BASE = 0x100      # + channel
_CMD_ESTOP = 0x180
_CMD_HB = 0x181
_RESP_ENC_BASE = 0x200     # + channel  (int32, 4 bytes)
_RESP_CUR_BASE = 0x280     # + channel  (uint16 milliamps, 2 bytes)

_DUTY_SCALE = 10000        # maps 0.0-1.0 → 0-10000


class SocketCANDriver(MotorDriverInterface):
    """
    CAN bus driver via python-can / SocketCAN.
    Transport failures propagate; tool control is not implemented or verified.
    """

    supports_tool = False

    def is_healthy(self) -> bool:
        """Transport status only; no physical actuator or firmware verification."""
        return bool(self._ready and self._bus is not None and self._last_error is None)

    def apply_tool(self, front: bool, mode: int, agitator: bool) -> None:
        if type(front) is not bool or type(agitator) is not bool or type(mode) is not int or mode < 0:
            raise ValueError('tool state requires booleans and a nonnegative integer mode')
        if front or mode or agitator:
            raise NotImplementedError('tool firmware protocol is not implemented or verified')
        # Neutral is an interface no-op, NOT a verified physical tool stop.
        # The gate refuses hardware arming while supports_tool remains False.

    def _require_healthy(self) -> None:
        if not self.is_healthy():
            raise RuntimeError('driver transport is unavailable or faulted') from self._last_error

    def __init__(
        self,
        interface: str = 'can0',
        bitrate: int = 500_000,
        config: Optional[dict] = None,
    ):
        self._interface = interface
        self._bitrate = bitrate
        self._config = config or {}

        self._bus = None
        self._ready = False
        self._last_error = None
        self._lock = threading.Lock()

        # Cached sensor data
        self._encoders: list[int] = [0, 0, 0, 0]
        self._currents: list[float] = [0.0] * 6

        self._stop_reader = threading.Event()
        self._reader_thread: Optional[threading.Thread] = None

    # ────────────────────────────────────────────────────────────────────
    # Lifecycle
    # ────────────────────────────────────────────────────────────────────

    def init(self) -> bool:
        if self._bus is not None:
            raise RuntimeError('driver already has a transport; shutdown before reinitializing')
        try:
            import can
            self._bus = can.interface.Bus(channel=self._interface, bustype='socketcan', bitrate=self._bitrate)
            self._last_error = None
            self._ready = True
            self._stop_reader.clear()
            self._reader_thread = threading.Thread(target=self._read_loop, daemon=True)
            self._reader_thread.start()
        except Exception as exc:
            self._ready = False
            self._last_error = exc
            if self._bus is not None:
                self.shutdown()
            raise
        return True

    def shutdown(self) -> None:
        try:
            if self._bus is not None:
                self.emergency_stop()
        finally:
            self._stop_reader.set()
            try:
                if self._reader_thread and self._reader_thread.is_alive():
                    self._reader_thread.join(timeout=1.0)
            finally:
                try:
                    with self._lock:
                        if self._bus is not None:
                            self._bus.shutdown()
                finally:
                    self._bus = None
                    self._ready = False

    # ────────────────────────────────────────────────────────────────────
    # Commands
    # ────────────────────────────────────────────────────────────────────

    def set_pwm(self, channel: int, duty: float) -> None:
        """
        Send PWM command.  duty ∈ [-1.0, 1.0].
        Direction and magnitude are packed into a 3-byte frame.
        """
        duty = max(-1.0, min(1.0, float(duty)))
        direction = 1 if duty >= 0.0 else 0
        magnitude = int(abs(duty) * _DUTY_SCALE)
        data = struct.pack('<BH', direction, magnitude)
        self._send(_CMD_PWM_BASE + int(channel), data)

    def set_direction(self, channel: int, forward: bool) -> None:
        """Send existing direction command with zero duty (protocol has no separate direction ID)."""
        data = struct.pack('<BH', 1 if forward else 0, 0)
        self._send(_CMD_PWM_BASE + int(channel), data)

    def emergency_stop(self) -> None:
        """Send existing estop frame; physical stop behavior is unverified."""
        self._send(_CMD_ESTOP, b'')

    def heartbeat(self) -> None:
        """Send heartbeat so MCU watchdogs don't trigger."""
        self._send(_CMD_HB, b'')

    # ────────────────────────────────────────────────────────────────────
    # Sensor reads
    # ────────────────────────────────────────────────────────────────────

    def read_encoder(self, channel: int) -> int:
        self._require_healthy()
        with self._lock:
            if 0 <= channel < len(self._encoders):
                return self._encoders[channel]
        return 0

    def read_current(self, channel: int) -> float:
        self._require_healthy()
        with self._lock:
            if 0 <= channel < len(self._currents):
                return self._currents[channel]
        return 0.0

    # ────────────────────────────────────────────────────────────────────
    # Internal
    # ────────────────────────────────────────────────────────────────────

    def _send(self, arb_id: int, data: bytes) -> None:
        if self._bus is None:
            raise RuntimeError('driver transport is not open') from self._last_error
        # Permit only a best-effort stop after a reader/transport fault.
        if arb_id != _CMD_ESTOP:
            self._require_healthy()
        try:
            import can  # type: ignore
            frame = can.Message(
                arbitration_id=arb_id,
                data=data,
                is_extended_id=False,
            )
            with self._lock:
                self._bus.send(frame, timeout=0.05)
        except Exception as exc:
            self._ready = False
            self._last_error = exc
            raise

    def _read_loop(self) -> None:
        """Background thread: parse incoming encoder / current frames."""
        while not self._stop_reader.is_set():
            try:
                if self._bus is None:
                    break
                frame = self._bus.recv(timeout=0.1)
                if frame is None:
                    continue
                self._parse_frame(frame)
            except Exception as exc:
                self._ready = False
                self._last_error = exc
                break

    def _parse_frame(self, frame) -> None:
        arb = frame.arbitration_id
        data = bytes(frame.data)

        # Encoder response: ID = 0x200 + channel, 4 bytes (int32 LE)
        if _RESP_ENC_BASE <= arb < _RESP_ENC_BASE + 6 and len(data) >= 4:
            ch = arb - _RESP_ENC_BASE
            value = struct.unpack('<i', data[:4])[0]
            with self._lock:
                if ch < len(self._encoders):
                    self._encoders[ch] = value

        # Current response: ID = 0x280 + channel, 2 bytes (uint16 milliamps LE)
        elif _RESP_CUR_BASE <= arb < _RESP_CUR_BASE + 6 and len(data) >= 2:
            ch = arb - _RESP_CUR_BASE
            milliamps = struct.unpack('<H', data[:2])[0]
            with self._lock:
                if ch < len(self._currents):
                    self._currents[ch] = milliamps / 1000.0
