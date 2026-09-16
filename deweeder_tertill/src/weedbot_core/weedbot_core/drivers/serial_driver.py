# serial_driver.py
"""
SerialDriver — JSON-over-UART protocol for weedbot MCU.

Wire protocol (newline-delimited JSON, 115200 8N1)
───────────────────────────────────────────────────
HOST → MCU  (commands):
  {"cmd":"set_pwm",  "ch":<0-5>, "duty":<-1.0..1.0>}
  {"cmd":"set_dir",  "ch":<0-5>, "fwd":<true|false>}
  {"cmd":"estop"}
  {"cmd":"hb"}                          # heartbeat keep-alive

MCU → HOST  (sensor frame, sent at ~50 Hz by MCU):
  {"enc":[i32,i32,i32,i32],
   "cur":[f32,f32,f32,f32,f32,f32],
   "ok":true}

If a command fails the MCU responds with:
  {"err":"<message>"}

Usage
─────
Instantiate SerialDriver and call init(); initialization failures propagate.
Hardware failure must never select a mock backend.
"""

import json
import threading
from typing import Optional

from weedbot_core.hal.driver_interface import MotorDriverInterface


class SerialDriver(MotorDriverInterface):
    """
    UART driver.  Requires pyserial (`pip install pyserial`).
    Transport failures propagate; tool control is not implemented or verified.
    """

    supports_tool = False

    def is_healthy(self) -> bool:
        """Transport status only; no physical actuator or firmware verification."""
        return bool(self._connected and self._serial is not None and self._last_error is None)

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
        port: str = '/dev/ttyUSB0',
        baud: int = 115200,
        config: Optional[dict] = None,
    ):
        self._port = port
        self._baud = baud
        self._config = config or {}

        self._serial = None
        self._connected = False
        self._last_error = None
        self._lock = threading.Lock()

        # Latest sensor frame received from MCU
        self._encoders: list[int] = [0, 0, 0, 0]
        self._currents: list[float] = [0.0] * 6

        # Background reader thread
        self._reader_thread: Optional[threading.Thread] = None
        self._stop_reader = threading.Event()

    # ────────────────────────────────────────────────────────────────────
    # Lifecycle
    # ────────────────────────────────────────────────────────────────────

    def init(self) -> bool:
        if self._serial is not None:
            raise RuntimeError('driver already has a transport; shutdown before reinitializing')
        try:
            import serial
            self._serial = serial.Serial(self._port, self._baud, timeout=0.1, write_timeout=0.1)
            self._last_error = None
            self._connected = True
            self._stop_reader.clear()
            self._reader_thread = threading.Thread(target=self._read_loop, daemon=True)
            self._reader_thread.start()
        except Exception as exc:
            self._connected = False
            self._last_error = exc
            if self._serial is not None:
                self.shutdown()
            raise
        return True

    def shutdown(self) -> None:
        try:
            if self._serial is not None:
                self.emergency_stop()
        finally:
            self._stop_reader.set()
            try:
                if self._reader_thread and self._reader_thread.is_alive():
                    self._reader_thread.join(timeout=1.0)
            finally:
                try:
                    with self._lock:
                        if self._serial is not None:
                            self._serial.close()
                finally:
                    self._serial = None
                    self._connected = False

    # ────────────────────────────────────────────────────────────────────
    # Commands (HOST → MCU)
    # ────────────────────────────────────────────────────────────────────

    def set_pwm(self, channel: int, duty: float) -> None:
        """Send PWM duty cycle [-1.0 … 1.0] to motor channel."""
        duty = max(-1.0, min(1.0, float(duty)))
        self._send({'cmd': 'set_pwm', 'ch': int(channel), 'duty': duty})

    def set_direction(self, channel: int, forward: bool) -> None:
        """Set motor direction independently of PWM magnitude."""
        self._send({'cmd': 'set_dir', 'ch': int(channel), 'fwd': bool(forward)})

    def emergency_stop(self) -> None:
        """Send existing estop command; physical stop behavior is unverified."""
        self._send({'cmd': 'estop'})

    def heartbeat(self) -> None:
        """Keep-alive frame — MCU watchdog resets its timeout on receipt."""
        self._send({'cmd': 'hb'})

    # ────────────────────────────────────────────────────────────────────
    # Sensor reads (returns latest value from background reader)
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
    # Internal helpers
    # ────────────────────────────────────────────────────────────────────

    def _send(self, obj: dict) -> None:
        if self._serial is None:
            raise RuntimeError('driver transport is not open') from self._last_error
        # Permit only a best-effort stop after a reader/transport fault.
        if obj.get('cmd') != 'estop':
            self._require_healthy()
        frame = (json.dumps(obj, separators=(',', ':')) + '\n').encode()
        with self._lock:
            try:
                written = self._serial.write(frame)
                if written != len(frame):
                    raise IOError('incomplete serial command write')
            except Exception as exc:
                self._connected = False
                self._last_error = exc
                raise

    def _read_loop(self) -> None:
        """Background thread: reads newline-delimited JSON from MCU."""
        while not self._stop_reader.is_set():
            try:
                if self._serial is None:
                    break
                raw = self._serial.readline()
                if not raw:
                    continue
                self._parse_frame(raw.decode(errors='replace').strip())
            except Exception as exc:
                self._connected = False
                self._last_error = exc
                break

    def _parse_frame(self, text: str) -> None:
        if not text:
            return
        try:
            obj = json.loads(text)
        except json.JSONDecodeError as exc:
            raise ValueError('invalid serial sensor frame') from exc

        if not isinstance(obj, dict):
            raise ValueError('serial sensor frame must be an object')
        if 'err' in obj or obj.get('ok') is False:
            raise RuntimeError('MCU reported an unhealthy state')

        with self._lock:
            if 'enc' in obj and isinstance(obj['enc'], list):
                enc = [int(v) for v in obj['enc']]
                # Accept 2-wheel (L/R) or 4-wheel layout
                if len(enc) == 2:
                    self._encoders = [enc[0], enc[1], enc[0], enc[1]]
                elif len(enc) >= 4:
                    self._encoders = enc[:4]

            if 'cur' in obj and isinstance(obj['cur'], list):
                cur = [float(v) for v in obj['cur']]
                self._currents = (cur + [0.0] * 6)[:6]
