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
Instantiate SerialDriver, call init().  If init() returns False the HAL
falls back to MockDriver automatically.
"""

import json
import threading
from typing import Optional

from weedbot_core.hal.driver_interface import MotorDriverInterface


class SerialDriver(MotorDriverInterface):
    """
    UART driver.  Requires pyserial (`pip install pyserial`).
    Falls back gracefully if the port is absent or pyserial is not installed.
    """

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
        try:
            import serial  # type: ignore
        except ImportError:
            return False  # pyserial not installed → HAL falls back to mock

        try:
            self._serial = serial.Serial(
                self._port,
                self._baud,
                timeout=0.1,
                write_timeout=0.1,
            )
            self._connected = True
        except Exception:
            self._connected = False
            return False

        # Start background thread that reads sensor frames from MCU
        self._stop_reader.clear()
        self._reader_thread = threading.Thread(
            target=self._read_loop, daemon=True)
        self._reader_thread.start()
        return True

    def shutdown(self) -> None:
        self._stop_reader.set()
        if self._reader_thread:
            self._reader_thread.join(timeout=1.0)
        with self._lock:
            if self._serial:
                try:
                    self._serial.close()
                except Exception:
                    pass
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
        """Broadcast emergency stop — MCU zeroes all PWM outputs immediately."""
        self._send({'cmd': 'estop'})

    def heartbeat(self) -> None:
        """Keep-alive frame — MCU watchdog resets its timeout on receipt."""
        self._send({'cmd': 'hb'})

    # ────────────────────────────────────────────────────────────────────
    # Sensor reads (returns latest value from background reader)
    # ────────────────────────────────────────────────────────────────────

    def read_encoder(self, channel: int) -> int:
        with self._lock:
            if 0 <= channel < len(self._encoders):
                return self._encoders[channel]
        return 0

    def read_current(self, channel: int) -> float:
        with self._lock:
            if 0 <= channel < len(self._currents):
                return self._currents[channel]
        return 0.0

    # ────────────────────────────────────────────────────────────────────
    # Internal helpers
    # ────────────────────────────────────────────────────────────────────

    def _send(self, obj: dict) -> None:
        if not self._connected or self._serial is None:
            return
        frame = (json.dumps(obj, separators=(',', ':')) + '\n').encode()
        with self._lock:
            try:
                self._serial.write(frame)
            except Exception:
                self._connected = False

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
            except Exception:
                # Serial error — stop thread; HAL will detect disconnection
                self._connected = False
                break

    def _parse_frame(self, text: str) -> None:
        if not text:
            return
        try:
            obj = json.loads(text)
        except json.JSONDecodeError:
            return

        if 'err' in obj:
            # MCU reported an error — log but don't crash
            return

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
