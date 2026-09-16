"""Driver regressions using only in-memory transports; never opens hardware."""

import sys
from pathlib import Path
from types import SimpleNamespace

import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'deweeder_tertill/src/weedbot_core'))

from weedbot_core.drivers.mock_driver import MockDriver
from weedbot_core.drivers.serial_driver import SerialDriver
from weedbot_core.drivers.socketcan_driver import SocketCANDriver


class FakeSerial:
    def __init__(self, fail=False, short=False):
        self.events = []
        self.fail = fail
        self.short = short

    def write(self, frame):
        self.events.append(frame)
        if self.fail:
            raise IOError('injected serial failure')
        return len(frame) - int(self.short)

    def close(self):
        self.events.append('close')


class FakeBus:
    def __init__(self, fail=False):
        self.events = []
        self.fail = fail

    def send(self, frame, timeout):
        self.events.append(frame.arbitration_id)
        if self.fail:
            raise IOError('injected CAN failure')

    def shutdown(self):
        self.events.append('close')


class DriverTests(unittest.TestCase):
    def test_mock_stops_every_tool_and_wheel(self):
        driver = MockDriver()
        self.assertFalse(driver.is_healthy())
        self.assertTrue(driver.init())
        for channel in range(4):
            driver.set_pwm(channel, .5)
        driver.apply_tool(True, 2, True)
        self.assertEqual(driver._last_tool, (True, 2, True))
        driver.emergency_stop()
        self.assertEqual(driver._last_pwm, [0.0] * 4)
        self.assertEqual(driver.tool_state, {'front': False, 'mode': 0, 'agitator': False})
        driver.apply_tool(True, 1, True)
        driver.set_pwm(0, .2)
        driver.shutdown()
        self.assertEqual(driver._last_tool, (False, 0, False))
        self.assertEqual(driver._last_pwm, [0.0] * 4)
        self.assertFalse(driver.is_healthy())

    def test_disconnected_send_raises(self):
        for driver_type in (SerialDriver, SocketCANDriver):
            with self.subTest(driver=driver_type.__name__):
                driver = driver_type()
                self.assertFalse(driver.is_healthy())
                for command in (lambda: driver.set_pwm(0, .2), driver.heartbeat, driver.emergency_stop):
                    with self.assertRaises(RuntimeError):
                        command()

    def test_hardware_tools_explicitly_unsupported(self):
        for driver_type in (SerialDriver, SocketCANDriver):
            for state in ((True, 0, False), (False, 1, False), (False, 0, True)):
                with self.subTest(driver=driver_type.__name__, state=state):
                    driver = driver_type()
                    self.assertIs(driver.supports_tool, False)
                    driver.apply_tool(False, 0, False)
                    with self.assertRaises(NotImplementedError):
                        driver.apply_tool(*state)

    def test_serial_stop_before_close_even_on_failure(self):
        for fail in (False, True):
            with self.subTest(fail=fail):
                driver = SerialDriver()
                transport = FakeSerial(fail)
                driver._serial, driver._connected = transport, True
                if fail:
                    with self.assertRaises(IOError):
                        driver.shutdown()
                else:
                    driver.shutdown()
                self.assertEqual(transport.events, [b'{"cmd":"estop"}\n', 'close'])
                self.assertIsNone(driver._serial)
                self.assertFalse(driver.is_healthy())

    def test_can_stop_before_close_even_on_failure(self):
        fake_can = SimpleNamespace(Message=lambda **kw: SimpleNamespace(**kw))
        with patch.dict(sys.modules, {'can': fake_can}):
            for fail in (False, True):
                with self.subTest(fail=fail):
                    driver = SocketCANDriver()
                    transport = FakeBus(fail)
                    driver._bus, driver._ready = transport, True
                    if fail:
                        with self.assertRaises(IOError):
                            driver.shutdown()
                    else:
                        driver.shutdown()
                    self.assertEqual(transport.events, [0x180, 'close'])
                    self.assertIsNone(driver._bus)
                    self.assertFalse(driver.is_healthy())

    def test_serial_send_failure_revokes_health(self):
        for short in (False, True):
            with self.subTest(short=short):
                driver = SerialDriver()
                driver._serial, driver._connected = FakeSerial(fail=not short, short=short), True
                with self.assertRaises(IOError):
                    driver.set_pwm(0, .2)
                self.assertFalse(driver.is_healthy())
                with self.assertRaises(RuntimeError):
                    driver.read_encoder(0)

    def test_reader_fault_latches_and_surfaces_on_next_io(self):
        driver = SerialDriver()
        driver._serial = SimpleNamespace(readline=lambda: b'{"err":"fault"}\n')
        driver._connected = True
        driver._read_loop()
        self.assertFalse(driver.is_healthy())
        self.assertIsInstance(driver._last_error, RuntimeError)
        with self.assertRaises(RuntimeError):
            driver.heartbeat()

    def test_missing_dependency_never_becomes_mock(self):
        for driver_type, module_name in ((SerialDriver, 'serial'), (SocketCANDriver, 'can')):
            with self.subTest(driver=driver_type.__name__), patch.dict(sys.modules, {module_name: None}):
                driver = driver_type()
                with self.assertRaises(ImportError):
                    driver.init()
                self.assertFalse(driver.is_healthy())
                self.assertIs(type(driver), driver_type)


if __name__ == '__main__':
    unittest.main()
