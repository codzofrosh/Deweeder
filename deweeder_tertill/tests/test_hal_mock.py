# tests/test_hal_mock.py
import pytest
from weedbot_core.drivers.mock_driver import MockDriver


def test_mock_init_and_operations():
    d = MockDriver()
    assert d.init() is True
    # initial encoder reads are ints
    e0 = d.read_encoder(0)
    assert isinstance(e0, int)
    # set pwm and ensure encoder increases after read
    d.set_pwm(0, 0.5)
    before = d.read_encoder(0)
    after = d.read_encoder(0)
    assert after >= before
    # current non-negative
    cur = d.read_current(0)
    assert cur >= 0.0
    d.emergency_stop()
    d.shutdown()
