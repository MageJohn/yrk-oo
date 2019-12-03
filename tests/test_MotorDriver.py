import unittest
from unittest.mock import Mock

from yrk_oo.motors import MotorDriver


class TestMotorDriver(unittest.TestCase):
    def test_creation(self):
        MotorDriver(0, Mock())

    def test_set_zero(self):
        bus = Mock()
        drv = MotorDriver(0, bus)
        drv.set(0)
        bus.write_byte_data.assert_called_with(0, 0, 0b00011000)

    def test_set_forward(self):
        bus = Mock()
        drv = MotorDriver(0, bus)
        drv.set(1)
        bus.write_byte_data.assert_called_with(0, 0, 0b11111101)

    def test_set_backward(self):
        bus = Mock()
        drv = MotorDriver(0, bus)
        drv.set(-1)
        bus.write_byte_data.assert_called_with(0, 0, 0b11111110)

    def test_brake(self):
        bus = Mock()
        drv = MotorDriver(0, bus)
        drv.brake()
        bus.write_byte_data.assert_called_with(0, 0, 0b00000011)

    def test_speed_property(self):
        bus = Mock()
        drv = MotorDriver(0, bus)
        self.assertEqual(drv.speed, 0)
        drv.set(0.5)
        self.assertEqual(drv.speed, 0.5)
        drv.set(-0.5)
        self.assertEqual(drv.speed, -0.5)

    def test_direction_property(self):
        bus = Mock()
        drv = MotorDriver(0, bus)
        self.assertEqual(drv.direction, 0)
        drv.set(-0.5)
        self.assertEqual(drv.direction, -1)
        drv.set(0.5)
        self.assertEqual(drv.direction, 1)

    def test_brake_state_property(self):
        bus = Mock()
        drv = MotorDriver(0, bus)
        self.assertFalse(drv.brake_state)
        drv.set(0.5)
        self.assertFalse(drv.brake_state)
        drv.brake()
        self.assertTrue(drv.brake_state)
        drv.set(0)
        self.assertFalse(drv.brake_state)

    def test_voltage_property(self):
        bus = Mock()
        drv = MotorDriver(0, bus)
        drv.set(0.5)
        self.assertAlmostEqual(drv.voltage, 2.73, places=2)
        drv.set(1)
        self.assertAlmostEqual(drv.voltage, 5.06, places=2)
