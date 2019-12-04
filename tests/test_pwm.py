import unittest
from unittest.mock import Mock, patch, call

from yrk_oo.pwm import PWMController, PWMOutput


@patch('yrk_oo.pwm.PWMOutput', autospec=True)
@patch('smbus2.SMBus', autospec=True)
class TestPWMController(unittest.TestCase):
    def test_init_runs(self, SMBus, *mocks):
        bus = SMBus(0)
        PWMController(0, bus)
        PWMController(0, bus, 200)

    def test_init_bus_write(self, SMBus, *mocks):
        bus = SMBus(0)
        PWMController(0, bus)
        bus.write_byte_data.assert_called_once()
        bus.write_byte_data.assert_called_with(0, 0, 0b00100000)

    def test_init_set_freq(self, SMBus, *mocks):
        bus = SMBus(0)
        PWMController(0, bus, 200)
        bus.write_byte_data.assert_any_call(0, 0xfe, 0x1e)

    def test_channels(self, SMBus, *mocks):
        bus = SMBus(0)
        ctrlr = PWMController(0, bus)
        self.assertTrue(hasattr(ctrlr, 'channels'))
        self.assertIsInstance(ctrlr.channels, tuple)
        self.assertEqual(len(ctrlr.channels), 16)
        for c in ctrlr.channels:
            self.assertIsInstance(c, PWMOutput)

    def test_global_channel(self, SMBus, *mocks):
        bus = SMBus(0)
        ctrlr = PWMController(0, bus)
        self.assertTrue(hasattr(ctrlr, 'channels'))
        self.assertIsInstance(ctrlr.global_channel, PWMOutput)

    def test_set_frequency(self, SMBus, *mocks):
        bus = SMBus(0)
        ctrlr = PWMController(0, bus)
        ctrlr.sleep = Mock(spec=PWMController.sleep)
        ctrlr.wakeup = Mock(spec=PWMController.wakeup)

        ctrlr._sleeping = True
        ctrlr.set_frequency(200)
        bus.write_byte_data.assert_any_call(0, 0xfe, 0x1e)
        ctrlr.sleep.assert_not_called()
        ctrlr.wakeup.assert_not_called()
        ctrlr._sleeping = False

        ctrlr.set_frequency(200)
        ctrlr.sleep.assert_called_once()
        ctrlr.wakeup.assert_called_once()

        ctrlr.set_frequency(1)
        self.assertAlmostEqual(ctrlr.frequency, 24, places=0)
        ctrlr.set_frequency(2000)
        self.assertAlmostEqual(ctrlr.frequency, 1526, places=0)

        self.assertRaises(ValueError, ctrlr.set_frequency, 0)

    def test_sleep(self, SMBus, *mocks):
        bus = SMBus(0)
        bus.read_byte_data.return_value = 0
        ctrlr = PWMController(0, bus)
        bus.reset_mock()

        ctrlr.sleep()
        bus.write_byte_data.assert_called_with(0, 0, 0b00010000)
        self.assertTrue(ctrlr.sleeping)

        bus.read_byte_data.return_value = 0b00101010
        ctrlr.sleep()
        bus.write_byte_data.assert_called_with(0, 0, 0b00111010)

    def test_wakeup(self, SMBus, *mocks):
        bus = SMBus(0)
        ctrlr = PWMController(0, bus)

        bus.reset_mock()
        bus.read_byte_data.return_value = 0
        self.assertFalse(ctrlr.sleeping)
        ctrlr.wakeup()
        bus.write_byte_data.assert_not_called()

        bus.reset_mock()
        bus.read_byte_data.return_value = 0
        ctrlr._sleeping = True
        ctrlr.wakeup()
        bus.write_byte_data.assert_called_once()
        bus.write_byte_data.assert_called_with(0, 0, 0)
        self.assertFalse(ctrlr.sleeping)

        bus.reset_mock()
        bus.read_byte_data.return_value = 0b10110101
        ctrlr._sleeping = True
        ctrlr.wakeup()
        calls = [call(0, 0, 0b00100101),
                 call(0, 0, 0b10100101)]
        bus.write_byte_data.assert_has_calls(calls)


@patch("yrk_oo.pwm.PWMController")
@patch("smbus2.SMBus")
class TestPWMOutput(unittest.TestCase):
    pass
