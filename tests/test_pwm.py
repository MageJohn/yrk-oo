import unittest
from unittest.mock import Mock, PropertyMock, patch, call, ANY
from ctypes import string_at

from yrk_oo.pwm import (
    PWMController,
    PWMOutput,
    Channels,
    Registers,
    width_limits_from_freq,
)


class TestPWMController(unittest.TestCase):
    def setUp(self):
        patcher1 = patch("yrk_oo.pwm.PWMOutput", autospec=True)
        patcher2 = patch("smbus2.SMBus", autospec=True)

        patcher1.start()
        SMBus = patcher2.start()

        self.bus = SMBus(0)

        self.addCleanup(patcher1.stop)
        self.addCleanup(patcher2.stop)

    def test_init_bus_write(self):
        PWMController(self.bus, 0)
        self.bus.write_byte_data.assert_called_once()
        self.bus.write_byte_data.assert_called_with(0, 0, 0b00100000)

    def test_init_set_freq(self):
        PWMController(self.bus, 0, 200)
        self.bus.write_byte_data.assert_any_call(0, 0xFE, 0x1E)

    def test_channels(self):
        ctrlr = PWMController(self.bus, 0)
        self.assertTrue(hasattr(ctrlr, "channels"))
        self.assertIsInstance(ctrlr.channels[Channels.CHAN0], PWMOutput)
        self.assertIs(ctrlr.channels[Channels.CHAN_ALL], ctrlr.global_channel)

    def test_set_frequency(self):
        with patch(
            "yrk_oo.pwm.PWMController.sleeping", new_callable=PropertyMock
        ) as sleeping:
            ctrlr = PWMController(self.bus, 0)
            ctrlr.sleep = Mock(spec=PWMController.sleep)
            ctrlr.wakeup = Mock(spec=PWMController.wakeup)

            sleeping.return_value = True

            ctrlr.set_frequency(200)
            self.bus.write_byte_data.assert_any_call(0, 0xFE, 0x1E)
            ctrlr.sleep.assert_not_called()
            ctrlr.wakeup.assert_not_called()

            sleeping.return_value = False

            ctrlr.set_frequency(200)
            ctrlr.sleep.assert_called_once()
            ctrlr.wakeup.assert_called_once()

            self.assertRaises(ValueError, ctrlr.set_frequency, 1)
            self.assertRaises(ValueError, ctrlr.set_frequency, 2000)

    def test_sleep_sleeping(self):
        ctrlr = PWMController(self.bus, 0)

        ctrlr.sleep()
        self.bus.write_byte_data.assert_called_with(0, Registers.MODE1, 0b00110000)
        self.assertTrue(ctrlr.sleeping)

    @patch("yrk_oo.pwm.PWMController.sleeping", new_callable=PropertyMock)
    @patch("yrk_oo.pwm.sleep")
    def test_wakeup_sleeping(self, sleep, sleeping):
        ctrlr = PWMController(self.bus, 0)
        self.bus.read_byte_data.return_value = 0
        container = Mock()
        container.attach_mock(sleep, "sleep")
        container.attach_mock(self.bus.write_byte_data, "write")

        self.bus.reset_mock()
        sleeping.return_value = False
        ctrlr.wakeup()
        self.bus.write_byte_data.assert_not_called()

        sleeping.return_value = True
        ctrlr.wakeup()
        self.bus.write_byte_data.assert_called_with(0, 0, 0)

        container.reset_mock()
        self.bus.read_byte_data.return_value = 0b10110101
        ctrlr.wakeup()
        calls = [
            call.write(0, 0, 0b00100101),
            call.sleep(ANY),
            call.write(0, 0, 0b10100101),
        ]
        container.assert_has_calls(calls)
        self.assertGreaterEqual(sleep.call_args[0][0], 500e-6)


class TestPWMOutput(unittest.TestCase):
    def setUp(self):
        patcher1 = patch("yrk_oo.pwm.PWMController", autospec=True)
        patcher2 = patch("smbus2.SMBus", autospec=True)

        self.PWMController = patcher1.start()

        SMBus = patcher2.start()

        self.bus = SMBus(0)

        self.addCleanup(patcher1.stop)
        self.addCleanup(patcher2.stop)

    def assertInternalState(self, obj, **kwargs):
        for kw, val in kwargs.items():
            self.assertEqual(getattr(obj, kw), val)

    @patch("yrk_oo.pwm.PWMOutput._write_state", autospec=True)
    def test_creation(self, write_state):
        chan = PWMOutput(self.bus, 0, 0, 0, self.PWMController)
        self.assertInternalState(
            chan, _on_count=0, _full_on=False, _off_count=0, _full_off=True
        )
        write_state.assert_called_once()
        write_state.reset_mock()

        chan = PWMOutput(self.bus, 0, 0, (0xFF / 4095), self.PWMController)
        self.assertInternalState(
            chan, _on_count=0xFF, _full_on=False, _off_count=0, _full_off=True
        )
        write_state.assert_called_once()

    def test_change_duty_cycle(self):
        chan = PWMOutput(self.bus, 0, 0, 0, self.PWMController)
        chan._write_state = Mock()

        self.assertRaises(ValueError, chan.change_duty_cycle, 0)
        self.assertRaises(ValueError, chan.change_duty_cycle, -1)
        self.assertRaises(ValueError, chan.change_duty_cycle, 1.1)

        self.PWMController.sleeping = True
        self.assertRaises(RuntimeError, chan.change_duty_cycle, 0.5)
        self.PWMController.sleeping = False

        chan.change_duty_cycle(0.5)
        self.assertInternalState(chan, _on_count=0, _off_count=0x800, _full_on=False)
        chan._write_state.assert_called_once()
        chan._write_state.reset_mock()

        chan.change_duty_cycle(1)
        self.assertInternalState(chan, _full_on=True)
        chan._write_state.assert_called_once()
        chan._write_state.reset_mock()

        chan.change_duty_cycle(0.2)
        self.assertInternalState(chan, _on_count=0, _off_count=0x333, _full_on=False)
        chan._write_state.assert_called_once()

    @patch("yrk_oo.pwm.width_limits_from_freq")
    def test_change_pulse_width(self, width_limits_from_freq):
        width_limits_from_freq.return_value = (1, 1000)
        chan = PWMOutput(self.bus, 0, 0, 0, self.PWMController)
        chan._write_state = Mock()

        self.assertRaises(ValueError, chan.change_pulse_width, 0)
        self.assertRaises(ValueError, chan.change_pulse_width, 1001)

        self.PWMController.sleeping = True
        self.assertRaises(RuntimeError, chan.change_duty_cycle, 0.5)
        self.PWMController.sleeping = False

        chan.change_pulse_width(500)
        self.assertInternalState(chan, _on_count=0, _off_count=500, _full_on=False)
        chan._write_state.assert_called_once()

    def test_set_enable(self):
        chan = PWMOutput(self.bus, 0, 0, 0, self.PWMController)
        chan._write_state = Mock()

        self.PWMController.sleeping = True
        self.assertRaises(RuntimeError, chan.set_enable, True)
        self.PWMController.sleeping = False

        chan.set_enable(True)
        self.assertInternalState(chan, _full_off=False)
        chan._write_state.assert_called_once()
        chan._write_state.reset_mock()

        chan.set_enable(False)
        self.assertInternalState(chan, _full_off=True)
        chan._write_state.assert_called_once()

    def test_prop_duty_cycle(self):
        chan = PWMOutput(self.bus, 0, 0, 0, self.PWMController)

        chan._full_off = True
        self.assertEqual(chan.duty_cycle, 0)
        chan._first_reg = Channels.CHAN_ALL
        chan._full_off = False
        self.assertEqual(chan.duty_cycle, 0)

        chan._first_reg = 0
        chan._off_count = 4096 * 0.5
        self.assertEqual(chan.duty_cycle, 0.5)

        chan._full_on = True
        self.assertEqual(chan.duty_cycle, 1.0)

    @patch("yrk_oo.pwm.width_limits_from_freq")
    def test_prop_pulse_width(self, width_limits_from_freq):
        self.PWMController.frequency = 200
        chan = PWMOutput(self.bus, 0, 0, 0, self.PWMController)
        width_limits_from_freq.return_value = (1, 1000)

        chan._full_off = True
        self.assertEqual(chan.pulse_width, 0)
        chan._first_reg = Channels.CHAN_ALL
        chan._full_off = False
        self.assertEqual(chan.pulse_width, 0)

        chan._off_count = 500
        chan._on_count = 0
        chan._first_reg = 0
        self.assertEqual(chan.pulse_width, 500)

        chan._full_on = True
        self.assertEqual(chan.pulse_width, 1e6 / 200)

    def test_prop_enabled(self):
        chan = PWMOutput(self.bus, 0, 0, 0, self.PWMController)

        self.assertInternalState(chan, _full_off=True)
        self.assertFalse(chan.enabled)

        chan._full_off = False
        self.assertTrue(chan.enabled)

        chan._first_reg = Channels.CHAN_ALL
        self.assertFalse(chan.enabled)
        chan._full_off = True
        self.assertFalse(chan.enabled)

    def test__write_state(self):
        states = (
            {"_on_count": 0, "_full_on": False, "_off_count": 0, "_full_off": True},
            {
                "_on_count": 0x123,
                "_full_on": False,
                "_off_count": 0x456,
                "_full_off": True,
            },
            {"_on_count": 0, "_full_on": True, "_off_count": 0, "_full_off": False},
        )
        expected_results = (
            b"\x00\x00\x00\x00\x10",
            b"\x00\x23\x01\x56\x14",
            b"\x00\x00\x10\x00\x00",
        )
        chan = PWMOutput(self.bus, 0, 0, 0, self.PWMController)

        for state, expected_result in zip(states, expected_results):
            self.bus.i2c_rdwr.reset_mock()
            # Set the object internal state
            chan.__dict__.update(state)
            chan._write_state()
            self.bus.i2c_rdwr.assert_called_once()

            # Take the i2c_msg passed to i2c_rdwr, take it's buf pointer, and
            # return the 5 bytes it points too:
            result = string_at(self.bus.i2c_rdwr.call_args[0][0].buf, size=5)
            self.assertEqual(result, expected_result)


class TestWidthLimitsFromFreq(unittest.TestCase):
    def test(self):
        limits = width_limits_from_freq(200)
        expected = (1.220703125, 4998.779296875)
        self.assertAlmostEqual(limits[0], expected[0], places=7)
        self.assertAlmostEqual(limits[1], expected[1], places=7)


if __name__ == "__main__":
    unittest.main()
