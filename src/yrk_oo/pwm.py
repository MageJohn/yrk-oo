"""Driver module for the 16-channel PWM controller on the YRL040

The YRL040 has a 16-channel PWM controller on it. Designed for LEDS, the
controller is still useful for all kinds of PWM output, including servos. The
chip has many options, but this interface is optimised for setting the duty
cycle on individual pins.

The datasheet can be found in docs/datasheets/PCA9685.pdf, or online at
https://www.nxp.com/docs/en/data-sheet/PCA9685.pdf.
"""

from smbus2 import SMBus, i2c_msg
from typing import Tuple, Dict, Optional
from time import sleep
from enum import IntEnum
import struct

PWM_RESOLUTION = 4096
PWM_MAX_COUNT = PWM_RESOLUTION - 1
PWM_CLOCK_HZ = 25e6

PWM_I2C_BUS = 11
"""I2C bus the PWM is on"""
PWM_I2C_ADDR = 0x40
"""I2C address of the PCA9685 on the YRL040"""


class Registers(IntEnum):
    """Enumerator which maps register names to addresses."""

    MODE1 = 0x00
    PRE_SCALE = 0xFE


class Channels(IntEnum):
    """Enumerator which maps channel names to the address of their first register.

    Each channel is controlled by two 13-bit registers, or as the I2C interface
    sees it, four 8-bit registers; the register addresses are sequential. This
    enumerator points to the first address of each channel, subsequent addresses
    can be found by addition.
    """

    CHAN0 = 0x06
    CHAN1 = 0x0A
    CHAN2 = 0x0E
    CHAN3 = 0x12
    CHAN4 = 0x16
    CHAN5 = 0x1A
    CHAN6 = 0x1E
    CHAN7 = 0x22
    CHAN8 = 0x26
    CHAN9 = 0x2A
    CHAN10 = 0x2E
    CHAN11 = 0x32
    CHAN12 = 0x36
    CHAN13 = 0x3A
    CHAN14 = 0x3E
    CHAN15 = 0x42
    CHAN_ALL = 0xFA


class PWMController:
    """Controls the state of the chip as a whole, including frequency and sleep
    mode.

    When created it disables the sleep state and enables autoincrementing the
    address.

    Arguments:
        bus: The open I2C bus the chip is attached to
        addr: The I2C address of the chip to use
        freq: The frequency of all the PWM outputs (not the duty cycle).
            Default: approx 200Hz
    Attributes:
        channels: Maps members of Channels to PWMOutputs representing each channel.
    """

    def __init__(self, bus: SMBus, addr: int, freq: Optional[float] = None):
        self._addr = addr
        self._bus = bus

        self.channels: Dict["Channels", "PWMOutput"] = {
            channel: PWMOutput(bus, addr, channel, self._calc_delay(channel), self)
            for channel in Channels
        }

        self._prescale = 0x1E
        if freq is not None:
            self._set_prescale(freq)

        # Set AI (MODE1 bit 5); enables register autoincrement
        # This also resets SLEEP (MODE1 bit 4) and disables low power mode
        self._mode1 = 1 << 5
        self._bus.write_byte_data(self._addr, Registers.MODE1, self._mode1)

    def sleep(self) -> None:
        """Put the PWM controller into sleep mode.

        In this mode the oscillator is off. All outputs are switched off and
        the chip is in low power mode. If all the outputs were disabled (by
        calling `set_enable(False)` on all enabled channels or on the global
        channel), then the state of each PWM is reset and will need to be set
        up again once the controller is woken up. If not then when the system
        is started again with :meth:`wakeup` all PWMs will continue.
        """
        self._mode1 |= 1 << 4
        self._bus.write_byte_data(self._addr, Registers.MODE1, self._mode1)

    def wakeup(self) -> None:
        """Wakeup the controller after being put into sleep mode.

        If the controller was not in sleep mode, this does nothing. Otherwise
        it disable sleep mode and attempts to restart any running PWMs if
        possible.
        """
        if self.sleeping:
            value = self._bus.read_byte_data(self._addr, Registers.MODE1)
            restart = bool(value & (1 << 7))
            value &= ~(1 << 4) & ~(1 << 7)
            self._mode1 = value
            self._bus.write_byte_data(self._addr, Registers.MODE1, value)
            if restart:
                sleep(500e-6)
                value |= 1 << 7
                self._bus.write_byte_data(self._addr, Registers.MODE1, value)

    @property
    def sleeping(self) -> bool:
        """True if the controller was put into sleep mode."""
        return bool(self._mode1 & (1 << 4))

    @property
    def frequency(self) -> float:
        """The frequency of the PWM signals.

        Calculated based on the prescale value set on the chip, and may be
        slightly different from the one set by :meth:`set_frequency`.
        """
        # prescale + 1 = clocks per increment
        # clocks per increment * increments per PWM cycle = clocks per PWM cycle
        # clocks per PWM cycle * seconds per clock (aka 1 / clock freqency) =
        #       seconds per PWM cycle
        # 1 / seconds per PWM cycle = PWM cycles per second, aka the frequency
        return 1 / ((self._prescale + 1) * PWM_RESOLUTION / PWM_CLOCK_HZ)

    @property
    def min_pulse_width(self) -> float:
        """Current minimum pulse width in microseconds
        """
        return width_limits_from_freq(self.frequency)[0]

    @property
    def max_pulse_width(self) -> float:
        """Current maximum pulse width in microseconds
        """
        return width_limits_from_freq(self.frequency)[1]

    @property
    def global_channel(self) -> "PWMOutput":
        """Writing to this channel writes the same pattern to all the
        channels. Reads from it always return 0/False.

        This is the same as `channels[Channels.CHAN_ALL]`.
        """
        return self.channels[Channels.CHAN_ALL]

    def set_frequency(self, freq: float) -> None:
        """Set the frequency of the PWM controller.

        If the chip is not asleep, this method puts the chip to sleep, sets the
        frequency, then wakes it up again.

        The frequency range is approximately 23.84Hz to 1525.88Hz.

        Arguments:
            freq: The new frequency.
        Raises:
            ValueError: If freq outside of approx range 23.84 < freq < 1525.88.
        """
        if self.sleeping:
            self._set_prescale(freq)
        else:
            self.sleep()
            self._set_prescale(freq)
            self.wakeup()

    def sync(self) -> None:
        """Debugging method. Syncs the values in registers on the chip with the
        values stored inside the PWMController object and it's child PWMOutput
        objects.
        """
        self._mode1 = self._bus.read_byte_data(self._addr, Registers.MODE1)
        self._prescale = self._bus.read_byte_data(self._addr, Registers.PRE_SCALE)
        for channel in self.channels.values():
            channel._read_state()

    def _set_prescale(self, freq: float) -> None:
        """Private function to set the prescale value from a frequency

        Arguments:
            freq: The target frequency.
        """
        # To produce the PWM signal a counter is incremented every
        # prescale_value + 1 clock cycles (the plus one is because the clock
        # cycles are counted from 0).
        # The calculation is:
        #     increments per second =
        #          increments per PWM cycle * desired PWM cycles per second
        #     seconds per increment = 1 / increments per second
        #     clocks per increment = clocks per second * seconds per increment
        #     prescale value = round(clocks per increment) - 1
        prescale_value = round(PWM_CLOCK_HZ / (PWM_RESOLUTION * freq)) - 1

        # 0x03 is minumum enforced by hardware; 0xFF is maximum 8-bit value.
        if prescale_value < 0x03 or prescale_value > 0xFF:
            raise ValueError(f"Frequency {freq} not supported by device.")
        self._prescale = prescale_value

        self._bus.write_byte_data(self._addr, Registers.PRE_SCALE, prescale_value)

    def _calc_delay(self, register: "Channels") -> float:
        """Private function to manage the delay between each channel.

        To alleviate the stress on the system, each PWM cycle can start at a
        different time. This function calculates a delay based on the register
        address of the channel.

        The delay on channel 0 will be 0, channel 1 1/16, channel 2 2/16, etc.

        Arguments:
            register: The register to calculate with.
        """
        if register is Channels.CHAN_ALL:
            return 0
        else:
            return (register - Channels.CHAN0) / (4 * 16 - 1)


class PWMOutput:
    """Used to control a single output on the chip.

    Usually you will not instantiate this class directly. Instead use the
    members of :attr:`PWMController.channels`.

    Arguments:
        addr: I2C address of the chip.
        bus: An open I2C bus.
        channel: The channel on the chip to control
        delay: Unit interval proportion of the cycle to delay the rising edge.
        parent: The parent chip the output belongs to.

    Raises:
        ValueError: If delay is not a unit interval.
    """

    def __init__(
        self,
        bus: "SMBus",
        addr: int,
        channel: "Channels",
        delay: float,
        parent: "PWMController",
    ):
        self._addr = addr
        self._bus = bus
        self._first_reg = channel
        self._parent = parent

        if delay < 0 or delay > 1:
            raise ValueError(f"{delay} is not in range [0, 1].")

        self._on_count = int(delay * PWM_MAX_COUNT)
        self._full_on = False
        self._off_count = 0
        self._full_off = True

        self._write_state()

    def change_duty_cycle(self, duty_cycle: float) -> None:
        """Change the duty cycle of the output.

        Arguments:
            duty_cycle: `0 < duty_cycle <= 1`. Represents the proportion of the
                wave which is high. The proportion which is low is `1 - duty_cycle`.
                A duty cycle of 0 is not allowed; instead disable the output with
                :meth:`set_enable`. A duty cycle of 1 sets the output to constant high.
        Raises:
            ValueError: If `duty_cycle` is not a unit interval.
            RuntimeError: If the chip is in sleep mode.
        """
        on_time = int(duty_cycle * PWM_RESOLUTION)
        if on_time == PWM_RESOLUTION:  # 100% on
            self._full_on = True
            self._write_state()
        elif on_time <= 0 or on_time > PWM_MAX_COUNT:
            # A duty cycle of approximately 0.00024 or less is technically in
            # the range (0, 1], but when multiplied by 4096 and converted to an
            # int results in 0.
            raise ValueError(
                f"{duty_cycle} is not in range (0, 1] (adjusted for resolution)."
            )
        elif self._parent.sleeping:
            raise RuntimeError("Tried to change PWM values while sleeping.")
        else:
            self._full_on = False
            self._off_count = (self._on_count + on_time) % PWM_RESOLUTION
            self._write_state()

    def change_pulse_width(self, us_pulse_width: float) -> None:
        """Change the pulse width in microseconds.

        The limiting values for the pulse width are decided by the frequency.
        The properties :attr:`PWMController.max_pulse_width` and
        :attr:`PWMController.min_pulse_width` provide the current limits.

        Arguments:
            us_pulse_width: The pulse width to set in microseconds.
        Raises:
            ValueError: if `us_pulse_width` is outside the current limits.
        """
        limits = width_limits_from_freq(self._parent.frequency)
        if us_pulse_width < limits[0] or us_pulse_width > limits[1]:
            raise ValueError(
                f"{us_pulse_width} can't be acheived with {self._parent.frequency}Hz."
            )
        us_per_increment = limits[0]
        increments = round(us_pulse_width / us_per_increment)
        self._off_count = (self._on_count + increments) % PWM_RESOLUTION
        self._full_on = False
        self._write_state()

    def set_enable(self, enable: bool) -> None:
        """Enable or disable the output.

        Arguments:
            enable: True to enable the output, False to disable.

        Raises:
            RuntimeError: If the chip is in sleep mode.
        """
        if self._parent.sleeping:
            raise RuntimeError("Tried to modify a PWM while sleeping.")

        self._full_off = not enable
        self._write_state()

    @property
    def duty_cycle(self) -> float:
        """The current duty cycle of the output.

        Given as a unit interval, a float in the range [0, 1]. Calculated from
        the values set on the chip, not the value set by
        :meth:`set_duty_cycle`. If the channel is not enabled or is the global
        channel the value is zero.
        """
        if self._full_off or self._first_reg == Channels.CHAN_ALL:
            return 0.0
        elif self._full_on:
            return 1.0
        else:
            on_time = (self._off_count - self._on_count) % PWM_RESOLUTION
            return on_time / PWM_RESOLUTION

    @property
    def pulse_width(self) -> float:
        """The current pulse width in microseconds.

        If the output is 100% on this will equal the period of the signal.
        Calculated from the value set on the chip, not the value set by
        :meth:`change_pulse_width`. If the channel is not enabled or is the
        global channel the value is zero.
        """
        if self._full_off or self._first_reg == Channels.CHAN_ALL:
            return 0.0
        elif self._full_on:
            return 1e6 / self._parent.frequency
        else:
            us_per_increment = width_limits_from_freq(self._parent.frequency)[0]
            increments = (self._off_count - self._on_count) % PWM_RESOLUTION
            return increments * us_per_increment

    @property
    def enabled(self) -> bool:
        """Whether the channel is enabled. Read only.

        If the object is controlling the global channel, always returns False."""
        if self._first_reg == Channels.CHAN_ALL:
            return False
        else:
            return not self._full_off

    def _read_state(self) -> None:
        """Debugging method. Sets the internal state based on the 4 registers
        in data.
        """
        msg = i2c_msg.read(self._addr, 4)
        self._bus.i2c_rdwr(msg)
        data = struct.unpack("<h<h", msg.buf)
        self._on_count = data[0] & 0xFFF
        self._full_on = bool(data[0] & 0x1000)
        self._off_count = data[1] & 0xFFF
        self._full_off = bool(data[1] & 0x1000)

    def _write_state(self) -> None:
        """Take the local object state and write it to the channel."""
        buf = struct.pack(
            "<Bhh",
            self._first_reg,
            (self._on_count | (int(self._full_on) << 12)),
            (self._off_count | (int(self._full_off) << 12)),
        )
        msg = i2c_msg.write(self._addr, buf)
        self._bus.i2c_rdwr(msg)


def width_limits_from_freq(freq: float) -> Tuple[float, float]:
    """Calculate min and max pulse widths possible with a particular frequency.

    Arguments:
        freq: Frequency to calculate for.
    Returns:
        Minumum and maximum pulse widths in microseconds.
    """
    period = (1 / freq) * 1e6  # In microseconds
    increment_period = period / PWM_RESOLUTION
    return increment_period, increment_period * PWM_MAX_COUNT
