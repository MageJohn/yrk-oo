"""Driver module for the 16-channel PWM controller on the YRL040

The YRL040 has a 16-channel PWM controller on it. Designed for LEDS, the
controller is still useful for all kinds of PWM output, including servos. The
chip has many options, but this interface is optimised for setting the duty
cycle on individual pins.

The datasheet can be found in docs/datasheets/PCA9685.pdf, or online at
https://www.nxp.com/docs/en/data-sheet/PCA9685.pdf.
"""

from smbus2 import SMBus
from typing import Union, Tuple
from time import sleep
from enum import Enum
from yrk_oo.utils import Pointer

YRL040_PWM = 0x40


class PointerEnum(Pointer, Enum):
    """Pointer enumerator. Members will have values of type Pointer."""
    def __new__(cls, value):
        obj = Pointer.__new__(cls, value)
        obj._value_ = Pointer(value)
        return obj


class Registers(PointerEnum):
    """Enumerator which maps register names to addresses."""
    MODE1 = 0x00
    PRE_SCALE = 0xFE


Channels = PointerEnum(
    "Channels",
    list(zip([f"CHAN{n}" for n in range(16)], range(0x06, 0x45 + 1, 4)))
    + [("CHAN_ALL", 0xFA)],
)
"""Enumerator which maps channel names to the address of their first register.

Each channel is controlled by two 13-bit registers, or as the I2C interface
sees it, four 8-bit registers; the register addresses are sequential. This
enumerator points to the first address of each channel, subsequent addresses
can be found by adding
"""


class PWMController:
    """Master controller for the PWM chip

    This class controls the entire PWM chip. It's used to control the state of
    the chip as a whole, including frequency and sleep mode.

    Attributes:
        channels: A tuple of PWMOutput objects representing the 16 different channels
    """

    channels: Tuple["PWMOutput", ...] = tuple()

    def __init__(self, addr: int, bus: SMBus, freq: Union[int, float, None] = None):
        """Initiliase the chip controller.

        If a frequency is passed it is set. If it is not passed, the default is
        approximately 200Hz.

        The chip begins in a sleep state, so disable SLEEP. Also enable
        autoincrementing the address so that all of the registers of a channel
        can be written to with only one start and stop.

        Arguments:
            addr: The I2C address of the chip to use
            bus: The open I2C bus the chip is attached to
            freq: The frequency of all the PWM outputs (not the duty cycle)
        """
        self._addr = addr
        self._bus = bus

        self._sleeping = True

        self.channels = tuple(
            [
                PWMOutput(addr, bus, reg, self._calc_delay(reg), self)
                for reg in range(Channels.CHAN0, Channels.CHAN15 + 1, 4)
            ]
        )

        # Writing to this channel writes to all the channels at once
        self._global_channel = PWMOutput(addr, bus, Channels.CHAN_ALL, 0, self)

        if freq is not None:
            self._set_prescale(freq)
        else:
            self._prescale = 0x1E

        # Set AI (MODE1 bit 5); enables register autoincrement
        # This also resets SLEEP (MODE1 bit 4) and disables low power mode
        self._bus.write_byte_data(self._addr, Registers.MODE1, (1 << 5))

        self._sleeping = False

    def sleep(self):
        """Put the PWM controller into sleep mode.

        In this mode the oscillator is off. All outputs are switched off and
        the chip is in low power mode. If all the outputs were disabled (by
        calling `set_enable(False)` on all enabled channels or on the global
        channel), then the state of each PWM is reset and will need to be set
        up again once the controller is woken up. If not then when the system
        is started again with :meth:`wakeup` all PWMs will continue.
        """
        value = self._bus.read_byte_data(self._addr, Registers.MODE1)
        value |= 1 << 4
        self._bus.write_byte_data(self._addr, Registers.MODE1, value)
        self._sleeping = True

    def wakeup(self):
        """Wakeup the controller after being put into sleep mode.

        If the controller was not in sleep mode, do nothing. Otherwise disable
        sleep mode and attempt to restart running PWMs if possible.
        """
        if self.sleeping:
            value = self._bus.read_byte_data(self._addr, Registers.MODE1)
            restart = bool(value & (1 << 7))
            value &= ~(1 << 4) & ~(1 << 7)
            self._bus.write_byte_data(self._addr, Registers.MODE1, value)
            if restart:
                sleep(500e-6)
                value |= 1 << 7
                self._bus.write_byte_data(self._addr, Registers.MODE1, value)
            self._sleeping = False

    @property
    def sleeping(self) -> bool:
        """True if the controller was put into sleep mode."""
        return self._sleeping

    @property
    def frequency(self) -> float:
        """The frequency of the PWM signals.

        Calculated based on the prescale value set on the chip.
        """
        return 1 / ((self._prescale + 1) * 4096 / 25e6)

    @property
    def global_channel(self) -> "PWMOutput":
        """Writing to this channel writes the same pattern to all the
        channels.
        """
        return self._global_channel

    def set_frequency(self, freq: Union[int, float]):
        """Set the frequency of the PWM controller.

        If the chip is not asleep, this method puts the chip to sleep, sets the
        frequency, then wakes it up again.

        The frequency range is approximately 24Hz to 1526Hz, and frequencies
        which fall outside of the limits of the device will be clamped.

        Arguments:
            freq: The new frequency.
        Raises:
            ValueError: If freq <= 0
        """
        if self.sleeping:
            self._set_prescale(freq)
        else:
            self.sleep()
            self._set_prescale(freq)
            self.wakeup()

    def _set_prescale(self, freq: Union[int, float]):
        """Private function to set the prescale value from a frequency

        Arguments:
            freq: The target frequency.
        """
        if freq <= 0:
            raise ValueError(freq)
        prescale_value = round(25e6 / (4096 * freq)) - 1
        prescale_value = max(prescale_value, 0x03)
        prescale_value = min(prescale_value, 0xFF)
        self._prescale = prescale_value

        self._bus.write_byte_data(self._addr, Registers.PRE_SCALE, prescale_value)

    def _calc_delay(self, register: "Channels") -> float:
        """Private function to manage the delay between each channel.

        To alleviate the stress on the system, each PWM cycle can start at a
        different time. This function calculates a delay based on the register
        address of the channel.

        Arguments:
            register: The register to calculate with.
        """
        return (register - 0x6) / (4 * 16 - 1)


class PWMOutput:
    """Used to control a single output on the chip."""

    def __init__(
        self,
        addr: int,
        bus: "SMBus",
        register: "Channels",
        delay: float,
        parent: "PWMController",
    ):
        """Initialise the class

        Arguments:
            addr: I2C address of the chip.
            bus: Open I2C bus.
            register: The first of the 4 sequential registers used to control
                the output.
            delay: Unit interval proportion of the cycle to delay the rising edge.
            parent: The parent chip the output belongs to.

        Raises:
            ValueError: If delay is not a unit interval.
        """
        self._addr = addr
        self._bus = bus
        self._registers = register
        self._off = True
        self._led_on = 0
        self._led_off = 0

        if delay < 0 or delay > 1:
            raise ValueError(f"{delay} is not in range [0, 1].")
        self._delay = delay

    def change_duty_cycle(self, duty_cycle: float):
        """Change the duty cycle of the output.

        Set the on time of the PWM signal.

        Arguments:
            duty_cycle: ``0 < duty_cycle <= 1``. Represents the proportion of the
                wave which is high. The proportion which is low is ``1 - duty_cycle``.
                A duty cycle of 0 is not allowed; instead disable the output with
                :meth:`set_enable`.
        Raises:
            ValueError: If *duty_cycle* is not a unit interval.
            RuntimeError: If the chip is in sleep mode.
        """
        if duty_cycle <= 0 or duty_cycle > 1:
            raise ValueError(f"{duty_cycle} is not in range (0, 1].")
        elif self._parent.sleeping:
            raise RuntimeError("Tried to change PWM values while sleeping.")
        elif duty_cycle == 1:
            self.set_full_on(True)
            return

        self._led_on = int(self._delay * 4095)
        self._led_off = (self._led_on + int(duty_cycle * 4095)) % 4095

        self._bus.write_block_data(
            self._addr,
            self._registers,
            data=[
                (self._led_on & 0xFF),
                ((self._led_on & 0xF00) >> 8),
                (self._led_off & 0xFF),
                (((self._led_off & 0xF00) >> 8) | int(self._off) << 4),
            ],
        )

    def set_enable(self, enable: bool):
        """Enable or disable the output.

        Arguments:
            enable: New state of the output.

        Raises:
            RuntimeError: If the chip is in sleep mode.
        """
        if self._parent.sleeping:
            raise RuntimeError("Tried to enable a PWM while sleeping.")
        led_off_h = self._registers[3]
        current_val = (self._led_off & 0xF00) >> 8
        self._bus.write_byte_data(
            self._addr, led_off_h, current_val | (int(enable) << 4)
        )
        self._off = enable

    def _set_full_on(self, enable: bool):
        """When set the output is always high.

        Used to set the duty cycle to 100%.

        Arguments:
            enable: Value of the full ON bit
        """
        led_on_h = self._registers[1]
        current_val = (self._led_on & 0xF00) >> 8
        self._bus.write_byte_data(
            self._addr, led_on_h, current_val | (int(enable) << 4)
        )
