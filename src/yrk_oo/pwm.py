"""Driver module for the 16-channel PWM controller on the YRL040

The YRL040 has a 16-channel PWM controller on it. Designed for LEDS, the
controller is still useful for all kinds of PWM output, including servos. The
chip has many options, but this interface is optimised for setting the duty
cycle on individual pins.

The datasheet can be found in docs/datasheets/PCA9685.pdf, or online at
https://www.nxp.com/docs/en/data-sheet/PCA9685.pdf.
"""

from smbus2 import SMBus
from typing import Union
from time import sleep

YRL040_PWM = 0x40


class PWMController:
    PRE_SCALE = 0xFE
    MODE1 = 0x00

    def __init__(self, addr: int, bus: SMBus, freq: Union[int, float, None] = None):
        self._addr = addr
        self._bus = bus

        self.channels = tuple(
            [
                PWMOutput(addr, bus, reg, self._calc_delay(reg), self)
                for reg in range(0x06, 0x46, 4)
            ]
        )

        # Writing to this channel writes to all the channels at once
        self.global_channel = PWMOutput(addr, bus, 0xFA, 0, self)

        if freq is not None:
            self._set_prescale(freq)

        # Set AI (bit 5); enables register autoincrement
        # Set SLEEP (bit 4); disables low power mode
        self._bus.write_byte_data(self._addr, self.MODE1, (1 << 5) | (1 << 4))

        self._sleeping = False

    def sleep(self):
        value = self._bus.read_byte_data(self._addr, self.MODE1)
        value |= 1 << 4
        self._bus.write_byte_data(self._addr, self.MODE1, value)
        self._sleeping = True

    def restart(self):
        if self.sleeping:
            value = self._bus.read_byte_data(self._addr, self.MODE1)
            value &= ~(1 << 4)
            self._bus.write_byte_data(self._addr, self.MODE1, value)
            if value & (1 << 7):
                sleep(500e-6)
                value |= 1 << 7
                self._bus.write_byte_data(self._addr, self.MODE1, value)
            self._sleeping = False

    @property
    def sleeping(self) -> bool:
        return self._sleeping

    def _set_prescale(self, freq: Union[int, float]):
        prescale_value = round(25e6 / (4096 * freq)) - 1

        if prescale_value < 0x03:
            raise ValueError(f"Frequency of {freq}Hz > 1526Hz. See datasheet, page 25.")
        elif prescale_value > 0xFF:
            raise ValueError(f"Frequency of {freq}Hz < 24Hz. See datasheet, page 25.")

        self._bus.write_byte_data(self._addr, self.PRE_SCALE, prescale_value)

    def _calc_delay(self, register: int) -> float:
        # To alleviate the stress on the system, have each PWM cycle start at a
        # different time. to achieve this, set a delay based on something
        # unique to each channel, its register.
        return (register - 0x6) / (4 * 16 - 1)


class PWMOutput:
    def __init__(
        self, addr: int, bus: SMBus, register: int, delay: float, parent: PWMController
    ):
        self._addr = addr
        self._bus = bus
        self._register = register
        self._off = True
        self._led_on = 0
        self._led_off = 0

        if delay < 0 or delay > 1:
            raise ValueError(f"{delay} is not in range [0, 1].")
        self._delay = delay

    def change_duty_cycle(self, duty_cycle: float):
        if duty_cycle < 0 or duty_cycle > 1:
            raise ValueError(f"{duty_cycle} is not in range [0, 1].")
        elif self._parent.sleeping:
            raise RuntimeError("Tried to change PWM values while sleeping.")

        self._led_on = int(self._delay * 4095)
        self._led_off = (self._led_on + int(duty_cycle * 4095)) % 4095

        self._bus.write_block_data(
            self._addr,
            self._register,
            data=[
                (self._led_on & 0xFF),
                ((self._led_on & 0xF00) >> 8),
                (self._led_off & 0xFF),
                (((self._led_off & 0xF00) >> 8) | int(self._off) << 4),
            ],
        )

    def set_enable(self, enable: bool):
        if self._parent.sleeping:
            raise RuntimeError("Tried to enable a PWM while sleeping.")
        led_off_h = self._register + 3
        current_val = (self._led_off & 0xF00) >> 8
        self._bus.write_byte_data(
            self._addr, led_off_h, current_val | (int(enable) << 4)
        )
        self._off = enable

    def set_full_on(self, enable: bool):
        if self._parent.sleeping:
            raise RuntimeError("Tried to turn channel full on while sleeping.")
        led_on_h = self._register + 1
        current_val = (self._led_on & 0xF00) >> 8
        self._bus.write_byte_data(
            self._addr, led_on_h, current_val | (int(enable) << 4)
        )
