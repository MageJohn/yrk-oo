"""Contains the MotorDriver class and some useful constants.

Example::

    from smbus2 import SMBus
    with SMBus(1) as bus:
        driver = MotorDriver(MOTOR1, bus)
        driver.set(1)

Attributes:
    MOTOR1:
    MOTOR2:
    MOTOR3:
    MOTOR4: Constants for the addresses of each motor driver on the YRK. To be
        passed to the initialiser of MotorDriver
"""

from smbus2 import SMBus

MOTOR1: int = 0x60
MOTOR2: int = 0x62
MOTOR3: int = 0x66
MOTOR4: int = 0x68


class MotorDriver:
    """
    Interface for the Texas Instruments DRV8830 I2C motor driver.

    The YRK includes 4 DRV8830 drivers. The outputs of each driver are
    connected to one of the the four push-level Wago terminals. The driver
    outputs a PWM signal which is regulated by a voltage from an internal DAC,
    set by writing to an internal register with I2C. The driver can apply the
    voltage forward or backward, it can let the motor coast, or it can short
    circuit it as a brake.

    The datasheet for the driver is included at docs/datasheets/drv8830.pdf, or
    it can be found online at https://www.ti.com/lit/ds/symlink/drv8830.pdf.

    This interface abstracts this away; set the speed with a positive or
    negative unit interval, where negative values cause the motor to reverse,
    positive values cause it to go forward, and 0 sets the driver to
    coast/standby mode. A seperate method applies the brakes.
    """
    def __init__(self, motor: int, bus: SMBus):
        """Init the class with a specific I2C address and bus.

        Args:
            motor_addr: I2C address of the motor controller.
            bus: An open SMBus object.
        """
        self._bus = bus
        self._motor = motor
        self._speed = 0
        self._brake_state = False

    def set(self, speed: float):
        """Set the speed of the motor.

        Resets the brake state.

        Args:
            speed: A positive or negative unit interval (a value between -1 and
                1). If speed < 0, the motor goes in reverse; if speed > 0, the
                motor goes forward; if speed = 0, the motor coasts.
        """
        if speed < -1 or speed > 1:
            raise ValueError(f"Speed {speed} not in range -1 to 1")

        self._speed = speed

        # Bits IN1 and IN2 of the control register select the device mode. When
        # both are 0, the device is in low power standby/coast mode, when one
        # is set but not both the device goes forward or backward, and when
        # both are set the device brakes
        if speed == 0:
            in2_in1 = 0b00
        elif speed < 0:
            in2_in1 = 0b10
        elif speed > 0:
            in2_in1 = 0b01

        vset = 0x06 + int((0x3f - 0x06) * abs(speed))

        byte = (vset << 2) | in2_in1
        self._bus.write_byte_data(self._motor, 0, byte)

        self._brake_state = False

    def brake(self):
        """Put the motor into the brake state."""
        in2_in1 = 0b11
        self._bus.write_byte_data(self._motor, 0, in2_in1)

        self._brake_state = True
        self._speed = 0

    @property
    def speed(self) -> float:
        """Current speed the motor is set to. -1 <= speed <= 1."""
        return self._speed

    @property
    def direction(self) -> int:
        """Current direction of the motor.

        Returns:
            -1 for reverse, 0 for stationary, and 1 for forward.
        """
        if self._speed == 0:
            return self._speed
        else:
            return int(self._speed / abs(self._speed))

    @property
    def brake_state(self) -> bool:
        """Boolean of the state of the brake."""
        return self._brake_state

    @property
    def voltage(self) -> float:
        """The current approximate voltage across the motor."""
        # This calculation is taken from page 10 of the datasheet, but has been
        # modified. The calculation in the datasheet adds one to the VSET
        # value, but this produces values higher than those in the table on the
        # same page. This version produces the same values as the table.
        return (4 * 1.285 * _speed_to_vset(self.speed) / 64)


def _speed_to_vset(speed: float) -> int:
    """Convert a unit interval into a VSET value for the chip.

    The chip takes a value between 0x06 and 0x3f. Values
    below 0x06 are reserved, and since VSET is 6 bits 0x3F is the max
    that fits. See pages 9 to 10 of the datasheet for more info.

    Args:
        speed: A unit interval (the sign is ignored).
    Returns:
        An integer in the range 0x06 to 0x3f.
    """
    return 0x06 + round((0x3f - 0x06) * abs(speed))


if __name__ == "__main__":
    from smbus2 import SMBus
    from time import sleep
    with SMBus(13) as bus:
        drv = MotorDriver(MOTOR1, bus)
        drv.set(1)
        print(f"reported values: speed={drv.speed} direction={drv.direction} brake={drv.brake_state}")
        sleep(0.5)
        drv.set(0.5)
        print(f"reported values: speed={drv.speed} direction={drv.direction} brake={drv.brake_state}")
        sleep(0.5)
        drv.set(0)
        print(f"reported values: speed={drv.speed} direction={drv.direction} brake={drv.brake_state}")
        sleep(0.5)
        drv.set(-0.5)
        print(f"reported values: speed={drv.speed} direction={drv.direction} brake={drv.brake_state}")
        sleep(0.5)
        drv.set(-1)
        print(f"reported values: speed={drv.speed} direction={drv.direction} brake={drv.brake_state}")
        sleep(0.5)
        drv.brake()
        print(f"reported values: speed={drv.speed} direction={drv.direction} brake={drv.brake_state}")
        drv.set(0)
