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
    Interface for a single I2C motor driver
    """
    def __init__(self, motor: int, bus: SMBus):
        """Init the class with a specific I2C address and bus

        Args:
            motor_addr: I2C address of the motor controller
            bus: An open SMBus object
        """
        self._bus = bus
        self._motor = motor
        self._speed = 0
        self._brake_state = False

    def set(self, speed: float) -> None:
        """Set the speed of the motor

        Args:
            speed: A value between -1 and 1. If speed < 0, the motor
                goes in reverse; if speed > 0, the motor goes forward; if
                motor = 0, then the motor coasts.
        """
        if speed < -1 or speed > 1:
            raise ValueError(speed)

        self._speed = speed

        if speed == 0:
            in2_in1 = 0b00
        elif speed < 0:
            in2_in1 = 0b01
        elif speed > 0:
            in2_in1 = 0b10
    
        # number between 0x06 and 0x3f
        vset = 0x06 + int((0x3f - 0x06) * abs(speed))
        byte = (vset << 2) | in2_in1
        self._bus.write_byte_data(self._motor, 0, byte)

    def brake(self) -> None:
        """Put the motor into the brake state"""
        in2_in1 = 0b11
        self._bus.write_byte_data(self._motor, 0, in2_in1)

        self._brake_state = True
        self._speed = 0

    @property
    def speed(self) -> float:
        """Current speed the motor is set to. -1 <= speed <= 1"""
        return self._speed

    @property
    def direction(self) -> int:
        """Current direction of the motor

        -1 is reverse, 0 is stationary, and 1 is forward
        """
        return int(self._speed / abs(self._speed))

    @property
    def brake_state(self) -> bool:
        """Boolean of the state of the brake"""
        return self._brake_state

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
