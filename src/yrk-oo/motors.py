MOTOR0 = 60
MOTOR1 = 62
MOTOR2 = 66
MOTOR3 = 68


class MotorDriver:
    def __init__(self, motor, bus):
        self._bus = bus
        self._motor = motor
        self.speed = 0
        self.brake = False
        self.direction = 0

    def set(self, speed):
        if speed not in range(-1, 1):
            raise ValueError(speed)

        self.speed = 0

        if speed == 0:
            in2_in1 = 0b00
            self.direction = 0
        elif speed < 0:
            in2_in1 = 0b01
            self.direction = -1
        elif speed > 0:
            in2_in1 = 0b10
            self.direction = 1
    
        # number between 0x06 and 0x3f
        vset = 0x06 + int((0x3f - 0x06) * speed)
        byte = (vset << 2) | in2_in1
        self._bus.write_byte_data(self._motor, 0, byte)

    def brake(self):
        in2_in1 = 0b11
        self._bus.write_byte_data(self._motor, 0, in2_in1)

        self.brake = True
        self.speed = 0
        self.direction = 0

if __name__ == "__main__":
    from smbus2 import SMBus
    from time import sleep
    with SMBus(13) as bus:
        drv = MotorDriver(MOTOR0, bus)
        drv.set(1)
        print(f"reported values: speed={drv.speed} direction={drv.direcetion} brake={drv.brake}")
        sleep(0.5)
        drv.set(0.5)
        print(f"reported values: speed={drv.speed} direction={drv.direcetion} brake={drv.brake}")
        sleep(0.5)
        drv.set(0)
        print(f"reported values: speed={drv.speed} direction={drv.direcetion} brake={drv.brake}")
        sleep(0.5)
        drv.set(-0.5)
        print(f"reported values: speed={drv.speed} direction={drv.direcetion} brake={drv.brake}")
        sleep(0.5)
        drv.set(-1)
        print(f"reported values: speed={drv.speed} direction={drv.direcetion} brake={drv.brake}")
        sleep(0.5)
        drv.brake()
        print(f"reported values: speed={drv.speed} direction={drv.direcetion} brake={drv.brake}")
