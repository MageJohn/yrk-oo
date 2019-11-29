from evdev import list_devices, InputDevice, categorize, ecodes
import RPi.GPIO as gpio
import time
import math

gpio.setwarnings(False)


class Motors(object):
    def __init__(self):
        gpio.setmode(gpio.BCM)
        gpio.setup(12,gpio.OUT)
        gpio.setup(16,gpio.OUT)
        gpio.setup(20,gpio.OUT)
        gpio.setup(21,gpio.OUT)

        
        self._motors = (gpio.PWM(12,100), # f1
                        gpio.PWM(16,100), # f2
                        gpio.PWM(20,100), # b1
                        gpio.PWM(21,100)) # b2
        
        for m in self._motors:
            m.start(0)

    def set_values(self, *values):
        values = [max(-v, v) for v in values]
        for i in range(len(self._motors)):
            self._motors[i].ChangeDutyCycle(values[i])
    def exit(self):
        for m in self._motors:
            m.stop()
            gpio.cleanup()


device = InputDevice(list_devices()[0])

dz = (-10, 10)

try:
    motors = Motors()
    while True:
        for event in device.read_loop():
            if event.code in (ecodes.ABS_Y, ecodes.ABS_Z):
                raw_value = float(categorize(event).event.value)
                ud = ((raw_value - 128) / 128) * 100
                print(raw_value, ud)
                if event.code == ecodes.ABS_Y:
                    if ud < dz[0]:
                        motors.set_values(ud, ud, 0, 0)        
                    elif ud > dz[1]:
                        motors.set_values(0, 0, ud, ud)        
                    else:
                        motors.set_values(0, 0, 0, 0)
                elif event.code == ecodes.ABS_Z:
                    if ud < dz[0]:
                        motors.set_values(ud, 0, 0, ud)
                    elif ud > dz[1]:
                        motors.set_values(0, ud, ud, 0)
                    else:
                        motors.set_values(0, 0, 0, 0)
except KeyboardInterrupt:
    motors.exit()
