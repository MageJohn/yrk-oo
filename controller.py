from evdev import list_devices, InputDevice, categorize, ecodes
import time
import math

class Controller:

    def __init__(self):
        self.devices = InputDevice(list_devices()[0])
        self.buffer = (-10, 10)

    def getAxisVal(self, axis):
        #set axis to 0 for left js and 1 for the right
        for event in self.device.read_loop():
            if axis == 0:
                if event in ecodes.ABS_Y:
                    raw_val = float(categorize(event).event.value)
                    out = ((raw_val - 128) /128) * 100
                    return out
            else:
                if event in ecodes.ABS_Z:
                    raw_val = float(categorize(event).event.value)
                    out = ((raw_val - 128) /128) * 100
                    return out

    
