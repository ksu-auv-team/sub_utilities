import ms5837
import time

class Depth_Sensor:
    def __init__(self):
        self.sensor = ms5837.MS5837_30BA()

        if not self.sensor.init():
            print("Sensor could not be initialized")
            exit(1)
        
        if not self.sensor.read():
            print("Sensor read failed!")
            exit(1)

        self.sensor.setFluidDensity(ms5837.DENSITY_FRESHWATER)

    def get_depth(self):
        if self.sensor.read():
            return self.sensor.depth()
        else:
            return -1