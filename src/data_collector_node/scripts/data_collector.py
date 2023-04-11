import ms5837
import rospy
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
        
class arduino:
    def __init__(self):
        pass

def startup():
    pub = rospy.Publisher('data', String, queue_size=10)
    rospy.init_node('data_collector', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    rospy.loginfo("Starting up data collector node")

    depth_sensor = Depth_Sensor()
    # arduino = arduino()

    while not rospy.is_shutdown():
        depth = depth_sensor.get_depth()
        # arduino.get_temp_hum()
        temp_hum = [0, 0]
        rospy.loginfo("Depth: {} Temp: {} Hum: {}".format(depth, temp_hum[0], temp_hum[1]))
        pub.publish(depth, temp_hum[0], temp_hum[1])
        rate.sleep()

if __name__ == '__main__':
    try:
        startup()
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down data collector node")
