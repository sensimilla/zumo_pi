#!/usr/bin/env python
# license removed for brevity
import rospy
#from std_msgs.msg import String
from sensor_msgs.msg._Range import Range
from zumo_pi.msg import Amg8833Raw
import time
import math
import busio
import board
import adafruit_amg88xx
import adafruit_vl53l0x

i2c = busio.I2C(board.SCL, board.SDA)
amg = adafruit_amg88xx.AMG88XX(i2c)
vl53 = adafruit_vl53l0x.VL53L0X(i2c)

def publish_message():
    rospy.init_node('zumo_pi', anonymous=False)
    amgPub = rospy.Publisher('~amg8833_raw', Amg8833Raw, queue_size=1)
    rangePub = rospy.Publisher('~vl503', Range, queue_size=1)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        rospy.loginfo('looping')
        flat_pixels = []
        for row in amg.pixels:
            for temp in row:
                flat_pixels.append(temp)

        amgReading = Amg8833Raw()
        amgReading.header.frame_id = rospy.get_param("~frame_id", "range_finder")
        amgReading.header.stamp = rospy.Time.now()
        amgReading.pixel_temps = flat_pixels
        amgReading.chip_temp = amg.temperature
        amgPub.publish(amgReading)

        reading = Range()
        reading.header.frame_id = rospy.get_param("~frame_id", "range_finder")
        reading.header.stamp = rospy.Time.now()
        reading.radiation_type = Range.INFRARED
        reading.field_of_view = rospy.get_param("~fov", math.radians(25.0))
        reading.min_range = rospy.get_param("~min_range", 0.03)
        reading.max_range = rospy.get_param("~max_range", 1.2)
        reading.range = vl53.range / 1000
        rangePub.publish(reading)

        rate.sleep()

if __name__ == '__main__':
    try:
        publish_message()
    except rospy.ROSInterruptException:
        pass
