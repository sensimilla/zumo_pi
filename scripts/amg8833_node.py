#!/usr/bin/env python3
import rospy
from zumo_pi.msg import Amg8833Raw
from zumo_pi.srv import StartAmg8833
from std_srvs.srv import Trigger
import busio
import board
import adafruit_amg88xx
import threading

# TODO switch to Seeed amg library, don't trust adafruit one
# TODO add rate setting to start service

class AMG8833DriverROSWrapper:
    def __init__(self):
        self.lock = threading.Lock()
        self.publishThread = None
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.amg = adafruit_amg88xx.AMG88XX(self.i2c)

        self.amgPub = rospy.Publisher('~amg8833_raw', Amg8833Raw, queue_size=1)

        rospy.Service("~start_amg833", StartAmg8833, self.callback_start_amg)
        rospy.Service("~stop_amg833", Trigger, self.callback_stop_amg)

        autostart = rospy.get_param("~autostart", True)

        self.rate = 10

        self.set_rate(rospy.get_param("~rate", 1))

        self.is_enabled = False

        #self.publish_timer = None

        if autostart:
            rospy.loginfo("Autostarting AMG8833 8x8 IR Sensor")
            self.start_amg()

    def start_amg(self):
        self.is_enabled = True
        period = rospy.Duration(1.0/self.rate)
        self.publishThread = rospy.Timer(period, self.read_amg)

        rospy.loginfo("Started Reading AMG8833 8x8 IR Sensor ")
        rospy.loginfo("Target rate " + str(self.rate) + " Hz" )

    def stop_amg(self):
        self.publishThread.shutdown()
        self.is_enabled = False
        rospy.loginfo("AMG8833 Reading Stopped")

    def read_amg(self, event=None):  # called from timer loop which adds event paramter
        with self.lock:
            flat_pixels = []
            try:
                for row in self.amg.pixels:
                    for temp in row:
                        flat_pixels.append(temp)
                chip_temp = self.amg.temperature
            except:
                rospy.logerr("failed to read AMG833")
                return

        amgReading = Amg8833Raw()
        amgReading.header.frame_id = rospy.get_param("~frame_id", "amg8833_link")
        amgReading.header.stamp = rospy.Time.now()
        amgReading.pixel_temps = flat_pixels
        amgReading.chip_temp = chip_temp

        self.amgPub.publish(amgReading)

    def stop(self):
        self.publishThread.shutdown()
        rospy.loginfo("AMG833 Closed")

    def set_rate(self, new_rate):
        if new_rate == 0 or new_rate > 10:
            self.rate = 10
        else:
            self.rate = new_rate

    def callback_stop_amg(self, req):
        if self.is_enabled:
            self.stop_amg()
            return {"success": True, "message": "AMG8833 has been stopped"}
        else:
            return {"success": False, "message": "AMG8833 is already stopped"}
    
    def callback_start_amg(self, req):
        if not self.is_enabled:
            self.set_rate(req.rate)
            self.start_amg()
            return {"success": True, "message": "AMG8833 has been started"}
        else:
            return {"success": False, "message": "AMG8833 is already started"}

if __name__ == "__main__":
    rospy.init_node('amg8833', anonymous=False)
    amg_driver_wrapper = AMG8833DriverROSWrapper()
    rospy.on_shutdown(amg_driver_wrapper.stop)
    rospy.loginfo("AMG8833 8x8 IR sensor is now initialised, ready to get commands.")

    rospy.spin()