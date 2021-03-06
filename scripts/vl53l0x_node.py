#!/usr/bin/env python3

# based on https://github.com/sebastiengemme/vl53l0x-ros which seg faults when run
# TODO threading, live rate updating ? 
# TODO 1 shot service

import rospy
from sensor_msgs.msg._Range import Range
from std_srvs.srv import Trigger
from zumo_pi.srv import *
from zumo_pi.srv._StartRanging import StartRanging, StartRangingResponse, \
    StartRangingRequest
import time
import math
import VL53L0X
import threading


class VL53L0XDriverROSWrapper:

    def __init__(self):
        self.lock = threading.Lock()
        self.rangingThread = None

        self.tof = VL53L0X.VL53L0X()
        self.tof.open()

        self.rangePub = rospy.Publisher('~vl53l0x', Range, queue_size=1)

        rospy.Service("~stop_ranging", Trigger, self.callback_stop_ranging)
        rospy.Service("~start_ranging", StartRanging, self.callback_start_ranging)
        rospy.Service("~get_range", GetRange, self.callback_ranging_single)

        autostart = rospy.get_param("~autostart", True)

        mode = rospy.get_param("~mode", StartRangingRequest.VL53L0X_BEST_ACCURACY_MODE)

        #self.publish_timer = None

        #  read and publish rate
        #  0 = use rate based on mode timings
        self.rate = rospy.get_param("~rate", 0)

        if autostart:
            rospy.loginfo("Autostarting VL53L0X Time-of-Flight Ranging Sensor")
            self.start_ranging(mode)

    def start_ranging(self,mode):
        # instantiate reponse part of ROS service definition
        res = StartRangingResponse()
        with self.lock: 
            self.tof.start_ranging(mode)

            if self.rate == 0:
                timing = self.tof.get_timing()
                if (timing < 20000):
                    timing = 20000
                # get sensor timing info from microseconds
                period = rospy.Duration(timing / 1e+6)
            else:
                period = rospy.Duration(1.0/self.rate)

            self.rangingThread = rospy.Timer(period, self.read_range)
            rospy.loginfo("starting ranging in mode " + repr(mode))
            rospy.loginfo("Polling at " + repr(period.nsecs / 1e+9) + "s")
            rospy.loginfo("Target rate " + repr(1 / period.nsecs * 1e+9) + " Hz" )

    def stop_ranging(self):
        rospy.loginfo("VL53L0X Ranging Stopped")
        self.rangingThread.shutdown()
        # leave the tof ranging otherwise oneshot is unreliable
        # doesn't seem to use much/any? CPU on host
        

        #self.tof.stop_ranging()

    def read_range_single(self, mode):
        rospy.loginfo("VL53L0X single shot triggered")
        #self.tof.start_ranging(mode)
        #timing = self.tof.get_timing()
        #if (timing < 20000):
        #    timing = 20000
            # get sensor timing info from microseconds
        #period = rospy.Duration(timing / 1e+6)
        #rospy.sleep(period)
        with self.lock:
           range = self.read_range()
        #self.stop_ranging()
        return range


    def read_range(self, event=None):  # called from timer loop which adds event paramter
        # latency detecion
        #  works for a bit but then shows every loop as late
        #  performance issue or normal ??  
        #time_error = event.current_real - event.current_expected
        #error_ms = time_error.nsecs / 1e+6  #  milliseconds
        #if error_ms > 100:
            #rospy.logwarn("high thread latency detected: %ims" % error_ms)
 

        reading = Range()
        reading.header.frame_id = rospy.get_param("~frame_id", "vl53l0x_link")
        reading.header.stamp = rospy.Time.now()
        reading.radiation_type = Range.INFRARED
        reading.field_of_view = rospy.get_param("~fov", math.radians(25.0))
        reading.min_range = rospy.get_param("~min_range", 0.005)
        reading.max_range = rospy.get_param("~max_range", 2.0)

        with self.lock:
            reading.range = self.tof.get_distance() / 1000.0

        self.rangePub.publish(reading)
        return reading.range

    def stop(self):
        self.rangingThread.shutdown()
        self.tof.stop_ranging()
        self.tof.close()
        rospy.loginfo("VL53L0X Closed")

    def callback_stop_ranging(self, req):
        self.stop_ranging()
        return {"success": True, "message": "Ranging has been stopped"}
    
    def callback_start_ranging(self, req):
        self.start_ranging(req.mode)
        return {"success": True, "message": "Ranging has been started"}

    def callback_ranging_single(self, req):
        range = self.read_range_single(req.mode)
        return {"success": True, "range": range}

if __name__ == "__main__":
    rospy.init_node('vl53l0x', anonymous=False)
    tof_driver_wrapper = VL53L0XDriverROSWrapper()
    rospy.on_shutdown(tof_driver_wrapper.stop)
    rospy.loginfo("VL53L0X sensor is now started, ready to get commands.")

    rospy.spin()
