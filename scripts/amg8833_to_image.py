#!/usr/bin/env python

import rospy
from zumo_pi.msg import Amg8833Raw
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 as cv
import numpy as np
from colour import Color
 
img = np.zeros((64,64),dtype=np.uint8)

#  flag to tell main thread that a new image is ready to send
new_image = False

def callback(data):
    global img
    global new_image
    pixel_values = []
    for p in data.pixel_temps:
       #  multiply pixel temps by four as measuremnt is accurate to 1/4 degree
       pixel_values.append(np.uint8(p * 4))

    #  reshape flat array of pixels into 8x8 
    img_orig = np.array(np.array(pixel_values)).reshape(8, 8)

    #  zoom in on image no interpolation just more pixels
    img = np.repeat(np.repeat(img_orig,8,axis=0), 8, axis=1)

    #TODO more processing...

    new_image = True

if __name__ == '__main__':
    rospy.init_node('amg8833_to_image', anonymous=False)

    #  gets raw pixel temperature from robot as float list
    sub = rospy.Subscriber("/amg8833/amg8833_raw", Amg8833Raw, callback)

    #  publish the processed image
    pub = rospy.Publisher("/amg8833/amg8833_image", Image, queue_size=1)

    # max sensor max  is 10Hz
    rate=rospy.Rate(10)

    # ROS <-> OpenCV bridge
    bridge = CvBridge()

    while not rospy.is_shutdown():
        if new_image:
            # convert img to ROS Image message
            image_message = bridge.cv2_to_imgmsg(img, encoding="passthrough")
            # send it
            pub.publish(image_message)
            new_image = False
        rate.sleep()

