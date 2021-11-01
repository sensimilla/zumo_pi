#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
import threading
import queue
import struct
import tf
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
import math
import sys


class MouseOdometer:
    def __init__(self):
        self.lock = threading.Lock()
        self.publishThread = None
        #self.mouse = open( "/dev/input/mice", "rb" )
        
        #  initial position in odometry frame
        self.x = 0
        self.y = 0
        self.th = 0

        self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=20)
        self.odom_broadcaster = tf.TransformBroadcaster()
        
        self.current_time = rospy.Time.now()
        self.last_time = rospy.Time.now()
        
        self.rate = rospy.get_param("~rate", 10)

        period = rospy.Duration(1.0/self.rate)

        self.publishThread = rospy.Timer(period, self.publishOdometry)

        #  accumulated deltas from mouse 
        self.delta_x = 0
        self.delta_y = 0
        self.delta_th = 0 

        # reading from the mouse is blocking IO so lets run in a seperate thread
        self.inputQueue = queue.Queue()
        #self.inputThread = threading.Thread(target=self.read_mouse, args=(self), daemon=True)
        self.inputThread = threading.Thread(target=self.read_mouse, daemon=True)
        
        self.inputThread.start()

    def read_mouse(self):
        mouse = open( "/dev/input/mice", "rb" )
        rospy.loginfo("Ready for mouse input")
        while not rospy.is_shutdown():
            buf = mouse.read(3)
            y,x = struct.unpack( "bb", buf[1:] )
            y = y * 0.00003125
            x = x * 0.00003125
            with self.lock:
                self.delta_x += x
                self.delta_y += y
        mouse.close()

    def publishOdometry(self, event=None):  # called from timer loop which adds event paramter
        self.current_time = rospy.Time.now()
        rospy.loginfo("sending odometry")
        # read and then reset accumlated deltas
        with self.lock:
            dx = self.delta_x
            dy = self.delta_y
            self.delta_x = 0
            self.delta_y = 0
        dt = (self.current_time - self.last_time).to_sec()
        dth = math.atan2(dy,dx)

        vx = dx / dt
        vy = dy / dt
        vth = dth / dt

        #  update position in Odometry frame
        self.x += dx
        self.y += dy
        self.th += dth

        self.last_time = self.current_time

        # since all odometry is 6DOF we'll need a quaternion created from yaw
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.th)

        # first, we'll publish the transform over tf
        # self.odom_broadcaster.sendTransform(
        #     (self.x, self.y, 0.),
        #     odom_quat,
        #     self.current_time,
        #     "odom",
        #     "base_footprint"
        # )

        # next, we'll publish the odometry message over ROS
        odom = Odometry()
        odom.header.stamp = self.current_time
        odom.header.frame_id = "odom"

        # set the position
        odom.pose.pose = Pose(Point(self.x, self.y, 0.), Quaternion(*odom_quat))

        # set the velocity
        odom.child_frame_id = "base_footprint"
        odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))

        # publish the message
        self.odom_pub.publish(odom)

    def stop(self):
        self.publishThread.shutdown()
        #self.mouse.close()
        rospy.loginfo("mouse odometer has shutdown")
        sys.exit("mouse odometer has shutdown")

if __name__ == "__main__":
    rospy.init_node('mouse_odometer', anonymous=False)
    mouse_odometer = MouseOdometer()
    rospy.on_shutdown(mouse_odometer.stop)
    rospy.loginfo("mouse_odometer is now initialised, sending Odometry messages.")

    rospy.spin()

 