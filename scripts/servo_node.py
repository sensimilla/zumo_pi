#!/usr/bin/env python3
import rospy
import time
import math
import sys
import tf.transformations
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
import threading

class ServosDriverROSWrapper:
    def __init__(self):
        #TODO test if file exists
        try:
            self.servos = open('/dev/servoblaster', 'w')
        except: 
            rospy.logerr("Unable to open servoblaster, Is it running ?")
            sys.exit()
        
        rospy.Subscriber("/head/cmd_vel", Twist, self.cmd_vel_callback)

        self.lock = threading.Lock()
        self.JointStatePub = rospy.Publisher('~joint_state_servos', JointState, queue_size=1)

        #  servo range in radians
        self.servos_max = 0.75
        self.servos_min = -0.75

        self.servos_steps_centre = 132

        self.pan_pos = self.servos_steps_centre
        self.tilt_pos = self.servos_steps_centre
        
        # update rate for joint state publisher
        self.jsp_rate = 30

        period = rospy.Duration(1.0/self.jsp_rate)

        self.jsp_timer = rospy.Timer(period, self.publish_joint_state)

    def angle_to_steps(self, angle):
        return int(math.degrees(angle) * 1.03)
    
    def steps_to_angle(self, steps):
        return math.radians(steps / 1.03)
    
    def write_servos(self, pan, tilt):
        with self.lock:
            self.servos.write('1=' + str(pan) + '\n')
            self.servos.flush()
            self.pan_pos = pan
            self.servos.write('0=' + str(tilt) + '\n')
            self.servos.flush()
            self.tilt_pos = tilt

    def publish_joint_state(self, evt=None):
        js = JointState()
        js.header.frame_id = rospy.get_param("~frame_id", "base_link")
        js.header.stamp = rospy.Time.now()
        js.name = ['pan_joint', 'tilt_joint']
        js.position = self.get_positions()

        self.JointStatePub.publish(js)

    def get_positions(self):
        positions = []
        pan_angle = self.steps_to_angle(self.servos_steps_centre - self.pan_pos)
        tilt_angle = self.steps_to_angle(self.servos_steps_centre - self.tilt_pos)
        positions.append(pan_angle)
        positions.append(tilt_angle)
        return positions
    
    def stop(self):
        self.centre_servos()
        self.servos.close()
        self.jsp_timer.shutdown()
        rospy.loginfo("Servo Node Closed")
    
    def centre_servos(self):
        self.write_servos(self.servos_steps_centre, self.servos_steps_centre)

    def cmd_vel_callback(self, msg):
        #rospy.loginfo("Received a /head/cmd_vel message!")
        #rospy.loginfo("Linear Components: [%f, %f, %f]"%(msg.linear.x, msg.linear.y, msg.linear.z))
        #rospy.loginfo("Angular Components: [%f, %f, %f]"%(msg.angular.x, msg.angular.y, msg.angular.z))

        #rospy.loginfo(int(math.degrees(msg.angular.z)))

        # - degrees is to the rright  +ve to right
        # centre is 132
        # servoblaster step = 10us

        # Convert rotation in radians to servo steps away from centre
        tilt_steps = self.servos_steps_centre - self.angle_to_steps(msg.angular.y) 
        pan_steps = self.servos_steps_centre - self.angle_to_steps(msg.angular.z)

        #servo_1 = 132 - servo_1
        #servo_0 = 132 - servo_0
        
        self.write_servos(pan_steps, tilt_steps)


if __name__ == "__main__":
    rospy.init_node('zumo_servos', anonymous=False)
    servos_driver_wrapper =  ServosDriverROSWrapper()
    rospy.on_shutdown(servos_driver_wrapper.stop)
    rospy.loginfo("Servos controller is now initialised, ready to accept commands.")

    rospy.spin()
