#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, CompressedImage
from move_robot import MoveKobuki
from pid_control import PID
import time

from std_msgs.msg import Float64

class LineFollower(object):

    def __init__(self):

        rospy.logwarn("Init line Follower")
        self.movekobuki_object = MoveKobuki()
        self.pid_object = PID()
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.angle = 0.0
        self.white_lane_available = False
        rospy.Subscriber("lane_position_x", Float64, self.callback_pos_x)
        rospy.Subscriber("lane_position_x", Float64, self.callback_pos_y)
        rospy.Subscriber("lane_angle_twist", Float64, self.callback_angle)
        rospy.Subscriber(
                "lane_availability",
                Float64,
                self.callback_white_lane_available)
        time.sleep(5) #delay 5 seconds

    def callback_pos_x(self, data):
        rospy.loginfo(rospy.get_caller_id() + "pos x: %d", data.data)
        self.pos_x = data.data

    def callback_pos_y(self, data):
        rospy.loginfo(rospy.get_caller_id() + "pos y: %d", data.data)
        self.pos_y = data.data

    def callback_angle(self, data):
        rospy.loginfo(rospy.get_caller_id() + "angle: %f", data.data)
        self.angle = data.data

    def callback_white_lane_available(self, data):
        rospy.loginfo(rospy.get_caller_id() + "lane detected: " + data.data)
        self.white_lane_available = data.data

    def update_move_values(self, data):

        # Move the Robot , center it in the middle of the width 640 => 320:
        setPoint_value = width/2
        self.pid_object.setpoint_update(value=setPoint_value)

        twist_object = Twist();
        # if white lane is available, then move the robot
        if self.white_lane_available == True:
            twist_object.linear.x = 0.2;
        else:
            twist_object.linear.x = 0.0;

        # make it start turning
        self.pid_object.state_update(value=cx)
        effort_value = self.pid_object.get_control_effort()

        twist_object.angular.z = self.angle
        #rospy.logwarn("Twist =="+str(twist_object.angular.z))
        self.movekobuki_object.move_robot(twist_object)
        
        elapsed = time.time() - t
        print (elapsed)
    
    def clean_up(self):
        self.movekobuki_object.clean_class()

def main():
    rospy.init_node('line_following_node', anonymous=True)

    line_follower_object = LineFollower()

    rate = rospy.Rate(30) #originally 5
    ctrl_c = False
    def shutdownhook():
        # Works better than the rospy.is_shut_down()
        line_follower_object.clean_up()
        rospy.loginfo("shutdown time!")
        ctrl_c = True

    rospy.on_shutdown(shutdownhook)

    while not ctrl_c:
        rate.sleep()

if __name__ == '__main__':
    main()
