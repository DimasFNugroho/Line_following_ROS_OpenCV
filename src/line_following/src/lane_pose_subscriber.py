#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64

def callback_pos_x(data):
    rospy.loginfo(rospy.get_caller_id() + "pos x: %d", data.data)

def callback_pos_y(data):
    rospy.loginfo(rospy.get_caller_id() + "pos y: %d", data.data)

def callback_angle(data):
    rospy.loginfo(rospy.get_caller_id() + "angle: %f", data.data)

def callback_white_lane_available(self, data):
    rospy.loginfo(rospy.get_caller_id() + "pos x: %d", data.data)

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('lane_pose_subscriber', anonymous=True)

    rospy.Subscriber("lane_position_x", Float64, callback_pos_x)
    rospy.Subscriber("lane_position_x", Float64, callback_pos_y)
    rospy.Subscriber("lane_angle_twist", Float64, callback_angle)
    rospy.Subscriber("lane_availability", Float64, callback_white_lane_available)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
