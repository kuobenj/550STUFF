#!/usr/bin/env python

import rospy
from turtlesim.msg import Pose
def PoseCallback(msg):
 rospy.loginfo("I received position=(%4.2f, %4.2f), orientation(RAD)=%4.2f",msg.x, msg.y, msg.theta);

if __name__ == '__main__':
 sub = rospy.Subscriber('turtle1/pose', Pose, PoseCallback)
 rospy.init_node('subscriber_node')
 loop_rate = rospy.Rate(5)
 while not rospy.is_shutdown():
	 loop_rate.sleep()
