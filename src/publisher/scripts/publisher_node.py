#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

if __name__ == '__main__':
 pub = rospy.Publisher('turtle1/cmd_vel',Twist,queue_size=1000)
 rospy.init_node('publisher_node')
 loop_rate = rospy.Rate(5)

 while not rospy.is_shutdown():
	 vel=Twist()
	 vel.linear.x = 1.0
	 vel.angular.z = 1.0
	 pub.publish(vel)
	 rospy.loginfo("publishing velocity command: linear (x)=%4.2f, angular (z)=%4.2f",vel.linear.x,vel.angular.z);
	 loop_rate.sleep()
