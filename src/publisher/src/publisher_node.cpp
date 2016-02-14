#include <ros/ros.h>
#include <geometry_msgs/Twist.h> 


int main(int argc, char **argv)
{
 ros::init(argc, argv, "publisher_node");  
 ros::NodeHandle n; 
 ros::Publisher pub = n.advertise<geometry_msgs::Twist>("turtle1/cmd_vel",1000);
 ros::Rate loop_rate(5);
 while(ros::ok()) 
 {
 geometry_msgs::Twist vel; 
 vel.linear.x = 1; 
 vel.angular.z = 1;
 pub.publish(vel);  
 ROS_INFO_STREAM("publishing velocity command: " << "linear (x)=" << vel.linear.x << ", angular (z)=" << vel.angular.z);  
 loop_rate.sleep();
 }
 return 0;
}
