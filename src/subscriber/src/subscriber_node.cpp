#include <ros/ros.h>
#include <turtlesim/Pose.h>

void PoseCallback(const turtlesim::Pose& msg)
{
 ROS_INFO_STREAM("I received " << "position=(" << msg.x << ", " << msg.y << "), " << "orientation(RAD)=" << msg.theta);
}

int main(int argc, char ** argv)
{
 ros::init(argc, argv, "subscriber_node"); 

 ros::NodeHandle n; 
 
 ros::Subscriber sub = n.subscribe("turtle1/pose",1000,&PoseCallback); 
 
 ros::spin(); //  enters a loop pumping callback functions until CTRL + C

 return 0;
}
