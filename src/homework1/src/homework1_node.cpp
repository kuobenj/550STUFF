#include <cmath>
#include <string>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h> 
#include <turtlesim/Pose.h>

static float local_x = 0;
static float local_y = 0;
static float local_theta = 0;
static float local_vel = 0;
static float local_omg = 0;

#define TEST_X_COORD 2.0
#define TEST_Y_COORD 2.0
#define PI 3.14159265
#define H_TOLERANCE 0.0001
#define D_TOLERANCE 0.1

void PoseCallback(const turtlesim::Pose& msg)
{
 ROS_INFO_STREAM("I received " << "position=(" << msg.x << ", " << msg.y << "), " << "orientation(RAD)=" << msg.theta);
 local_y = msg.y;
 local_x = msg.x;
 local_theta = msg.theta;
 if (local_theta >= PI)
 {
 	local_theta - (2*PI);
 }
 else if (local_theta <= -(PI))
 {
 	local_theta + (2*PI);
 }
 local_vel = msg.linear_velocity;
 local_omg = msg.angular_velocity;
 
}


int main(int argc, char **argv)
{
 if (argc < 3)
 {
 	ROS_INFO_STREAM("NOT ENOUGH ARGUMENTS");
 	return -1;
 }

 float x_coord = strtof(argv[1],NULL);
 float y_coord = strtof(argv[2],NULL);

 ros::init(argc, argv, "homework1_node");  
 ros::NodeHandle n; 
 ros::Publisher pub = n.advertise<geometry_msgs::Twist>("turtle1/cmd_vel",1000);
 ros::Subscriber sub = n.subscribe("turtle1/pose",1000,&PoseCallback); 
 ros::Rate loop_rate(5);

 float heading_target;
 float heading_error;
 float my_distance;

 float temp_lin_vel;
 float temp_ang_vel;

 while(ros::ok()) 
 {
 geometry_msgs::Twist vel; 
 ros::spinOnce();
 heading_target = atan2(y_coord-local_y, x_coord-local_x);
 heading_error = local_theta - heading_target;
 my_distance = sqrt((y_coord-local_y)*(y_coord-local_y)+(x_coord-local_x)*(x_coord-local_x));
 // temp_ang_vel = -0.5*heading_error;//+0.3*local_omg;
 // temp_lin_vel = 0.5*my_distance-0.3*local_vel; 
 if (abs(heading_error) > H_TOLERANCE)
 {
 	vel.linear.x = 0; 
 	vel.angular.z = -2*heading_error+0.3*local_omg;
 }
 else if (my_distance > D_TOLERANCE)
 {
 	vel.linear.x = 0.5*my_distance;//-0.3*local_vel; 
 	vel.angular.z = -0.3*heading_error+0.3*local_omg;
 }
 else
 	return 0;

 pub.publish(vel);
 // ROS_INFO_STREAM("publishing velocity command: " << "linear (x)=" << vel.linear.x << ", angular (z)=" << vel.angular.z);  
 // ROS_INFO_STREAM("I received " << "position=(" << local_x << ", " << local_y << "), " << "orientation(RAD)=" << local_theta);
 ROS_INFO_STREAM("SPEED: " << vel.linear.x << " TURN: " << vel.angular.z << " HEADING: " << heading_target << " ANGLE: "<< local_theta << " H_ERROR: " << heading_error << " DIST: " << my_distance);
 // ROS_INFO_STREAM("TARGET: (" << x_coord << "," << y_coord << ")SPEED: " << temp_lin_vel << " TURN: " << temp_ang_vel << " HEADING: " << heading_target << " ANGLE: "<< local_theta << " H_ERROR: " << heading_error << " DIST: " << my_distance);
 loop_rate.sleep();
 }
 return 0;
}
