#include <cmath>
#include <string>
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include <geometry_msgs/Twist.h> 
#include "polygon.h"

#define NUM_OBS_SIDES 4
#define D_S_GOAL (1.25) //how close to get before quadratic attraction
#define OB_INFUENCE (0.75) //how close to get before replusion starts
#define ATTRACTION_FACTOR (-2.0*0.5)
#define REPULSION_FACTOR (0.07*0.5)
#define AX (0.9)
#define AY (0.7)

static float local_x = 0;
static float local_y = 0;
static float local_theta = 0;
static float local_vel = 0;
static float local_omg = 0;

/**
 * This tutorial demonstrates simple receipt of position and speed of the Evarobot over the ROS system.
 */

/**
 * Callback function executes when new topic data comes.
 * Task of the callback function is to print data to screen.
 */
void chatterCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  ROS_INFO("Seq: [%d]", msg->header.seq);
  ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", msg->pose.pose.position.x,msg->pose.pose.position.y, msg->pose.pose.position.z);
  ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  ROS_INFO("Vel-> Linear: [%f], Angular: [%f]", msg->twist.twist.linear.x,msg->twist.twist.angular.z);
	local_x = msg->pose.pose.position.x;
	local_y = msg->pose.pose.position.y;
	local_theta = atan2(2*((msg->pose.pose.orientation.w)*(msg->pose.pose.orientation.z)+(msg->pose.pose.orientation.x)*(msg->pose.pose.orientation.y)),1-2*((msg->pose.pose.orientation.y)*(msg->pose.pose.orientation.y)+(msg->pose.pose.orientation.z)*(msg->pose.pose.orientation.z)));
	ROS_INFO("local_theta:%f",local_theta);
}

float to_omega(float fx, float fy, int i, float theta){

	float ax;
	float ay;

	if (i == 0)
	{
		ax = -AX;
		ay = -AY;
	}
	if (i == 1)
	{
		ax = AX;
		ay = -AY;
	}
	if (i == 2)
	{
		ax = AX;
		ay = AY;
	}
	if (i == 3)
	{
		ax = -AX;
		ay = AY;
	}

	return -fx*(ax*sin(theta)+ay*cos(theta))+fy*(ax*cos(theta)+ay*sin(theta));
}

float simple_dist(float x1, float y1, float x2, float y2)
{
	return sqrt(((x1-x2)*(x1-x2))+((y1-y2)*(y1-y2)));
}

int main(int argc, char **argv)
{
  	ros::init(argc, argv, "homework2_node");

  	ros::NodeHandle n;

 	ros::Publisher pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1000);
  	ros::Subscriber sub = n.subscribe("odom", 1000, chatterCallback);
  	ros::Rate loop_rate(60);

  	//hardcoded obstacles
  	My_Polygon ob1(2.5,2.5,1.0,1.0);
  	My_Polygon ob2(5.0,1.0,0.75,0.75);
  	My_Polygon ob3(3.0,5.0,0.5,0.5);
  	My_Polygon ob4(0.0,3.0,0.25,0.25);
  	My_Polygon wall1(2.5,-1.5,10.0,0.5);
  	My_Polygon wall2(6.5,2.5,0.5,10.0);
  	My_Polygon wall3(2.5,6.5,10.0,0.5);
  	My_Polygon wall4(-1.5,2.5,0.5,10.0);

  	My_Polygon* my_polygons[8] = {&ob1, &ob2, &ob3, &ob4, &wall1, &wall2, &wall3, &wall4};

  	// My_Polygon my_robot(0.0,0.0,0.580,0.380);
  	My_Polygon my_robot(0.0,0.0,AX,AY);

  	float goal_x = 5.0;
  	float goal_y = 5.0;
  	while(ros::ok())
  	{
  		ros::spinOnce();

  		geometry_msgs::Twist vel;

	  	float fx = 0.0;
  		float fy = 0.0;
	  	float xvel = 0.0;
  		float yvel = 0.0;
  		float omega = 0.0;

  		my_robot.update_center(local_x, local_y, local_theta);

  		//for each point the jacobian * forces yeilds

  		//ux = fx
  		//uy = fy
  		//utheta = -fx*(ax*sin(theta)+ay*cos(theta))+fy*(ax*cos(theta)+ay*sin(theta))

  		for (int i = 0; i < 4; i++)
  		{
	   		// float dist_to_goal = my_robot.distance_to_point(goal_x,goal_y);
	   		float dist_to_goal = simple_dist(my_robot.corners[i][0],my_robot.corners[i][0],goal_x,goal_y);
	  		if (dist_to_goal < D_S_GOAL)
	  		{
	  			fx = ATTRACTION_FACTOR*(local_x-goal_x);
	  			fy = ATTRACTION_FACTOR*(local_y-goal_y);
	  		}
	  		else
	  		{
	  			fx = D_S_GOAL*ATTRACTION_FACTOR*(local_x-goal_x)/(sqrt((local_x-goal_x)*(local_x-goal_x)+(local_y-goal_y)*(local_y-goal_y)));
	  			fy = D_S_GOAL*ATTRACTION_FACTOR*(local_y-goal_y)/(sqrt((local_x-goal_x)*(local_x-goal_x)+(local_y-goal_y)*(local_y-goal_y)));
	  		}

	  		xvel += fx/4.0;
	  		yvel += fy/4.0;
	  		omega += to_omega(fx,fy,i,local_theta)/4.0;
	  	}

  			// printf("attraction (%f, %f) vel mag: %f\n", xvel, yvel, sqrt(xvel*xvel+yvel*yvel));


  // 		ROS_INFO_STREAM("ob1 corners (" << my_polygons[0]->corners[0][0] << "," << my_polygons[0]->corners[0][1] << ")" <<
  // 										my_polygons[0]->corners[1][0] << "," << my_polygons[0]->corners[1][1] << ")" <<
  // 										my_polygons[0]->corners[2][0] << "," << my_polygons[0]->corners[2][1] << ")" <<
  // 										my_polygons[0]->corners[3][0] << "," << my_polygons[0]->corners[3][1] << ")" <<
  // 										my_polygons[0]->corners[4][0] << "," << my_polygons[0]->corners[4][1] << ")" << "\n"
  // 			);
  		float temp_x;
  		float temp_y;
  		// my_polygons[0]->closest_point(0.0,0.0,temp_x,temp_y);
		// ROS_INFO_STREAM("(" << 0.0 << " , "<< 0.0 << ") to ob1: " << my_polygons[0]->distance_to_point(0.0,0.0) << " closest_point:" << temp_x << " , " << temp_y << "\n");
		// my_polygons[0]->closest_point(5.0,0.0,temp_x,temp_y);
		// ROS_INFO_STREAM("(" << 5.0 << " , "<< 0.0 << ") to ob1: " << my_polygons[0]->distance_to_point(5.0,0.0) << " closest_point:" << temp_x << " , " << temp_y << "\n");
		// my_polygons[0]->closest_point(5.0,5.0,temp_x,temp_y);
		// ROS_INFO_STREAM("(" << 5.0 << " , "<< 5.0 << ") to ob1: " << my_polygons[0]->distance_to_point(5.0,5.0) << " closest_point:" << temp_x << " , " << temp_y << "\n");
		// my_polygons[0]->closest_point(0.0,5.0,temp_x,temp_y);
		// ROS_INFO_STREAM("(" << 0.0 << " , "<< 5.0 << ") to ob1: " << my_polygons[0]->distance_to_point(0.0,5.0) << " closest_point:" << temp_x << " , " << temp_y << "\n");
		// my_polygons[0]->closest_point(2.5,0.0,temp_x,temp_y);
		// ROS_INFO_STREAM("(" << 2.5 << " , "<< 0.0 << ") to ob1: " << my_polygons[0]->distance_to_point(2.5,0.0) << " closest_point:" << temp_x << " , " << temp_y << "\n");
		// my_polygons[0]->closest_point(2.5,0.9,temp_x,temp_y);
		// ROS_INFO_STREAM("(" << 2.5 << " , "<< 0.9 << ") to ob1: " << my_polygons[0]->distance_to_point(2.5,0.9) << " closest_point:" << temp_x << " , " << temp_y << "\n");
		// my_polygons[0]->closest_point(3.17042,2.02971,temp_x,temp_y);
		// ROS_INFO_STREAM("(" << 3.17042 << " , "<< 2.02971 << ") to ob1: " << my_polygons[0]->distance_to_point(3.17042,2.02971) << " closest_point:" << temp_x << " , " << temp_y << "\n");

  		for (int j = 0; j < 8; j++)
  		{
  			for (int i = 0; i < 4; i++)
	  		{
	  			if (j == 0)
	  			{
		  			// ROS_INFO_STREAM("my_robot corner " << i+1  << " at (" << my_robot.corners[i][0] << " , "<< my_robot.corners[i][1] << ") to ob1: " << my_polygons[0]->distance_to_point(my_robot.corners[i][0],my_robot.corners[i][1]) << " at " <<"\n");
					my_polygons[0]->closest_point(my_robot.corners[i][0],my_robot.corners[i][1],temp_x,temp_y);
					ROS_INFO_STREAM("(" << my_robot.corners[i][0] << " , "<< my_robot.corners[i][1] << ") to ob1: " << my_polygons[0]->distance_to_point(my_robot.corners[i][0],my_robot.corners[i][1]) << " closest_point:" << temp_x << " , " << temp_y << "\n");
		  			// ROS_INFO_STREAM(my_polygons[0]->corners[0][0] << " , " << my_polygons[0]->corners[0][1] << "\n" <<
		  			// 				my_polygons[0]->corners[1][0] << " , " << my_polygons[0]->corners[1][1] << "\n" <<
		  			// 				my_polygons[0]->corners[2][0] << " , " << my_polygons[0]->corners[2][1] << "\n" <<
		  			// 				my_polygons[0]->corners[3][0] << " , " << my_polygons[0]->corners[3][1] << "\n" <<
		  			// 				my_polygons[0]->corners[4][0] << " , " << my_polygons[0]->corners[4][1] << "\n");
		  		}

	  			if (my_polygons[j]->distance_to_point(my_robot.corners[i][0],my_robot.corners[i][1]) > OB_INFUENCE)
	  			{
	  				fx = 0.0;
	  				fy = 0.0;
	  			}
	  			else
	  			{
	  				//figure out closest point
	  				float closest_x;
	  				float closest_y;
	  				my_polygons[j]->closest_point(my_robot.corners[i][0],my_robot.corners[i][1],closest_x,closest_y);
	  				//take gradient which is controlpoint-closestpoint/distance_between_the_two and multiply by some factor
	  				float grad_x;
	  				float grad_y;
	  				grad_x = (my_robot.corners[i][0]-closest_x)/my_polygons[j]->distance_to_point(my_robot.corners[i][0],my_robot.corners[i][1]);
	  				grad_y = (my_robot.corners[i][1]-closest_y)/my_polygons[j]->distance_to_point(my_robot.corners[i][0],my_robot.corners[i][1]);

	  				fx = grad_x*REPULSION_FACTOR;
	  				fy = grad_y*REPULSION_FACTOR;
	  			}
	  			xvel += fx;
	  			yvel += fy;
	  			omega += to_omega(fx,fy,i,local_theta);

	  		}
	  	}

	  	ROS_INFO_STREAM("velocity " << xvel << " , " << yvel << " \n");

  		vel.linear.x = xvel;
 		vel.linear.y = yvel;
 		// vel.angularx = xvel;
 		// vel.linear.y = yvel;
 		vel.angular.z = omega;

 		pub.publish(vel);

 		loop_rate.sleep();
 		// return 0;

 		// if ((abs(local_x-goal_x)<0.01) && (abs(local_y-goal_y)<0.01))
 		// {
 		// 	vel.linear.x = 0.0;
 		// 	vel.linear.y = 0.0;
 		// 	return 0;
 		// }
  	}

  	return 0;
}
