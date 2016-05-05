#include <cmath>
#include <string>
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include <geometry_msgs/Twist.h>
#include <fcl/shape/geometric_shapes.h>
#include <fcl/shape/geometric_shapes_utility.h>
#include <fcl/narrowphase/narrowphase.h>
#include <fcl/collision.h>
#include <list>
#include <vector>
#include <memory>
#include <functional>
#include <stdexcept>
#include <stdlib.h>
#include <iostream>

// #include <pr2_controllers_msgs/JointTrajectoryAction.h>
// #include <actionlib/client/simple_action_client.h>
//https://github.com/wnowak/youbot_ros_samples/blob/master/youbot_ros_hello_world/src/youbot_ros_hello_world.cpp
#include "boost/units/systems/si.hpp"
#include "boost/units/io.hpp"
#include "brics_actuator/JointPositions.h"

#include <trajectory_msgs/JointTrajectory.h>
#include <std_msgs/Header.h>

// typedef actionlib::SimpleActionClient< pr2_controllers_msgs::JointTrajectoryAction > TrajClient;

#define PI 3.14159

// #define RRT_STEP_SIZE 0.1
// #define GOAL_MAX_DIST 0.1
#define NUM_OBS 12
#define NUM_DRIVE_RRT 10000
#define NUM_ARM_RRT 10000

// ARM PARAMETERS
#define SELECT_ARM
#define RRT_STEP_SIZE 0.07
#define GOAL_MAX_DIST 0.08


#define BASE_X_OFFSET (-0.0014)
#define BASE_Y_OFFSET (0.0)
#define BASE_Z_OFFSET (0.0956)

#define ARM_2_X_OFFSET (0.0740)
#define ARM_2_Y_OFFSET (0.0)
#define ARM_2_Z_OFFSET (-0.0412)


#define ARM_3_X_OFFSET (0.0662)
#define ARM_3_Y_OFFSET (0.0)
#define ARM_3_Z_OFFSET (0.0370)


#define ARM_4_X_OFFSET (0.0869)
#define ARM_4_Y_OFFSET (0.0)
#define ARM_4_Z_OFFSET (0.0)

#define JOINT_1_OFFSET (2.95)
#define JOINT_2_OFFSET (1.13)
#define JOINT_3_OFFSET (-2.55)
#define JOINT_4_OFFSET (1.79)
#define JOINT_5_OFFSET (2.875)


using namespace std;
using namespace fcl;

static float local_x = 0;
static float local_y = 0;
static float local_theta = 0;
static float local_vel = 0;
static float local_omg = 0;

class Node
{

public:
    Node(Vec3f* state, Node* parent = NULL) {
        _parent = parent;
        _state = state;

        visited = false;
   		dist = -1;
   		previous = NULL;

        if (_parent) {
            _parent->_children.push_back(this);
        }
    }

    Node* parent() { return _parent; }

    /**
     * Gets the number of ancestors (parent, parent's parent, etc) that
     * the node has.
     * Returns 0 if it doesn't have a parent.
     */
    int depth() {
        int n = 0;
        for (Node* ancestor = _parent; ancestor != NULL;
             ancestor = ancestor->_parent) {
            n++;
        }
        return n;
    }

    /**
     * The @state property is the point in the state-space that this
     * Node represents.  Generally this is a vector (could be 2d, 3d, etc)
     */
    Vec3f* state() { return _state; }

    bool visited;
   	float dist;
   	Node* previous;

private:
    Vec3f* _state;
    std::list<Node*> _children;
    Node* _parent;
};

/**
 * A list of all Node objects in the tree.
 */
// std::vector<Node*> _nodes;
 Node* _nodes[20000];
 Node* _the_path[20000];
 int node_count = 0;

Vec3f* drive_source;
Vec3f* drive_dest;
Vec3f* arm_source;
Vec3f* arm_dest;

float simple_dist_orig(float x1, float y1, float x2, float y2)
{
	return sqrt(((x1-x2)*(x1-x2))+((y1-y2)*(y1-y2)));
}

float simple_dist_drive(Vec3f* x, Vec3f* y)
{
	// ROS_INFO_STREAM("x location " << x <<" \n");
	// ROS_INFO_STREAM("y location " << y <<" \n");
	if ((x==NULL) || (y==NULL))
	{
		return -1.0;
	}


	float x1 = (*x)[0];
	float y1 = (*x)[1];
	float x2 = (*y)[0];
	float y2 = (*y)[1];

	// float x1 = x->data[0];
	// float y1 = x->data[1];
	// float x2 = y->data[0];
	// float y2 = y->data[1];
	return sqrt(((x1-x2)*(x1-x2))+((y1-y2)*(y1-y2)));

	// Vec3f* temp = new Vec3f();
	// *temp = *x-*y;

	// return temp->length();
}

float simple_dist_arm(Vec3f* x, Vec3f* y)
{
	// ROS_INFO_STREAM("x location " << x <<" \n");
	// ROS_INFO_STREAM("y location " << y <<" \n");
	if ((x==NULL) || (y==NULL))
	{
		return -1.0;
	}


	// float x1 = (*x)[0];
	// float y1 = (*x)[1];
	// float x2 = (*y)[0];
	// float y2 = (*y)[1];

	// // float x1 = x->data[0];
	// // float y1 = x->data[1];
	// // float x2 = y->data[0];
	// // float y2 = y->data[1];
	// return sqrt(((x1-x2)*(x1-x2))+((y1-y2)*(y1-y2)));

	Vec3f* temp = new Vec3f();
	*temp = *x-*y;

	return temp->length();
}

float random_float_location()
{
	float r = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
	return r*5.0;
}

float random_float_angle()
{
	float r = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
	r *= 2*PI;
	return r-PI;
}


Node* nearest(Vec3f* state) {
    float bestDistance = -1;
    Node* best = NULL;
    // if (_dimension == 2)
    // {
       	// for (Node* other : _nodes) {
    	// for (std::list<Node*>::iterator it = _nodes.begin(); it != _nodes.end(); it++)
    	// for (std::list<Node*>::iterator it = _nodes.begin(); it != _nodes.end(); it++)
    	for (int i = 0; i < node_count; i++)
    	{
    		#ifndef SELECT_ARM
            float dist = simple_dist_drive(_nodes[i]->state(),state); //_stateSpace->distance(other->state(), state);
            #else
            float dist = simple_dist_arm(_nodes[i]->state(),state);
            #endif
            if (bestDistance < 0 || dist < bestDistance) {
                bestDistance = dist;
                best = _nodes[i];
            }
        }
    // }

    return best;
}


Vec3f* intermediateStateFunction(Vec3f* source, Vec3f* state, float stepSize)
{
	Vec3f* intermediateState = new Vec3f();
	*intermediateState = (*state)-(*source);
	// Node* intermediateState = new Node(&temp)
	intermediateState->normalize();
	*intermediateState *= stepSize;

	*intermediateState += *source;

	return intermediateState;
}

bool transitionValid(Vec3f* source, Vec3f* state)
{	
  static const int num_max_contacts = std::numeric_limits<int>::max();
  static const bool enable_contact = true;
  fcl::CollisionResult result;
  fcl::CollisionRequest request(num_max_contacts,
                                enable_contact);

	//https://github.com/flexible-collision-library/fcl/blob/master/test/general_test.cpp
  	//hardcoded obstacles

  	boost::shared_ptr<Box> obs_1(new Box(2.0,2.0,1.0));
  	boost::shared_ptr<Box> obs_2(new Box(0.75,0.75,0.75));
  	boost::shared_ptr<Box> obs_3(new Box(0.75,0.75,0.75));
  	boost::shared_ptr<Box> obs_4(new Box(0.5,0.5,0.5));
  	boost::shared_ptr<Box> wall_1(new Box(8.0,0.5,0.5));
  	boost::shared_ptr<Box> wall_2(new Box(0.5,8.0,0.5));
  	boost::shared_ptr<Box> wall_3(new Box(8.0,0.5,0.5));
  	boost::shared_ptr<Box> wall_4(new Box(0.5,0.5,8.0));
  	boost::shared_ptr<Box> obs_5_1(new Box(0.4,0.4,0.2));
  	boost::shared_ptr<Box> obs_5_2(new Box(0.05+0.01,0.05+0.01,0.15+0.01));
  	boost::shared_ptr<Box> obs_5_3(new Box(0.05+0.01,0.05+0.01,0.15+0.01));
  	boost::shared_ptr<Box> obs_5_4(new Box(0.25+0.01,0.25+0.01,0.15+0.01));

  	Transform3f tf_obs_1;
  	Transform3f tf_obs_2;
  	Transform3f tf_obs_3;
  	Transform3f tf_obs_4;
  	Transform3f tf_wall_1;
  	Transform3f tf_wall_2;
  	Transform3f tf_wall_3;
  	Transform3f tf_wall_4;
  	Transform3f tf_obs_5_1;
  	Transform3f tf_obs_5_2;
  	Transform3f tf_obs_5_3;
  	Transform3f tf_obs_5_4;

  	tf_obs_1.setTranslation(Vec3f(2.5,2.5,0.5));
  	tf_obs_2.setTranslation(Vec3f(5.0,1.0,0.375));
  	tf_obs_3.setTranslation(Vec3f(3.0,5.0,0.375));
  	tf_obs_4.setTranslation(Vec3f(0,3.0,0.25));
  	tf_wall_1.setTranslation(Vec3f(2.5,-1.25,0.25));
  	tf_wall_2.setTranslation(Vec3f(6.25,2.5,0.25));
  	tf_wall_3.setTranslation(Vec3f(2.5,6.25,0.25));
  	tf_wall_4.setTranslation(Vec3f(-1.25,2.5,0.25));
  	tf_obs_5_1.setTranslation(Vec3f(0.6,0.0,0.1));
  	tf_obs_5_2.setTranslation(Vec3f(0.5,0.1,0.275));
  	tf_obs_5_3.setTranslation(Vec3f(0.5,-0.1,0.275));
  	tf_obs_5_4.setTranslation(Vec3f(0.5,0.0,0.375));

  	//Robot stuff
	boost::shared_ptr<Box> base_combined(new Box(0.5701,0.3570,0.1600));
 	boost::shared_ptr<Box> arm_1(new Box(0.1697,0.1699,0.1060));
 	boost::shared_ptr<Box> arm_2(new Box(0.22,0.0745,0.0823));
 	boost::shared_ptr<Box> arm_3(new Box(0.1920,0.0591,0.0750));
 	boost::shared_ptr<Box> arm_4(new Box(0.2401,0.0581,0.0987));

	Transform3f tf_base_combined;
 	Transform3f tf_arm_1;
 	Transform3f tf_arm_2;
 	Transform3f tf_arm_3;
 	Transform3f tf_arm_4;

  	for (int j = 0; j < 19; j++)
  	{
  		int total_contacts = 0;
	  	vector<Contact> contacts;

	  	#ifndef SELECT_ARM
	 	tf_base_combined.setTranslation((*state/20.0*j)+(*source/20.0*(20-j))+Vec3f(BASE_X_OFFSET,BASE_Y_OFFSET,BASE_Z_OFFSET));

	  	CollisionObject co1(obs_1, tf_obs_1);
	  	CollisionObject co2(obs_2, tf_obs_1);
	  	CollisionObject co3(obs_3, tf_obs_1);
	  	CollisionObject co4(obs_4, tf_obs_1);
	  	CollisionObject co5(wall_1, tf_wall_1);
	  	CollisionObject co6(wall_2, tf_wall_2);
	  	CollisionObject co7(wall_3, tf_wall_3);
	  	CollisionObject co8(wall_4, tf_wall_4);
	  	CollisionObject co9(obs_5_1, tf_obs_5_1);

	  	CollisionObject robot(base_combined, tf_base_combined);

		fcl::collide(&robot, &co1, request, result);
		result.getContacts(contacts);
		total_contacts += contacts.size();

		fcl::collide(&robot, &co2, request, result);
		result.getContacts(contacts);
		total_contacts += contacts.size();

		fcl::collide(&robot, &co3, request, result);
		result.getContacts(contacts);
		total_contacts += contacts.size();

		fcl::collide(&robot, &co4, request, result);
		result.getContacts(contacts);
		total_contacts += contacts.size();

		fcl::collide(&robot, &co5, request, result);
		result.getContacts(contacts);
		total_contacts += contacts.size();

		fcl::collide(&robot, &co6, request, result);
		result.getContacts(contacts);
		total_contacts += contacts.size();

		fcl::collide(&robot, &co7, request, result);
		result.getContacts(contacts);
		total_contacts += contacts.size();

		fcl::collide(&robot, &co8, request, result);
		result.getContacts(contacts);
		total_contacts += contacts.size();

		fcl::collide(&robot, &co9, request, result);
		result.getContacts(contacts);
		total_contacts += contacts.size();
		#else
		// float /*temp_*/theta = 0.1;
		// float /*temp_*/d = 0.1;
		// float /*temp_*/a = 0.1;
		// float /*temp_*/alpha = 0.1;
		Transform3f temp_tf;
		Transform3f temp_tf_accum;
		Vec3f test_angles = (*state/10.0*j)+(*source/10.0*(10-j));//+Vec3f(0.0-(PI/2),0.0,0.0);
		// Vec3f test_angles = Vec3f(0.628-(PI/2.0),1.964,-1.047);
		// Vec3f test_angles = Vec3f(0.628,1.964,-1.047);
		// Vec3f test_angles = Vec3f(0.0-(PI/2),0.0,0.0);
		// Vec3f test_angles = Vec3f(0.0,0.0,0.0);
		//sudden revelation remember to do the box offset first
		// temp_tf.setRotation(Matrix3f(cos(theta),-sin(theta),0.0,sin(theta),cos(theta),0.0,0.0,0.0,1.0));
		// temp_tf_accum = temp_tf_accum*temp_tf;
		// temp_tf.setTranslation(Vec3f(0.0,0.0,d));
		// temp_tf_accum = temp_tf_accum*temp_tf;
		// temp_tf.setTranslation(Vec3f(a,0.0,0.0));
		// temp_tf_accum = temp_tf_accum*temp_tf;
		// temp_tf.setRotation(Matrix3f(1.0,0.0,0.0,0.0,cos(alpha),-sin(alpha),0.0,sin(alpha),cos(alpha)));
		// temp_tf_accum = temp_tf_accum*temp_tf;
		temp_tf_accum.setIdentity();
	 	// tf_base_combined.setTranslation(Vec3f(BASE_X_OFFSET,BASE_Y_OFFSET,BASE_Z_OFFSET));
	 	tf_arm_2.setTranslation(Vec3f(ARM_2_X_OFFSET,ARM_2_Y_OFFSET,ARM_2_Z_OFFSET));
	 	tf_arm_3.setTranslation(Vec3f(ARM_3_X_OFFSET,ARM_3_Y_OFFSET,ARM_3_Z_OFFSET));
	 	tf_arm_4.setTranslation(Vec3f(ARM_4_X_OFFSET,ARM_4_Y_OFFSET,ARM_4_Z_OFFSET));

	 // 	temp_tf.setRotation(Matrix3f(cos(0.0),-sin(0.0),0.0,sin(0.0),cos(0.0),0.0,0.0,0.0,1.0));
		// temp_tf_accum *= temp_tf;
		temp_tf.setIdentity();
		temp_tf.setTranslation(Vec3f(0.0,0.0, 0.046+0.084+0.115));
		temp_tf_accum *= temp_tf;
		temp_tf.setIdentity();
		temp_tf.setTranslation(Vec3f(0.143+0.024,0.0,0.0));
		temp_tf_accum *= temp_tf;
		temp_tf.setIdentity();
		temp_tf.setRotation(Matrix3f(1.0,0.0,0.0,0.0,cos(PI),-sin(PI),0.0,sin(PI),cos(PI)));
		temp_tf_accum *= temp_tf;

		// temp_tf.setRotation(Matrix3f(cos(0.0),-sin(0.0),0.0,sin(0.0),cos(0.0),0.0,0.0,0.0,1.0));
		// temp_tf_accum *= temp_tf;
		// temp_tf.setTranslation(Vec3f(0.0,0.0, 0.046+0.084+0.115));
		// temp_tf_accum *= temp_tf;
		temp_tf.setIdentity();
		temp_tf.setTranslation(Vec3f(0.033,0.0,0.0));
		temp_tf_accum *= temp_tf;
		temp_tf.setIdentity();
		temp_tf.setRotation(Matrix3f(1.0,0.0,0.0,0.0,cos(PI/2.0),-sin(PI/2.0),0.0,sin(PI/2.0),cos(PI/2.0)));
		temp_tf_accum *= temp_tf;
		tf_arm_1 = temp_tf_accum*tf_arm_1;

		temp_tf.setIdentity();
		// temp_tf.setRotation(Matrix3f(cos(test_angles[0]-(PI/2)),-sin(test_angles[0]-(PI/2)),0.0,sin(test_angles[0]-(PI/2)),cos(test_angles[0]-(PI/2)),0.0,0.0,0.0,1.0));
		temp_tf.setRotation(Matrix3f(cos(test_angles[0]+(PI/2)),-sin(test_angles[0]+(PI/2)),0.0,sin(test_angles[0]+(PI/2)),cos(test_angles[0]+(PI/2)),0.0,0.0,0.0,1.0));
		// temp_tf.setRotation(Matrix3f(cos(test_angles[0]),-sin(test_angles[0]),0.0,sin(test_angles[0]),cos(test_angles[0]),0.0,0.0,0.0,1.0));
		temp_tf_accum = temp_tf_accum*temp_tf;
		// temp_tf.setTranslation(Vec3f(0.0,0.0,d));
		// temp_tf_accum = temp_tf_accum*temp_tf;
		temp_tf.setIdentity();
		temp_tf.setTranslation(Vec3f( 0.155,0.0,0.0));
		temp_tf_accum = temp_tf_accum*temp_tf;
		// temp_tf.setRotation(Matrix3f(1.0,0.0,0.0,0.0,cos(alpha),-sin(alpha),0.0,sin(alpha),cos(alpha)));
		// temp_tf_accum = temp_tf_accum*temp_tf;
		tf_arm_2 = temp_tf_accum * tf_arm_2;

		temp_tf.setIdentity();
		temp_tf.setRotation(Matrix3f(cos(test_angles[1]),-sin(test_angles[1]),0.0,sin(test_angles[1]),cos(test_angles[1]),0.0,0.0,0.0,1.0));
		temp_tf_accum = temp_tf_accum*temp_tf;
		// temp_tf.setTranslation(Vec3f(0.0,0.0,d));
		// temp_tf_accum = temp_tf_accum*temp_tf;
		temp_tf.setIdentity();
		temp_tf.setTranslation(Vec3f(0.135,0.0,0.0));
		temp_tf_accum = temp_tf_accum*temp_tf;
		// temp_tf.setRotation(Matrix3f(1.0,0.0,0.0,0.0,cos(alpha),-sin(alpha),0.0,sin(alpha),cos(alpha)));
		// temp_tf_accum = temp_tf_accum*temp_tf;
		tf_arm_3 = temp_tf_accum * tf_arm_3;

		temp_tf.setIdentity();
		temp_tf.setRotation(Matrix3f(cos(test_angles[2]),-sin(test_angles[2]),0.0,sin(test_angles[2]),cos(test_angles[2]),0.0,0.0,0.0,1.0));
		temp_tf_accum = temp_tf_accum*temp_tf;
		// temp_tf.setTranslation(Vec3f(0.0,0.0,d));
		// temp_tf_accum = temp_tf_accum*temp_tf;
		temp_tf.setIdentity();
		temp_tf.setTranslation(Vec3f(0.1136,0.0,0.0));
		temp_tf_accum = temp_tf_accum*temp_tf;
		temp_tf.setIdentity();
		temp_tf.setRotation(Matrix3f(1.0,0.0,0.0,0.0,cos(-PI/2),-sin(-PI/2),0.0,sin(-PI/2),cos(-PI/2)));
		temp_tf_accum = temp_tf_accum*temp_tf;
		tf_arm_4 = temp_tf_accum *tf_arm_4;
		// ROS_INFO_STREAM("Rotation Matrix\n");
		// ROS_INFO_STREAM("[" << temp_tf_accum.getRotation()(0,0) << " " << temp_tf_accum.getRotation()(0,1) << " " << temp_tf_accum.getRotation()(0,2) << "]\n");
		// ROS_INFO_STREAM("[" << temp_tf_accum.getRotation()(1,0) << " " << temp_tf_accum.getRotation()(1,1) << " " << temp_tf_accum.getRotation()(1,2) << "]\n");
		// ROS_INFO_STREAM("[" << temp_tf_accum.getRotation()(2,0) << " " << temp_tf_accum.getRotation()(2,1) << " " << temp_tf_accum.getRotation()(2,2) << "]\n");
		// ROS_INFO_STREAM("Translation: "<<  temp_tf_accum.getTranslation()[0] << "," <<  temp_tf_accum.getTranslation()[1] << "," <<  temp_tf_accum.getTranslation()[2] << "\n");

		// ROS_INFO_STREAM("Rotation Matrix\n");
		// ROS_INFO_STREAM("[" << tf_arm_4.getRotation()(0,0) << " " << tf_arm_4.getRotation()(0,1) << " " << tf_arm_4.getRotation()(0,2) << "]\n");
		// ROS_INFO_STREAM("[" << tf_arm_4.getRotation()(1,0) << " " << tf_arm_4.getRotation()(1,1) << " " << tf_arm_4.getRotation()(1,2) << "]\n");
		// ROS_INFO_STREAM("[" << tf_arm_4.getRotation()(2,0) << " " << tf_arm_4.getRotation()(2,1) << " " << tf_arm_4.getRotation()(2,2) << "]\n");
		// ROS_INFO_STREAM("Translation: "<<  tf_arm_4.getTranslation()[0] << "," <<  tf_arm_4.getTranslation()[1] << "," <<  tf_arm_4.getTranslation()[2] << "\n");

	  	CollisionObject co1(obs_5_1, tf_obs_5_1);
	  	CollisionObject co2(obs_5_2, tf_obs_5_2);
	  	CollisionObject co3(obs_5_3, tf_obs_5_3);
	  	CollisionObject co4(obs_5_4, tf_obs_5_4);

	  	// CollisionObject robot_arm_1(arm_1, tf_arm_1);
	  	CollisionObject robot_arm_2(arm_2, tf_arm_2);
	  	CollisionObject robot_arm_3(arm_3, tf_arm_3);
	  	CollisionObject robot_arm_4(arm_4, tf_arm_4);

		// fcl::collide(&robot_arm_1, &co1, request, result);
		// result.getContacts(contacts);
		// total_contacts += contacts.size();

		// fcl::collide(&robot_arm_1, &co2, request, result);
		// result.getContacts(contacts);
		// total_contacts += contacts.size();

		// fcl::collide(&robot_arm_1, &co3, request, result);
		// result.getContacts(contacts);
		// total_contacts += contacts.size();

		// fcl::collide(&robot_arm_1, &co4, request, result);
		// result.getContacts(contacts);
		// total_contacts += contacts.size();

		fcl::collide(&robot_arm_2, &co1, request, result);
		result.getContacts(contacts);
		total_contacts += contacts.size();

		fcl::collide(&robot_arm_2, &co2, request, result);
		result.getContacts(contacts);
		total_contacts += contacts.size();

		fcl::collide(&robot_arm_2, &co3, request, result);
		result.getContacts(contacts);
		total_contacts += contacts.size();

		fcl::collide(&robot_arm_2, &co4, request, result);
		result.getContacts(contacts);
		total_contacts += contacts.size();

		fcl::collide(&robot_arm_3, &co1, request, result);
		result.getContacts(contacts);
		total_contacts += contacts.size();

		fcl::collide(&robot_arm_3, &co2, request, result);
		result.getContacts(contacts);
		total_contacts += contacts.size();

		fcl::collide(&robot_arm_3, &co3, request, result);
		result.getContacts(contacts);
		total_contacts += contacts.size();

		fcl::collide(&robot_arm_3, &co4, request, result);
		result.getContacts(contacts);
		total_contacts += contacts.size();

		fcl::collide(&robot_arm_4, &co1, request, result);
		result.getContacts(contacts);
		total_contacts += contacts.size();

		fcl::collide(&robot_arm_4, &co2, request, result);
		result.getContacts(contacts);
		total_contacts += contacts.size();

		fcl::collide(&robot_arm_4, &co3, request, result);
		result.getContacts(contacts);
		total_contacts += contacts.size();

		fcl::collide(&robot_arm_4, &co4, request, result);
		result.getContacts(contacts);
		total_contacts += contacts.size();
		#endif	

		// cout << contacts.size() << " contacts found" << endl;
		if (total_contacts != 0)
		{
			ROS_INFO_STREAM("COLLISION DETECTED! # of Contacts: " << total_contacts	 << " \n");
			// if (total_contacts < 6)
			// {
			// 	return true;
			// }
			return false;
		}
	}

  return true;
}

Node* extend(Vec3f* target) {
    //  if we weren't given a source point, try to find a close node
    Node* source = nearest(target);

    if (source == NULL)
    {
    	printf("nearest is NULL \n");
    	return NULL;
    }

    //  Get a state that's in the direction of @target from @source.
    //  This should take a step in that direction, but not go all the
    //  way unless the they're really close together.
    Vec3f* intermediateState;
    intermediateState = intermediateStateFunction(source->state(), target, RRT_STEP_SIZE);

    //  Make sure there's actually a direct path from @source to
    //  @intermediateState.  If not, abort
    if (!transitionValid(source->state(), intermediateState)) {
        return NULL;
    }

    // Add a node to the tree for this state
    Node* n = new Node(intermediateState, source);
    // _nodes.push_back(n);
    _nodes[node_count] = n;
    node_count++;
    return n;
}

// create a brics actuator message with the given joint position values
brics_actuator::JointPositions createArmPositionCommand(std::vector<double>& newPositions) {
	int numberOfJoints = 5;
	brics_actuator::JointPositions msg;

	if (newPositions.size() < numberOfJoints)
		return msg; // return empty message if not enough values provided

	for (int i = 0; i < numberOfJoints; i++) {
		// Set all values for one joint, i.e. time, name, value and unit
		brics_actuator::JointValue joint;
		joint.timeStamp = ros::Time::now();
		joint.value = newPositions[i];
		joint.unit = boost::units::to_string(boost::units::si::radian);

		// create joint names: "arm_joint_1" to "arm_joint_5" (for 5 DoF)
		std::stringstream jointName;
		jointName << "arm_joint_" << (i + 1);
		joint.joint_uri = jointName.str();

		// add joint to message
		msg.positions.push_back(joint);
	}

	return msg;
}

trajectory_msgs::JointTrajectory createArmTrajectoryCommand(std::vector<double>& newPositions) {
	int numberOfJoints = 5;
	trajectory_msgs::JointTrajectory msg;
	std_msgs::Header header;

	// ROS_INFO_STREAM("newPositions length: " << newPositions.size() << "\n");

	if (newPositions.size() < numberOfJoints)
		return msg; // return empty message if not enough values provided
	
	trajectory_msgs::JointTrajectoryPoint joint;
	for (int i = 0; i < numberOfJoints; i++) {
		// Set all values for one joint, i.e. time, name, value and unit
		joint.positions.push_back(newPositions[i]);

		// // create joint names: "arm_joint_1" to "arm_joint_5" (for 5 DoF)
		std::stringstream jointName;
		jointName << "arm_joint_" << (i + 1);

		// // add joint to message
		msg.joint_names.push_back(jointName.str());
	}

	joint.time_from_start = ros::Duration(0.0,1.0);

	msg.header = header;
	msg.points.push_back(joint);

	return msg;
}

/**
 * Callback function executes when new topic data comes.
 * Task of the callback function is to print data to screen.
 */
void chatterCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  // ROS_INFO("Seq: [%d]", msg->header.seq);
  // ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", msg->pose.pose.position.x,msg->pose.pose.position.y, msg->pose.pose.position.z);
  // ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  // ROS_INFO("Vel-> Linear: [%f], Angular: [%f]", msg->twist.twist.linear.x,msg->twist.twist.angular.z);
	local_x = msg->pose.pose.position.x;
	local_y = msg->pose.pose.position.y;
	local_theta = atan2(2*((msg->pose.pose.orientation.w)*(msg->pose.pose.orientation.z)+(msg->pose.pose.orientation.x)*(msg->pose.pose.orientation.y)),1-2*((msg->pose.pose.orientation.y)*(msg->pose.pose.orientation.y)+(msg->pose.pose.orientation.z)*(msg->pose.pose.orientation.z)));
	// ROS_INFO("local_theta:%f",local_theta);
}

int main(int argc, char **argv)
{
  	ros::init(argc, argv, "homework3_node");

  	ros::NodeHandle n;

 	ros::Publisher pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1000);
 	ros::Publisher armPublisher = n.advertise<brics_actuator::JointPositions>("arm_1/arm_controller/position_command", 1);
 	ros::Publisher pub_arm = n.advertise<trajectory_msgs::JointTrajectory >("arm_1/arm_controller/command", 1);
  	ros::Subscriber sub = n.subscribe("odom", 1000, chatterCallback);
  	ros::Rate loop_rate(120);

  	sleep(1);

	// srand (time(NULL));
  	ROS_INFO_STREAM("Homework3 started \n");

  	// tf_base_combined.setTranslation(Vec3f(2.5,2.5,0.5));
  	// tf_arm_1.setTranslation(Vec3f(5.0,1.0,0.375));
  	// tf_arm_2.setTranslation(Vec3f(3.0,5.0,0.375));
  	// tf_arm_3.setTranslation(Vec3f(0,3.0,0.25));
  	// tf_arm_4.setTranslation(Vec3f(2.5,-1.25,0.25)); 
  	// while(1)
  	// 	ROS_INFO_STREAM("random angles" << random_float_angle());

  	drive_source = new Vec3f(0.0,0.0,0.0);
  	drive_dest = new Vec3f(5.0,5.0,0.0);
  	// arm_source = new Vec3f(0.0-(PI/2),0.0,0.0);
  	// arm_dest = new Vec3f(0.628-(PI/2),1.964,-1.047);
  	arm_source = new Vec3f(0.0,0.0,0.0);
  	arm_dest = new Vec3f(0.628,1.964,-1.047);

  	// ROS_INFO_STREAM("drive_dest" << drive_dest << "(" << (*drive_dest)[0] <<","<< (*drive_dest)[1] << ")"<<" \n");
  	// ROS_INFO_STREAM("drive_source" << drive_source <<" \n");
  	// ROS_INFO_STREAM("Homework3 drive_dest" << drive_dest <<" \n");
  	// ROS_INFO_STREAM("Homework3 drive_dest" << drive_dest <<" \n");

	// Tree<Vec3f> myRRT = new Tree<Vec3f>(2, NUM_DRIVE_RRT, (float) GOAL_MAX_DIST, drive_source, drive_dest);
	// myRRT.run();

	ROS_INFO_STREAM("Get's Here \n");

    bool pathfound = false;
    Node* new_dest = NULL;

    #ifndef SELECT_ARM
    Node* root = new Node(drive_source, NULL);
        // _nodes.push_back(root);
    _nodes[node_count] = root;
    node_count++;

	for (int i = 0; i < NUM_DRIVE_RRT; i++)
	{
		Node* newNode = NULL;
		while(newNode == NULL){
			newNode = extend(new Vec3f(random_float_location(), random_float_location(),0));
		}

    	if (simple_dist_drive(newNode->state(),drive_dest) < GOAL_MAX_DIST)
    	{
        	pathfound = true;
        	new_dest = newNode;
        	break;
    	}

    // ROS_INFO_STREAM("Node # " << i << " \n");
	}
	#else
	Node* root = new Node(arm_source, NULL);
        // _nodes.push_back(root);
    _nodes[node_count] = root;
    node_count++;

	for (int i = 0; i < NUM_ARM_RRT; i++)
	{
		Node* newNode = NULL;
		while(newNode == NULL){
			newNode = extend(new Vec3f(random_float_angle(), random_float_angle(),random_float_angle()));
		}

		ROS_INFO_STREAM("Node # " << i << " Dist from Goal: " << simple_dist_arm(newNode->state(),arm_dest) <<" \n");

    	if (simple_dist_arm(newNode->state(),arm_dest) < GOAL_MAX_DIST)
    	{
        	pathfound = true;
        	new_dest = newNode;
        	break;
    	}

    	// ROS_INFO_STREAM("Node # " << i << " \n");
	}

	#endif

	for (int i = 0; i < node_count; i++)
  	{
  	    ROS_INFO_STREAM("RRT Node # " << i << "Point" << (*(_nodes[i]->state()))[0] << ","<< (*(_nodes[i]->state()))[1]<< " \n");
  	}

 	ROS_INFO_STREAM("Pathfound " << pathfound << " \n");

	// return 0;
	Node* curr = _nodes[0];
  	Node* next = new_dest;
  	int next_point_flag = 0;
  	// Vec3f* direction = new Vec3f();

  	int temp_shit = 0;

  	while(ros::ok())
  	{
  		ros::spinOnce();

  		geometry_msgs::Twist vel;

	  	float xvel = 0.0;
  		float yvel = 0.0;
  		float omega = 0.0;

  		float joint_1 = JOINT_1_OFFSET;
  		float joint_2 = JOINT_2_OFFSET;
  		float joint_3 = JOINT_3_OFFSET;
  		float joint_4 = JOINT_4_OFFSET;
  		float joint_5 = JOINT_5_OFFSET;

		#ifndef SELECT_ARM
		if (next_point_flag == 0)
	  		{		
		  		next = new_dest;
		  		while(next->parent() != curr)
		  			next = next->parent();
		  		// *direction = (*(next->state()))-(*(curr->state()));
		  		next_point_flag = 1;
	  		}
	  		else
	  		{
	  			if (simple_dist_orig(local_x,local_y,(*(next->state()))[0],(*(next->state()))[1]) > 0.01)
	  			{
	  				xvel = ((*(next->state()))[0]-local_x)*10;
	  				yvel = ((*(next->state()))[1]-local_y)*10;
	  			}
	  			else
	  			{
	  				ROS_INFO_STREAM("ON TO THE NEXT POINT \n");
	  				xvel = 0;
	  				yvel = 0;
	  				curr = next;
	  				// if (curr == new_dest)
	  				// {
	  				// 	return 0;
	  				// }
	  				next_point_flag = 0;
	  			}
	  			
	  		}

	  	ROS_INFO_STREAM("velocity " << xvel << " , " << yvel << " \n");
	  	ROS_INFO_STREAM("dist to next point : " << simple_dist_orig(local_x,local_y,(*(next->state()))[0],(*(next->state()))[1]) << " \n");

  		vel.linear.x = xvel;
 		vel.linear.y = yvel;
 		// // vel.angularx = xvel;
 		// // vel.linear.y = yvel;
 		// vel.angular.z = omega;

 		pub.publish(vel);
	  	#else
		// brics_actuator::JointPositions msg;
		// std::vector<double> jointvalues(5);

		// // move arm straight up. values were determined empirically
		// ROS_INFO_STREAM("UP \n");
		// jointvalues[0] = 2.95;
		// jointvalues[1] = 1.05;
		// jointvalues[2] = -2.44;
		// jointvalues[3] = 1.73;
		// jointvalues[4] = 2.95;
		// msg = createArmPositionCommand(jointvalues);
		// armPublisher.publish(msg);

		// ros::Duration(5).sleep();

		// // move arm back close to calibration position
		// ROS_INFO_STREAM("DOWN \n");
		// jointvalues[0] = 0.11;
		// jointvalues[1] = 0.11;
		// jointvalues[2] = -0.11;
		// jointvalues[3] = 0.11;
		// jointvalues[4] = 0.111;
		// msg = createArmPositionCommand(jointvalues);
		// armPublisher.publish(msg);

		// ros::Duration(2).sleep();
  		next = new_dest;
  		while(next->parent() != curr)
  			next = next->parent();

		trajectory_msgs::JointTrajectory msg;
		std::vector<double> jointvalues(5);

		// jointvalues[0] = 2.95;
		// jointvalues[1] = 1.05;
		// jointvalues[2] = -2.44;
		// jointvalues[3] = 1.73;
		// jointvalues[4] = 2.95;

		// jointvalues[0] = 0.0;
		// jointvalues[1] = 0.0;
		// jointvalues[2] = 0.0;
		// jointvalues[3] = 0.0;
		// jointvalues[4] = 0.0;

		jointvalues[0] = JOINT_1_OFFSET;
		jointvalues[1] = (*(next->state()))[0]+JOINT_2_OFFSET;
		jointvalues[2] = (*(next->state()))[1]+JOINT_3_OFFSET;
		jointvalues[3] = (*(next->state()))[2]+JOINT_4_OFFSET;
		jointvalues[4] = JOINT_5_OFFSET;

		// jointvalues[0] = JOINT_1_OFFSET;
		// jointvalues[1] = JOINT_2_OFFSET-(PI/2);
		// jointvalues[2] = JOINT_3_OFFSET;
		// jointvalues[3] = JOINT_4_OFFSET;
		// jointvalues[4] = JOINT_5_OFFSET;

		msg = createArmTrajectoryCommand(jointvalues);
		pub_arm.publish(msg);

		curr = next;
		ROS_INFO_STREAM("ON TO THE NEXT POINT \n");
		ROS_INFO_STREAM("Last Point:[" << jointvalues[0] << "," << jointvalues[1] << "," << jointvalues[2] << "," << jointvalues[3] << "," << jointvalues[4] << "] \n");

		ros::Duration(1).sleep();
		// ros::Duration(0,100000000).sleep();

	  	#endif


 		loop_rate.sleep();

 		temp_shit++;
 		ROS_INFO_STREAM("LOOP COUNT: " << temp_shit << " \n");
 		// if (temp_shit > 10)
 		// {
 		// 	return 0;
 		// }

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
