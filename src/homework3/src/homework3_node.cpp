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

#define PI 3.14159

#define RRT_STEP_SIZE 0.1
#define NUM_OBS 12
#define NUM_DRIVE_RRT 10000
#define NUM_ARM_RRT 1000
#define GOAL_MAX_DIST 0.1
// #define SELECT_ARM

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
  	boost::shared_ptr<Box> obs_5_2(new Box(0.05,0.05,0.15));
  	boost::shared_ptr<Box> obs_5_3(new Box(0.05,0.05,0.15));
  	boost::shared_ptr<Box> obs_5_4(new Box(0.25,0.25,0.15));

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

  	for (int j = 0; j < 9; j++)
  	{
  		int total_contacts = 0;
	  	vector<Contact> contacts;

	  	#ifndef SELECT_ARM
	 	tf_base_combined.setTranslation((*state/10.0*j)+(*source/10.0*(10-j))+Vec3f(BASE_X_OFFSET,BASE_Y_OFFSET,BASE_Z_OFFSET));

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
		Transform3f temp_tf;
	 	tf_base_combined.setTranslation(Vec3f(BASE_X_OFFSET,BASE_Y_OFFSET,BASE_Z_OFFSET));

	  	CollisionObject co1(obs_5_1, tf_obs_5_1);
	  	CollisionObject co2(obs_5_2, tf_obs_5_2);
	  	CollisionObject co3(obs_5_3, tf_obs_5_3);
	  	CollisionObject co4(obs_5_4, tf_obs_5_4);

	  	tf_arm_1 = tf_arm_1*temp_tf;

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
			// ROS_INFO_STREAM("COLLISION DETECTED! \n");
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
  	ros::Subscriber sub = n.subscribe("odom", 1000, chatterCallback);
  	ros::Rate loop_rate(120);

  	ROS_INFO_STREAM("Homework3 started \n");

  	// tf_base_combined.setTranslation(Vec3f(2.5,2.5,0.5));
  	// tf_arm_1.setTranslation(Vec3f(5.0,1.0,0.375));
  	// tf_arm_2.setTranslation(Vec3f(3.0,5.0,0.375));
  	// tf_arm_3.setTranslation(Vec3f(0,3.0,0.25));
  	// tf_arm_4.setTranslation(Vec3f(2.5,-1.25,0.25)); 

  	drive_source = new Vec3f(0.0,0.0,0.0);
  	drive_dest = new Vec3f(5.0,5.0,0.0);
  	arm_source = new Vec3f(0.0,0.0,0.0);
  	arm_dest = new Vec3f(0.628,1.964,-1.047);

  	ROS_INFO_STREAM("drive_dest" << drive_dest << "(" << (*drive_dest)[0] <<","<< (*drive_dest)[1] << ")"<<" \n");
  	ROS_INFO_STREAM("drive_source" << drive_source <<" \n");
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

	for (int i = 0; i < NUM_DRIVE_RRT; i++)
	{
		Node* newNode = NULL;
		while(newNode == NULL){
			newNode = extend(new Vec3f(random_float_angle(), random_float_angle(),random_float_angle()));
		}

    	if (simple_dist_drive(newNode->state(),drive_dest) < GOAL_MAX_DIST)
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
	  	#else

	  	#endif


	  	ROS_INFO_STREAM("velocity " << xvel << " , " << yvel << " \n");
	  	ROS_INFO_STREAM("dist to next point : " << simple_dist_orig(local_x,local_y,(*(next->state()))[0],(*(next->state()))[1]) << " \n");

  		vel.linear.x = xvel;
 		vel.linear.y = yvel;
 		// // vel.angularx = xvel;
 		// // vel.linear.y = yvel;
 		// vel.angular.z = omega;

 		pub.publish(vel);

 		loop_rate.sleep();

 		// temp_shit++;
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
