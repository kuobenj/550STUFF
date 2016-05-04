class Tree
{
public:
    Tree(int dimension, int maxIterations, float goalMaxDist, Vec3f* source, Vec3f* dest) {
    	_dimension = dimension;
    	_maxIterations = maxIterations;
    	_goalMaxDist = goalMaxDist;

    	_goalState = dest;

    	Node* root = new Node(source, NULL);
        _nodes.push_back(root);

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

  	  	obs_1 = new Box(2.0,2.0,1.0);
	  	obs_2 = new Box(0.75,0.75,0.75);
	  	obs_3 = new Box(0.75,0.75,0.75);
	  	obs_4 = new Box(0.5,0.5,0.5);
	  	wall_1 = new Box(8.0,0.5,0.5);
	  	wall_2 = new Box(0.5,8.0,0.5);
	  	wall_3 = new Box(8.0,0.5,0.5);
	  	wall_4 = new Box(0.5,0.5,8.0);
	  	obs_5_1 = new Box(0.4,0.4,0.2);
	  	obs_5_2 = new Box(0.05,0.05,0.15);
	  	obs_5_3 = new Box(0.05,0.05,0.15);
	  	obs_5_4 = new Box(0.25,0.25,0.15);

	  	base_combined = new Box(0.5701,0.3570,0.1600);
		arm_1 = new Box(0.1697,0.1699,0.1060);
		arm_2 = new Box(0.22,0.0745,0.0823);
		arm_3 = new Box(0.1920,0.0591,0.0750);
		arm_4 = new Box(0.2401,0.0581,0.0987);

    }

    /**
     * Executes the RRT algorithm with the given start state.
     *
     * @return a bool indicating whether or not it found a path to the goal
     */
    bool run() {
        //  grow the tree until we find the goal or run out of iterations
        for (int i = 0; i < _maxIterations; i++) {
        	if (_dimension == 2)
        	{
        		Node* newNode;
        		while(newNode != NULL)
        			newNode = extend(new fcl::Vec3f(random_float_location(), random_float_location(),0));

            	if (newNode &&
                	simple_dist_drive((*(newNode->state()))[0],(*(newNode->state()))[1], (*_goalState)[0],(*_goalState)[1]) <
                    _goalMaxDist)
                return true;
        	}
        	else
        	{
        		// Node<T>* newNode = extend(/****RANDOM STATE****/);

          //   	if (newNode &&
          //       	_stateSpace->distance(newNode->state(), _goalState) <
          //           _goalMaxDist)
          //       return true;
        	}

        }

        //  we hit our iteration limit and didn't reach the goal :(
        return false;
    }

    /**
     * Find the node int the tree closest to @state.  Pass in a floay available with -std=c++11 or -std=gnu++11 [enabled by default]
t pointer
     * as the second argument to get the distance that the node is away from
     * @state.
     */
    Node* nearest(Vec3f* state) {
        float bestDistance = -1;
        Node* best = NULL;
        if (_dimension == 2)
        {
	       	for (Node* other : _nodes) {
	            float dist = simple_dist_drive((*(other->state()))[0],(*(other->state()))[1],(*state)[0],(*state)[1]); //_stateSpace->distance(other->state(), state);
	            if (bestDistance < 0 || dist < bestDistance) {
	                bestDistance = dist;
	                best = other;
	            }
	        }
        }

        return best;
    }

    /**
     * Grow the tree in the direction of @state
     *
     * @return the new tree Node (may be NULL if we hit Obstacles)
     * @param target The point to extend the tree to
     * @param source The Node to connect from.  If source == NULL, then
     *             the closest tree point is used
     */
    Node* extend(Vec3f* target) {
        //  if we weren't given a source point, try to find a close node
        Node* source = nearest(target);

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
        _nodes.push_back(n);
        return n;
    }

    float random_float_location()
    {
    	float r = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
    	return r*5.0;
    }

    float random_float_angle()
    {
    	return 0.0;
    }

    float simple_dist_drive(float x1, float y1, float x2, float y2)
	{
		return sqrt(((x1-x2)*(x1-x2))+((y1-y2)*(y1-y2)));
	}

	Vec3f* intermediateStateFunction(Vec3f* source, Vec3f* state, float stepSize)
	{
		Vec3f intermediateState = new Vec3f();
		*intermediateState = (*state)-(*source);
		// Node* intermediateState = new Node(&temp)
		intermediateState->normalize();
		*intermediateState *= stepSize;

		*intermediateState += source;

		return intermediateState;
	}

	bool transitionValid(Vec3f* source, Vec3f* state)
	{	
	  static const int num_max_contacts = std::numeric_limits<int>::max();
	  static const bool enable_contact = true;
	  fcl::CollisionResult result;
	  fcl::CollisionRequest request(num_max_contacts,
	                                enable_contact);

	  for (int j = 0; j < 9; j++)
	  {
		  tf_base_combined.setTranslation((state/10.0*j)+(source/10.0*(10-j))+Vec3f(BASE_X_OFFSET,BASE_Y_OFFSET,BASE_Z_OFFSET));

		  for (int i = 0; i < NUM_OBS; i++)
		  {
		  	CollisionObject co0((obst_box_array[i]), *(obst_tf_array[i]));
		  	CollisionObject co1(base_combined, tf_base_combined);

	  		fcl::collide(&co0, &co1, request, result);
			vector<Contact> contacts;
			result.getContacts(contacts);

			// cout << contacts.size() << " contacts found" << endl;
			if (contacts.size() != 0)
			{
				return false;
			}
		  }
		}

	  return true;
	}

    // /**
    //  * Get the path from the receiver's root point to the dest point
    //  *
    //  * @param callback The lambda to call for each state in the path
    //  * @param reverse if true, the states will be sent from @dest to the
    //  *                tree's root
    //  */
    // void getPath(std::function<void(const T& stateI)> callback, Node<T>* dest,
    //              const bool reverse = false) {
    //     const Node<T>* node = dest;
    //     if (reverse) {
    //         while (node) {
    //             callback(node->state());
    //             node = node->parent();
    //         }
    //     } else {
    //         //  order them correctly in a list
    //         std::list<const Node<T>*> nodes;
    //         while (node) {
    //             nodes.push_front(node);
    //             node = node->parent();
    //         }

    //         //  then pass them one-by-one to the callback
    //         for (const Node<T>* n : nodes) callback(n->state());
    //     }
    // }

    // /**
    //  * Get the path from the receiver's root point to the dest point.
    //  *
    //  * @param vectorOut The vector to append the states along the path
    //  * @param reverse if true, the states will be sent from @dest to the
    //  *                tree's root
    //  */
    // void getPath(std::vector<T>& vectorOut, Node<T>* dest,
    //              const bool reverse = false) {
    //     getPath([&](const T& stateI) { vectorOut.push_back(stateI); }, dest,
    //             reverse);
    // }

protected:
    /**
     * A list of all Node objects in the tree.
     */
    std::vector<Node*> _nodes;

    Vec3f* _goalState;

    int _maxIterations;

    float _goalMaxDist;

    float _stepSize;
    float _maxStepSize;

    int _dimension;

    //https://github.com/flexible-collision-library/fcl/blob/master/test/general_test.cpp
  	//hardcoded obstacles

  	// std::shared_ptr<Box> obs_1(new Box(2.0,2.0,1.0));
  	// std::shared_ptr<Box> obs_2(new Box(0.75,0.75,0.75));
  	// std::shared_ptr<Box> obs_3(new Box(0.75,0.75,0.75));
  	// std::shared_ptr<Box> obs_4(new Box(0.5,0.5,0.5));
  	// std::shared_ptr<Box> wall_1(new Box(8.0,0.5,0.5));
  	// std::shared_ptr<Box> wall_2(new Box(0.5,8.0,0.5));
  	// std::shared_ptr<Box> wall_3(new Box(8.0,0.5,0.5));
  	// std::shared_ptr<Box> wall_4(new Box(0.5,0.5,8.0));
  	// std::shared_ptr<Box> obs_5_1(new Box(0.4,0.4,0.2));
  	// std::shared_ptr<Box> obs_5_2(new Box(0.05,0.05,0.15));
  	// std::shared_ptr<Box> obs_5_3(new Box(0.05,0.05,0.15));
  	// std::shared_ptr<Box> obs_5_4(new Box(0.25,0.25,0.15));

  	Box* obs_1;
  	Box* obs_2;
  	Box* obs_3;
  	Box* obs_4;
  	Box* wall_1;
  	Box* wall_2;
  	Box* wall_3;
  	Box* wall_4;
  	Box* obs_5_1;
  	Box* obs_5_2;
  	Box* obs_5_3;
  	Box* obs_5_4;

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

	// (std::shared_ptr<Box>)* obst_box_array[12] = {&obs_1, &obs_2, &obs_3, &obs_4, 
	// 										&wall_1, &wall_2, &wall_3, &wall_4, 
	// 										&obs_5_1, &obs_5_2, &obs_5_3, &obs_5_4};
	Box* obst_box_array[12] = {obs_1, obs_2, obs_3, obs_4, 
											wall_1, wall_2, wall_3, wall_4, 
											obs_5_1, obs_5_2, obs_5_3, obs_5_4};
	
	Transform3f* obst_tf_array[12] = {&tf_obs_1, &tf_obs_2, &tf_obs_3, &tf_obs_4, 
											&tf_wall_1, &tf_wall_2, &tf_wall_3, &tf_wall_4, 
											&tf_obs_5_1, &tf_obs_5_2, &tf_obs_5_3, &tf_obs_5_4};

  	//Robot bounding boxes

	// std::shared_ptr<Box> base_combined(new Box(0.5701,0.3570,0.1600));
 //  	std::shared_ptr<Box> arm_1(new Box(0.1697,0.1699,0.1060));
 //  	std::shared_ptr<Box> arm_2(new Box(0.22,0.0745,0.0823));
 //  	std::shared_ptr<Box> arm_3(new Box(0.1920,0.0591,0.0750));
 //  	std::shared_ptr<Box> arm_4(new Box(0.2401,0.0581,0.0987));

	Box* base_combined;
  	Box* arm_1;
  	Box* arm_2;
  	Box* arm_3;
  	Box* arm_4;

  	Transform3f tf_base_combined;
  	Transform3f tf_arm_1;
  	Transform3f tf_arm_2;
  	Transform3f tf_arm_3;
  	Transform3f tf_arm_4;
};


/**
 * This tutorial demonstrates simple receipt of position and speed of the Evarobot over the ROS system.
 */