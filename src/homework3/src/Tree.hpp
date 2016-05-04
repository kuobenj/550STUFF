//Tree.h
#ifndef TREE_H_
#define TREE_H_

#include <list>
#include <vector>
#include <memory>
#include <functional>
#include <stdexcept>
#include <stdlib.h>
#include <iostream>

template <typename T>
class Node
{

public:
    Node(const T& state, Node<T>* parent = nullptr) {
        _parent = parent;
        _state = state;

        if (_parent) {
            _parent->_children.push_back(this);
        }
    }

    const Node<T>* parent() const { return _parent; }

    /**
     * Gets the number of ancestors (parent, parent's parent, etc) that
     * the node has.
     * Returns 0 if it doesn't have a parent.
     */
    int depth() const {
        int n = 0;
        for (Node<T>* ancestor = _parent; ancestor != nullptr;
             ancestor = ancestor->_parent) {
            n++;
        }
        return n;
    }

    /**
     * The @state property is the point in the state-space that this
     * Node represents.  Generally this is a vector (could be 2d, 3d, etc)
     */
    const T& state() const { return _state; }

private:
    T _state;
    std::list<Node<T>*> _children;
    Node<T>* _parent;

};

template <typename T>
class Tree
{
public:
    Tree() {

        //  default values
        setStepSize(0.1);
        setMaxIterations(1000);
        setGoalMaxDist(0.1);
    }

    /**
     * The maximum number of random states in the state-space that we will
     * try before giving up on finding a path to the goal.
     */
    int maxIterations() const { return _maxIterations; }
    void setMaxIterations(int itr) { _maxIterations = itr; }

    float stepSize() const { return _stepSize; }
    void setStepSize(float stepSize) { _stepSize = stepSize; }

    /**
     * @brief How close we have to get to the goal in order to consider it
     * reached.
     * @details The RRT will continue to run unti we're within @goalMaxDist of
     * the
     * goal state.
     */
    float goalMaxDist() const { return _goalMaxDist; }
    void setGoalMaxDist(float maxDist) { _goalMaxDist = maxDist; }

    /**
     * Executes the RRT algorithm with the given start state.
     *
     * @return a bool indicating whether or not it found a path to the goal
     */
    bool run() {
        //  grow the tree until we find the goal or run out of iterations
        for (int i = 0; i < _maxIterations; i++) {
            Node<T>* newNode = grow();

            if (newNode &&
                _stateSpace->distance(newNode->state(), _goalState) <
                    _goalMaxDist)
                return true;
        }

        //  we hit our iteration limit and didn't reach the goal :(
        return false;
    }

    /**
     * Picks a random state and attempts to extend the tree towards it.
     * This is called at each iteration of the run() method.
     */
    Node<T>* grow() {
        //  extend towards goal, waypoint, or random state depending on the
        //  biases
        //  and a random number
        float r =
            rand() /
            (float)RAND_MAX;  //  r is between 0 and one since we normalize it
        if (r < goalBias()) {
            return extend(goalState());
        } else if (r < goalBias() + waypointBias() && _waypoints.size() > 0) {
            const T& waypoint = _waypoints[rand() % _waypoints.size()];
            return extend(waypoint);
        } else {
            return extend(_stateSpace->randomState());
        }
    }

    /**
     * Find the node int the tree closest to @state.  Pass in a float pointer
     * as the second argument to get the distance that the node is away from
     * @state.
     */
    Node<T>* nearest(const T& state, float* distanceOut = nullptr) {
        float bestDistance = -1;
        Node<T>* best = nullptr;

        for (Node<T>* other : _nodes) {
            float dist = _stateSpace->distance(other->state(), state);
            if (bestDistance < 0 || dist < bestDistance) {
                bestDistance = dist;
                best = other;
            }
        }

        if (distanceOut) *distanceOut = bestDistance;

        return best;
    }

    /**
     * Grow the tree in the direction of @state
     *
     * @return the new tree Node (may be nullptr if we hit Obstacles)
     * @param target The point to extend the tree to
     * @param source The Node to connect from.  If source == nullptr, then
     *             the closest tree point is used
     */
    virtual Node<T>* extend(const T& target, Node<T>* source = nullptr) {
        //  if we weren't given a source point, try to find a close node
        if (!source) {
            source = nearest(target, nullptr);
            if (!source) {
                return nullptr;
            }
        }

        //  Get a state that's in the direction of @target from @source.
        //  This should take a step in that direction, but not go all the
        //  way unless the they're really close together.
        T intermediateState;
        if (_isASCEnabled) {
            intermediateState = _stateSpace->intermediateState(
                source->state(), target, stepSize(), maxStepSize());
        } else {
            intermediateState = _stateSpace->intermediateState(
                source->state(), target, stepSize());
        }

        //  Make sure there's actually a direct path from @source to
        //  @intermediateState.  If not, abort
        if (!_stateSpace->transitionValid(source->state(), intermediateState)) {
            return nullptr;
        }

        // Add a node to the tree for this state
        Node<T>* n = new Node<T>(intermediateState, source);
        _nodes.push_back(n);
        return n;
    }

    /**
     * Get the path from the receiver's root point to the dest point
     *
     * @param callback The lambda to call for each state in the path
     * @param reverse if true, the states will be sent from @dest to the
     *                tree's root
     */
    void getPath(std::function<void(const T& stateI)> callback, Node<T>* dest,
                 const bool reverse = false) {
        const Node<T>* node = dest;
        if (reverse) {
            while (node) {
                callback(node->state());
                node = node->parent();
            }
        } else {
            //  order them correctly in a list
            std::list<const Node<T>*> nodes;
            while (node) {
                nodes.push_front(node);
                node = node->parent();
            }

            //  then pass them one-by-one to the callback
            for (const Node<T>* n : nodes) callback(n->state());
        }
    }

    /**
     * Get the path from the receiver's root point to the dest point.
     *
     * @param vectorOut The vector to append the states along the path
     * @param reverse if true, the states will be sent from @dest to the
     *                tree's root
     */
    void getPath(std::vector<T>& vectorOut, Node<T>* dest,
                 const bool reverse = false) {
        getPath([&](const T& stateI) { vectorOut.push_back(stateI); }, dest,
                reverse);
    }

    /**
     * @return The root node or nullptr if none exists
     */
    Node<T>* rootNode() const {
        if (_nodes.empty()) return nullptr;

        return _nodes.front();
    }

    /**
     * @brief The start state for this tree
     */
    const T& startState() const {
        if (_nodes.empty())
            throw std::logic_error("No start state specified for RRT::Tree");
        else
            return rootNode()->state();
    }
    void setStartState(const T& startState) {
        reset(true);

        //  create root node from provided start state
        Node<T>* root = new Node<T>(startState, nullptr);
        _nodes.push_back(root);
    }

    /**
     * @brief The goal this tree is trying to reach.
     */
    const T& goalState() const { return _goalState; }
    void setGoalState(const T& goalState) { _goalState = goalState; }

protected:
    /**
     * A list of all Node objects in the tree.
     */
    std::vector<Node<T>*> _nodes;

    T _goalState;

    int _maxIterations;

    float _goalMaxDist;

    float _stepSize;
    float _maxStepSize;
};

#endif // MY_TREE