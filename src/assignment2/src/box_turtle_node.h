#include <vector>
#include <queue>

#include "ros/ros.h"
#include "turtlesim/Pose.h"

#pragma once

enum Action { TurnLeft, TurnRight, GoStraight };
enum Direction {East = 0, North, West, South};

const double y_unit = 2;
const double x_unit = 1.414 * y_unit;
const double PI = 3.14159265359;

ros::Publisher velocity_publisher;
ros::Subscriber pose_subscriber;
// Global variable store the current turtle pose
turtlesim::Pose turtlesim_pose;

void poseCallback(const turtlesim::Pose::ConstPtr & pose_message);
double move_forward(double speed, double distance);
double rotate(double speed, double relative_angle);

class Coordinate {
private:
    double x;
    double y;
public:
    // Instructor
    Coordinate(double x, double y): x(x), y(y) {}

    // add x, y to current coordinate
    void add(double x_value, double y_value);

    // get straight-line distance between another coordinate
    double getDistance(const Coordinate& other) const;
};

class TurtleState {
private:
    // relative coordinate to original position, x-axis direction is same as turtle's original forward direction
    Coordinate relative_coordinate;

    // relative goal coordinate to original position
    Coordinate &goal_coordinate;

    double rotative_theta; // rotated theta to previous position, radian

    // the current forward direction of Turtle, East if direction same as x-axis
    Direction direction;

    double path_length; // path length to original position

    TurtleState * const pre_state_ptr; // the parent state of current state, for backtrack
public:
    // Instructor, create a original state
    TurtleState(Coordinate& goal);

    // Instructor, create a child state base on the action
    TurtleState(TurtleState* previous_state, Action action);

    double getPathLength() const {
        return path_length;
    }

    TurtleState* getParentState() {
        return pre_state_ptr;
    }

    double getStraightDistanceToGoal() const;

    // Move Turtle to this state from previous state
    void move();
};

class Heuristic {
private:
    // evaluation function g(n), return the cost to the state
    static double g(const TurtleState* state_ptr);

    // evaluation function h(n), return heuristic estimate of cost to goal
    static double h(const TurtleState* state_ptr);

    struct CompareTurtleState {
        // compare function used in priority queue
        // evaluation function f(n) = g(n) + h(n)
        // the state which has smaller value of f(n) has higher priority
        bool operator()(const TurtleState * ptr1, const TurtleState * ptr2) {
            return g(ptr1) + h(ptr1) > g(ptr2) + h(ptr2);
        }
    };

    // The coordinate of goal state, (0,0) in this homework
    Coordinate goal_coordinate;

    // pointer to the original state
    TurtleState* orig_state_ptr;

    // list storing visited states.
    std::vector<TurtleState*> closed_state_ptr_list;

    // priority queue (heap) storing the new states
    std::priority_queue<TurtleState*, std::vector<TurtleState*>, CompareTurtleState> open_state_ptr_list;

    // store the calculated path
    std::vector<TurtleState*> path;

public:
    Heuristic();
    ~Heuristic();
    void calculate_path();
    // move turtle follow the states in path
    void move_turtle();
};
