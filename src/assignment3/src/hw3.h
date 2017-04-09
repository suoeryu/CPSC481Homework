#include <vector>
#include <queue>
#include <string>
#include <sstream>
#include <string>

#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "geometry_msgs/Twist.h"

#pragma once

const double PI = 3.14159265359;

class Rectangle;

class Point {
    friend class Rectangle;
    friend std::ostream& operator<<(std::ostream&, const Point&);
    friend std::ostream& operator<<(std::ostream&, const Rectangle&);
private:
    double x;
    double y;
public:
    Point(double x, double y): x(x), y(y) {};
    double euclidean_distance(const Point &p) const;
    double euclidean_distance(const double x, const double y) const;
    double radian(const Point & p) const;
    double get_x()const {
        return x;
    }
    double get_y()const {
        return y;
    }
    bool operator==(const Point& rhs) const {
        return x == rhs.x && y== rhs.y;
    }
    bool operator!=(const Point& rhs) const {
        return !(*this == rhs);
    }
};
std::ostream& operator<<(std::ostream& os, const Point& p) {
    os << "(" << p.x << ", " << p.y << ")";
}

class Rectangle {
    friend std::ostream& operator<<(std::ostream&, const Rectangle&);
private:
    Point center;
    Point bottom_left;
    Point top_right;

    std::vector<Point> inner_points;
    std::vector<Point> outer_points;

    // Used for Cohen–Sutherland algorithm to solve Line clipping problem
    typedef int Outcode;
    static const int INSIDE = 0; // 0000
    static const int LEFT = 1;   // 0001
    static const int RIGHT = 2;  // 0010
    static const int BELOW = 4; // 0100
    static const int ABOVE = 8;    // 1000

    Outcode compute_outcode(const Point &p) const;
public:
    Rectangle(double center_x, double center_y, double range_factor);
    bool is_point_in(const Point &p) const ;
    // Use Cohen–Sutherland algorithm
    bool is_linesegment_in(const Point &p1, const Point &p2) const;

    const std::vector<Point> & get_inner_points() const {
        return inner_points;
    }
    const std::vector<Point> & get_outer_points() const {
        return outer_points;
    }
};
std::ostream& operator<<(std::ostream& os, const Rectangle & obj) {
    os << "[(" << obj.bottom_left.x << ", " << obj.bottom_left.y << "), "
       << "(" << obj.bottom_left.x << ", " << obj.top_right.y << "), "
       << "(" << obj.top_right.x << ", " << obj.bottom_left.y << "), "
       << "(" << obj.top_right.x << ", " << obj.top_right.y << ")]";
}

class Turtle {
    friend std::ostream& operator<<(std::ostream&, const Turtle&);
private:
    Point pos;
    Rectangle scope;
    std::string name;
    bool alive;
public:
    Turtle(double x, double y, double range_factor, std::string name)
        : pos(x, y), scope(x, y, range_factor), name(name), alive(true) {};
    const std::vector<Point> & inner_positions() const {
        return scope.get_inner_points();
    }
    const std::vector<Point> & outer_positions() const {
        return scope.get_outer_points();
    }
    const Point& get_position() const {
        return pos;
    }
    const std::string& get_name() const {
        return name;
    }
    bool is_alive() const {
        return alive;
    }
    bool impact(const Point &p1, const Point &p2) const {
        return scope.is_linesegment_in(p1, p2);
    }
    bool impact(const Point &p) const {
        return scope.is_point_in(p);
    }
    void kill(){
        alive = false;
    }
};
std::ostream& operator<<(std::ostream& os, const Turtle& turtle){
    os << turtle.name << " at " << turtle.pos << " Impact Scope: " << turtle.scope;
}

class State {
    Point pos;
    const double path_length;
    const State * pre_state_ptr;

    std::vector<const Turtle*> targets;
    const std::vector<Turtle>& villains;

    bool is_acceptable(const Point& pos) const;
public:
    State(const std::vector<Turtle> & targets, const std::vector<Turtle> & villains);
    State(const State * pre_ptr, const Point & pos);
    const Point & get_position() const {
        return pos;
    }
    double get_path_length() const {
        return path_length;
    }
    double estimate_remain_distance() const;
    std::vector<Point> get_possible_next_postion() const;
    bool is_init() const {
        return pre_state_ptr == NULL;
    }
    bool is_final() const {
        return targets.empty();
    }
    const State * get_pre_state()const {
        return pre_state_ptr;
    }
};

class Heuristic {
private:
    // evaluation function g(n), return the cost to the state
    static double g(const State * state_ptr);

    // evaluation function h(n), return heuristic estimate of cost to goal
    static double h(const State * state_ptr);

    struct CompareTurtleState {
        // compare function used in priority queue
        // evaluation function f(n) = g(n) + h(n)
        // the state which has smaller value of f(n) has higher priority
        bool operator()(const State * ptr1, const State * ptr2) {
            return g(ptr1) + h(ptr1) > g(ptr2) + h(ptr2);
        }
    };

    // priority queue (heap) storing the new states
    std::priority_queue<const State*, std::vector<State*>, CompareTurtleState> open_state_ptr_list;

    // list storing visited states.
    std::vector<const State*> closed_state_ptr_list;

    bool is_visited(const Point& from, const Point& to) const;
public:
    Heuristic(const std::vector<Turtle> & targets, const std::vector<Turtle> &villains);
    ~Heuristic();
    const std::vector<Point> calculate_path();
};

void poseCallback(const turtlesim::Pose::ConstPtr & pose_message);

class Environment {
    friend void poseCallback(const turtlesim::Pose::ConstPtr & pose_message);
private:
    // Store the current turtle pose
    static turtlesim::Pose turtlesim_pose;

    ros::Publisher velocity_publisher;
    ros::Subscriber pose_subscriber;
    ros::ServiceClient reset_client;
    ros::ServiceClient kill_client;
    ros::ServiceClient spawn_client;

    std::vector<Turtle> targets;
    std::vector<Turtle> villains;

    bool reset();
    bool kill_turtle(std::string name);
    bool spawn_turtle(const Turtle &turtle, double theta);
    bool spawn_turtle(double x, double y, double theta, std::string name);

    void set_speed(const Point &dest, geometry_msgs::Twist &vel_msg);
    void check_capture();
public:
    Environment(int argc, char *argv[]);
    const std::vector<Turtle> & get_targets() {
        return targets;
    }
    const std::vector<Turtle> & get_villains() {
        return villains;
    }
    void random_init_turtles(unsigned seed, int targets_num, int villains_num);
    double move_turtle1(const Point &pos);
};
