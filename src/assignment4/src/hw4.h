#include <string>
#include <vector>

#pragma once

#include "ros/ros.h"
#include "ros/topic.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"

const double PI = 3.14159265359;
struct Point {
    double x;
    double y;

    Point (): x(0), y(0) {};
    Point (double x, double y): x(x), y(y) {};

    double distance(const Point& p) const;
    double direct(const Point& p) const;
    bool is_in_boundary() const;
};

std::ostream& operator<<(std::ostream& out, const Point& p);

class Rectangle {
    friend std::ostream& operator<<(std::ostream& out, const Rectangle& r);
public:
    Rectangle (double x1, double y1, double x2, double y2): bottom_left(x1, y1), top_right(x2, y2) {};

    bool is_point_in(const Point &p) const;
    bool is_linesegment_in(const Point &p1, const Point &p2) const;

    Point get_nearest_outer_corner(const Point& p) const;
    std::vector<Point> get_outer_corners() const;
private:
    Point bottom_left;
    Point top_right;

    // Used for Cohen–Sutherland algorithm to solve Line clipping problem
    typedef int Outcode;
    static const int INSIDE = 0; // 0000
    static const int LEFT = 1;   // 0001
    static const int RIGHT = 2;  // 0010
    static const int BELOW = 4; // 0100
    static const int ABOVE = 8;    // 1000

    Outcode compute_outcode(const Point &p) const;
};
std::ostream& operator<<(std::ostream& out, const Rectangle& r);

class Turtle {
    friend std::ostream& operator<< (std::ostream& out, const Turtle& t);
public:
    Turtle (std::string name, ros::NodeHandle& n);

    void callBack(const turtlesim::Pose::ConstPtr& msg);

    const std::string& getName() const {
        return name;
    }
    double getTheta() const {
        return theta;
    }
    const Point& getPosition() const {
        return position;
    }

    Point predict_pos(double dur_time) const;
    Rectangle predict_scope(double dur_time) const;

private:
    std::string name;
    ros::Subscriber pose_subscriber;

    Point position;
    double theta;

    double tmp_time;
    Point tmp_pos;

    double speed;
    double direction;
};

std::ostream& operator<< (std::ostream& out, const Turtle& t);

class TurtleDriver {
public:
    TurtleDriver (const Turtle* turtle, double speed, ros::NodeHandle& n);

    void add_villian(const Turtle* v);
    void add_target(const Turtle* t);

    void capture_targets();
    void capture(const Turtle* target);

private:
    const double speed;
    const Turtle* turtle;
    std::vector<const Turtle*> targets;
    std::vector<const Turtle*> villains;

    ros::Publisher velocity_publisher;

    Point calculate_tmp_dest(const Point& dest, double dur_time);
    void set_vel_msg(geometry_msgs::Twist& vel_msg, const Point& dest);
    static bool is_captured(const Point& s, const Point& e, std::vector<Rectangle>& villain_scopes);
};
