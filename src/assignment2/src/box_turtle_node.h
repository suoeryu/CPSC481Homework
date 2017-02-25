#include <vector>

#include "ros/ros.h"
#include "turtlesim/Pose.h"

#pragma once

enum Action { TurnLeft, TurnRight, GoStraight };

const double y_unit = 2;
const double x_unit = 1.414 * y_unit;
const double PI = 3.14159265359;

ros::Publisher velocity_publisher;
ros::Subscriber pose_subscriber;
turtlesim::Pose turtlesim_pose;

void poseCallback(const turtlesim::Pose::ConstPtr & pose_message);
double move_forward(double speed, double distance);
double rotate(double speed, double relative_angle);

class TurtlePosition {
private:
    double relative_x; // relative x to original position
    double relative_y; // relative y to original position
    double rotative_theta; // rotate theta to previous position, angle
    int direction;

    double path_length; // path length to original position
    std::vector<TurtlePosition> children;

public:
    TurtlePosition(): relative_x(0), relative_y(0), rotative_theta(0), direction(0), path_length(0) {};
    TurtlePosition(TurtlePosition& previous_pos, Action action);

    double getPathLength(){return path_length;}
    std::vector<TurtlePosition> & getChildren();
    double getStraightDistance(TurtlePosition & pos);
    void move();
};

class Heuristic{
private:
    TurtlePosition orig;
    double g(TurtlePosition& pos);
    double h(TurtlePosition& pos);

    std::vector<TurtlePosition*> path;
public:
    void calculate_path();
    std::vector<TurtlePosition*>& getPath(){return path;}
};
