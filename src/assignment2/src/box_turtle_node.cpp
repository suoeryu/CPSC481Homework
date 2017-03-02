#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

#include "box_turtle_node.h"

using namespace std;

int main(int argc, char *argv[]) {

    //Initialize a new ROS node
    ros::init(argc, argv, "box_turtle");

    //Main access point to communications with ROS system.
    ros::NodeHandle n;

    //Publishing to a message of type geometry_msgs::Twist on topic /turtle1/cmd_vel
    //Max of 10 messages in buffer
    velocity_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);

    //Subscribing to /turtle1/pose topic with master
    //Max of 10 messages in queue.
    //ROS will call poseCallBack when a new message arrives.
    pose_subscriber = n.subscribe("/turtle1/pose", 10, poseCallback);

    // I don't know why, but it seems that the ros::spinOnce must be run some times
    // before turtlesim_pose get correct value
    ros::Rate loop_rate(10);
    for (int i = 0; i < 5; ++i) {
        ros::spinOnce();
        //Enforces loop rate by sleeping the cycle
        loop_rate.sleep();
    }
    Heuristic heuristic;
    // calculate the path of turtle
    heuristic.calculate_path();
    // move turtle follow the path
    heuristic.move_turtle();
    return 0;
}

//In ROS callback is like a message handler.
//Whenever a message arrives ROS will call poseCallBack and pass it the new message.
void poseCallback(const turtlesim::Pose::ConstPtr & pose_message) {
    turtlesim_pose.x = pose_message->x;
    turtlesim_pose.y = pose_message->y;
    turtlesim_pose.theta = pose_message->theta;
}

// make the turtle move forward using given speed and distance
// return the actual distance the turtle moved
double move_forward(double speed, double distance) {
    double start_x = turtlesim_pose.x;
    double start_y = turtlesim_pose.y;
    speed = abs(speed);
    geometry_msgs::Twist vel_msg;
    vel_msg.linear.x = speed;
    vel_msg.linear.y = 0;
    vel_msg.linear.z = 0;
    vel_msg.angular.x = 0;
    vel_msg.angular.y = 0;
    vel_msg.angular.z = 0;

    double expect_distance = abs(distance);
    double current_distance = 0.0;
    double t0 = ros::Time::now().toSec();
    ros::Rate loop_rate(100);
    do {
        velocity_publisher.publish(vel_msg);
        double t1 = ros::Time::now().toSec();
        current_distance = speed * (t1 - t0);
        ros::spinOnce();
        loop_rate.sleep();
    } while(current_distance < expect_distance);
    vel_msg.linear.x = 0;
    velocity_publisher.publish(vel_msg);
    ros::spinOnce();
    /* ros::Rate r(0.5); */
    /* r.sleep(); */
    double end_x = turtlesim_pose.x;
    double end_y = turtlesim_pose.y;
    return sqrt(pow(end_x - start_x, 2) + pow(end_y - start_y, 2));
}

// Rotate the turtle using given speed and radian, return the actual rotated ridian
double rotate(double speed, double relative_radian) {
    double start_theta = turtlesim_pose.theta;
    speed = abs(speed);

    geometry_msgs::Twist vel_msg;
    vel_msg.linear.x = 0;
    vel_msg.linear.y = 0;
    vel_msg.linear.z = 0;
    vel_msg.angular.x = 0;
    vel_msg.angular.y = 0;
    vel_msg.angular.z = relative_radian < 0 ? -speed : speed;

    double expect_rotation = abs(relative_radian);
    double current_rotation = 0.0;
    double t0 = ros::Time::now().toSec();
    ros::Rate loop_rate(100);
    do {
        velocity_publisher.publish(vel_msg);
        double t1 = ros::Time::now().toSec();
        current_rotation = speed * (t1 - t0);
        ros::spinOnce();
        loop_rate.sleep();
    } while(current_rotation < expect_rotation);

    vel_msg.angular.z = 0;
    velocity_publisher.publish(vel_msg);
    ros::spinOnce();
    double end_theta = turtlesim_pose.theta;
    if(relative_radian > 0 && end_theta < start_theta) {
        return end_theta + 2 * PI - start_theta;
    } else if ( relative_radian < 0 && end_theta > start_theta ) {
        return end_theta - 2 * PI - start_theta;
    } else {
        return end_theta - start_theta;
    }
}

double Coordinate::getDistance(const Coordinate& other) const {
    return sqrt(pow(x - other.x, 2) + pow(y - other.y, 2));
}

void Coordinate::add(double x_value, double y_value) {
    x += x_value;
    y += y_value;
}

TurtleState::TurtleState(Coordinate& goal)
    : relative_coordinate(0, 0), goal_coordinate(goal),
      rotative_theta(0), direction(East), path_length(0),
      pre_state_ptr(NULL) {};

TurtleState::TurtleState(TurtleState* parent_ptr, Action action)
    : relative_coordinate(parent_ptr->relative_coordinate),
      goal_coordinate(parent_ptr->goal_coordinate),
      direction(parent_ptr->direction), pre_state_ptr(parent_ptr) {
    switch (action) {
    case GoStraight:
        rotative_theta = 0;
        break;
    case TurnLeft:
        rotative_theta = PI / 2;
        direction = static_cast<Direction>((int(direction) + 1) % 4);
        break;
    case TurnRight:
        rotative_theta = -PI / 2;
        direction = static_cast<Direction>((int(direction) - 1 + 4) % 4);
        break;
    default:
        break;
    }
    switch (direction) {
    case East:
        relative_coordinate.add(x_unit, 0);
        path_length = parent_ptr->path_length + x_unit;
        break;
    case North:
        relative_coordinate.add(0, -y_unit);
        path_length = parent_ptr->path_length + y_unit;
        break;
    case West:
        relative_coordinate.add(-x_unit, 0);
        path_length = parent_ptr->path_length + x_unit;
        break;
    case South:
        relative_coordinate.add(0, y_unit);
        path_length = parent_ptr->path_length + y_unit;
        break;
    default:
        break;
    }
}

void TurtleState::move() {
    // rotate the turtle if needed
    if(rotative_theta != 0) {
        // the angle rotate function rotated does not exactly same as we expect, use a progress strategy
        double remain_rotation = rotative_theta;
        while(abs(remain_rotation) > 0.000001) {
            double rotate_speed = remain_rotation * 2;
            remain_rotation -= rotate(rotate_speed, remain_rotation * 0.9);
            /* cout << "Remain Rotation: " << remain_rotation << endl; */
        }
    }
    // the distance move_forword function moved does not exactly same as we expect, use a progress strategy
    double speed = 1;
    double remain_distance = direction % 2 ? y_unit : x_unit, current_distance = 0;
    while(remain_distance > 0) {
        remain_distance -= move_forward(speed, remain_distance * 0.9);
        speed /= 2;
        /* cout << "Remain Distance: " << remain_distance << endl; */
    }
}

double TurtleState::getStraightDistanceToGoal() const {
    return relative_coordinate.getDistance(goal_coordinate);
}

double Heuristic::g(const TurtleState * state_ptr) {
    return state_ptr->getPathLength();
}

double Heuristic::h(const TurtleState * state_ptr) {
    return state_ptr->getStraightDistanceToGoal();
}

Heuristic::Heuristic(): goal_coordinate(0, 0) {
    orig_state_ptr = new TurtleState(goal_coordinate);
    // when turtle at original position, the only action is go straight
    open_state_ptr_list.push(new TurtleState(orig_state_ptr, GoStraight));
    closed_state_ptr_list.push_back(orig_state_ptr);
}

Heuristic::~Heuristic() {
    orig_state_ptr = NULL;
    for(vector<TurtleState*>::iterator it = closed_state_ptr_list.begin();
            it != closed_state_ptr_list.end();
            ++it) {
        delete *it;
    }
    while(!open_state_ptr_list.empty()) {
        delete open_state_ptr_list.top();
        open_state_ptr_list.pop();
    }
}

void Heuristic::calculate_path() {
    if(path.empty()) {
        TurtleState * current_state_ptr = open_state_ptr_list.top();
        open_state_ptr_list.pop();
        while(current_state_ptr->getStraightDistanceToGoal() > 0) {
            open_state_ptr_list.push(new TurtleState(current_state_ptr, TurnRight));
            open_state_ptr_list.push(new TurtleState(current_state_ptr, GoStraight));
            open_state_ptr_list.push(new TurtleState(current_state_ptr, TurnLeft));
            closed_state_ptr_list.push_back(current_state_ptr);
            current_state_ptr = open_state_ptr_list.top();
            open_state_ptr_list.pop();
        }
        closed_state_ptr_list.push_back(current_state_ptr);
        while(current_state_ptr->getParentState()) {
            path.insert(path.begin(), current_state_ptr);
            current_state_ptr = current_state_ptr->getParentState();
        }
    }
}

void Heuristic::move_turtle() {
    if(path.empty()){
        calculate_path();
    }
    for(vector<TurtleState*>::iterator it = path.begin();
            it != path.end();
            it++) {
        (*it)->move();
    }
}
