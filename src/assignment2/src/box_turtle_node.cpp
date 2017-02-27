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

    ros::spinOnce();
    /* cout << turtlesim_pose.x << turtlesim_pose.y << turtlesim_pose.theta << endl; */
    // I don't know why, but it seems that the ros::spinOnce must be run some times
    // before turtlesim_pose get correct value
    ros::Rate loop_rate(10);
    int count = 5;
    for (int i = 0; i < count; ++i) {
        ros::spinOnce();
        //Enforces loop rate by sleeping the cycle
        loop_rate.sleep();
    }
    /* cout << turtlesim_pose.x << turtlesim_pose.y << turtlesim_pose.theta << endl; */
    Heuristic heuristic;
    heuristic.calculate_path();
    for(vector<TurtlePosition*>::iterator it = heuristic.getPath().begin();
            it != heuristic.getPath().end();
            it++
       ) {
        (*it)->move();
    }
    return 0;
}

//In ROS callback is like a message handler.
//Whenever a message arrives ROS will call poseCallBack and pass it the new message.
void poseCallback(const turtlesim::Pose::ConstPtr & pose_message) {
    turtlesim_pose.x = pose_message->x;
    turtlesim_pose.y = pose_message->y;
    turtlesim_pose.theta = pose_message->theta;
}


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
        //cout<<(t1-t0)<<", "<<current_distance <<", "<<distance<<endl;
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

double rotate(double speed, double relative_angle) {
    double start_theta = turtlesim_pose.theta;
    speed = abs(speed);

    geometry_msgs::Twist vel_msg;
    vel_msg.linear.x = 0;
    vel_msg.linear.y = 0;
    vel_msg.linear.z = 0;
    vel_msg.angular.x = 0;
    vel_msg.angular.y = 0;
    vel_msg.angular.z = relative_angle < 0 ? -speed : speed;

    double expect_rotation = abs(relative_angle);
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
    ros::Rate r(0.5);
    /* r.sleep(); */
    /* cout << current_rotation << endl; */
    double end_theta = turtlesim_pose.theta;
    if(relative_angle > 0 && end_theta < start_theta) {
        return end_theta + 2 * PI - start_theta;
    } else if ( relative_angle < 0 && end_theta > start_theta ) {
        return end_theta - 2 * PI - start_theta;
    } else {
        return end_theta - start_theta;
    }
}

TurtlePosition::TurtlePosition(TurtlePosition& previous_pos, Action action) {
    relative_x = previous_pos.relative_x;
    relative_y = previous_pos.relative_y;
    direction = previous_pos.direction;
    switch (action) {
    case GoStraight:
        rotative_theta = 0;
        break;
    case TurnLeft:
        rotative_theta = PI / 2;
        direction = (direction + 1) % 4;
        break;
    case TurnRight:
        rotative_theta = -PI / 2;
        direction = (direction - 1 + 4) % 4;
        break;
    default:
        break;
    }
    switch (direction) {
    case 0:
        relative_x += x_unit;
        path_length = previous_pos.path_length + x_unit;
        break;
    case 1:
        relative_y -= y_unit;
        path_length = previous_pos.path_length + y_unit;
        break;
    case 2:
        relative_x -= x_unit;
        path_length = previous_pos.path_length + x_unit;
        break;
    case 3:
        relative_y += y_unit;
        path_length = previous_pos.path_length + y_unit;
        break;
    default:
        break;
    }
    /* cout << "New Position: x->" << relative_x */
    /*      << ", y->" << relative_y */
    /*      << ", Direction->" << direction */
    /*      << ", theta->" << rotative_theta */
    /*      << ", path_length->" << path_length << endl; */
}

vector<TurtlePosition> & TurtlePosition::getChildren() {
    if(children.empty()) {
        if(relative_x == 0.0 && relative_y == 0.0) {
            TurtlePosition p(*this, GoStraight);
            children.push_back(p);
        } else {
            TurtlePosition pr(*this, TurnRight);
            TurtlePosition p(*this, GoStraight);
            TurtlePosition pl(*this, TurnLeft);
            children.push_back(pr);
            children.push_back(p);
            children.push_back(pl);
        }
    }
    return children;
}

void TurtlePosition::move() {
    if(rotative_theta != 0) {
        double remain_rotation = rotative_theta;
        while(abs(remain_rotation) > 0.000001) {
            double rotate_speed = remain_rotation * 2;
            remain_rotation -= rotate(rotate_speed, remain_rotation * 0.9);
            /* cout << "Remain Rotation: " << remain_rotation << endl; */
        }
    }
    double speed = 1;
    double remain_distance = direction % 2 ? y_unit : x_unit, current_distance = 0;
    while(remain_distance > 0) {
        remain_distance -= move_forward(speed, remain_distance * 0.9);
        speed /= 2;
        /* cout << "Remain Distance: " << remain_distance << endl; */
    }
}

double TurtlePosition::getStraightDistance(TurtlePosition& pos) {
    return sqrt(pow(relative_x - pos.relative_x, 2) + pow(relative_y - pos.relative_y, 2));
}

double Heuristic::g(TurtlePosition& pos) {
    /* cout << "Path Length: " << pos.getPathLength() << endl; */
    return pos.getPathLength();
}

double Heuristic::h(TurtlePosition& pos) {
    /* cout << "Straight Distance: " << pos.getStraightDistance(orig) << endl; */
    return pos.getStraightDistance(orig);
}

void Heuristic::calculate_path() {
    double min_cost;
    TurtlePosition* min_cost_pos = NULL;
    for(vector<TurtlePosition>::iterator it = orig.getChildren().begin();
            it != orig.getChildren().end(); it++) {
        double cost = g(*it) + h(*it);
        /* cout << "Cost: " << cost << endl; */
        if(min_cost_pos == NULL || min_cost > cost) {
            min_cost = cost;
            min_cost_pos = &(*it);
        }
        /* cout << "Min Cost: " << min_cost << endl << endl; */
    }
    path.push_back(min_cost_pos);
    while((*path.rbegin()) -> getStraightDistance(orig) > 0) {
        min_cost_pos = NULL;
        TurtlePosition* p = *path.rbegin();
        for(vector<TurtlePosition>::iterator it = p -> getChildren().begin();
                it != p -> getChildren().end();
                it++) {
            double cost = g(*it) + h(*it);
            /* cout << "Cost: " << cost << endl; */
            if(min_cost_pos == NULL || min_cost > cost) {
                min_cost = cost;
                min_cost_pos = &(*it);
            }
            /* cout << "Min Cost: " << min_cost << endl; */
        }
        path.push_back(min_cost_pos);
    }
}
