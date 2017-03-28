#include <iostream>
#include <cmath>
#include <queue>
#include <cstdlib>

using namespace std;

#include "turtlesim/Kill.h"
#include "turtlesim/Spawn.h"
#include "hw3.h"

int main(int argc, char *argv[]) {
    Environment env(argc, argv);
    env.init_turtles();
    Heuristic heuristic(env.get_targets(), env.get_villains());
    vector<Point> move_path = heuristic.calculate_path();
    for(vector<Point>::iterator it = move_path.begin(); it != move_path.end(); it++) {
        env.move_turtle1(*it);
    }
    return 0;
}

double Point::euclidean_distance(const Point &p) const {
    return sqrt(pow(p.x - x, 2) + pow(p.y - y, 2));
}

double Point::euclidean_distance(const double x, const double y) const {
    return sqrt(pow(this->x - x, 2) + pow(this->y - y, 2));
}

double Point::radian(const Point &p) const {
    double x_dist = p.x - x;
    double y_dist = p.y - y;
    return atan2(y_dist, x_dist);
}

Rectangle::Rectangle(double center_x, double center_y, double range_factor)
    : bottom_left(center_x - center_x * range_factor, center_y - center_y * range_factor),
      top_right(center_x + center_x * range_factor, center_y + center_y * range_factor) {
    double factor = 0.1;

    inner_points.push_back(Point(bottom_left.x + factor, bottom_left.y + factor));
    inner_points.push_back(Point(bottom_left.x + factor, top_right.y - factor));
    inner_points.push_back(Point(top_right.x - factor, bottom_left.y + factor));
    inner_points.push_back(Point(top_right.x - factor, top_right.y - factor));

    outer_points.push_back(Point(bottom_left.x - factor, bottom_left.y - factor));
    outer_points.push_back(Point(bottom_left.x - factor, top_right.y + factor));
    outer_points.push_back(Point(top_right.x + factor, bottom_left.y - factor));
    outer_points.push_back(Point(top_right.x + factor, top_right.y + factor));
}

Rectangle::Outcode Rectangle::compute_outcode(const Point & p) const {
    Outcode code = INSIDE;
    if(p.x < bottom_left.x) {
        code |= LEFT;
    } else if (p.x > top_right.x) {
        code |= RIGHT;
    }
    if(p.y < bottom_left.y) {
        code |= BELOW;
    } else if (p.y > top_right.y) {
        code |= ABOVE;
    }
    return code;
}

bool Rectangle::is_point_in(const Point &p) const {
    Outcode code = compute_outcode(p);
    return !code;
}

bool Rectangle::is_linesegment_in(const Point &p1, const Point &p2) const {
    Outcode code1 = compute_outcode(p1);
    Outcode code2 = compute_outcode(p2);
    if(!code1 || !code2) {
        return true;
    } else if(code1 & code2) {
        return false;
    } else if( p1.x == p2.x || p1.y == p2.y) {
        return true;
    } else {
        // y = kx + c
        const double k = (p1.y - p2.y) / (p1.x - p2.x);
        double left_y = p1.y + k * (bottom_left.x - p1.x);
        double right_y = p1.y + k * (top_right.x - p1.x);
        double top_x = p1.x + (top_right.y - p1.y) / k;
        double bottom_x = p1.x + (bottom_left.y - p1.y) / k;
        bool cross_left = left_y >= bottom_left.y && left_y <= top_right.y;
        bool cross_right = right_y >= bottom_left.y && right_y <= top_right.y;
        bool cross_top = top_x >= bottom_left.x && top_x <= top_right.x;
        bool cross_bottom = bottom_x >= bottom_left.x && bottom_x <= top_right.x;
        return cross_left || cross_right || cross_top || cross_bottom;
    }
}

State::State(const vector<Turtle> & targets, const vector<Turtle> & villains)
    : pos(0, 0), path_length(0), pre_state_ptr(NULL), villains(villains) {
    for(vector<Turtle>::const_iterator it = targets.begin(); it != targets.end(); it++) {
        this->targets.push_back(&(*it));
    }
}

State::State(const State * pre_ptr, const Point & pos)
    : pos(pos),
      path_length(pre_ptr == NULL ? 0 : pre_ptr->path_length + pos.euclidean_distance(pre_ptr->pos)),
      pre_state_ptr(pre_ptr), villains(pre_ptr->villains) {
    for(vector<const Turtle*>::const_iterator it = pre_ptr->targets.begin(); it != pre_ptr->targets.end(); it++) {
        if(!((*it)->impact(pos))) {
            targets.push_back(*it);
        }
    }
}

bool State::is_acceptable(const Point &dest) const {
    bool acceptable = true;
    for(vector<Turtle>::const_iterator it = villains.begin(); it != villains.end(); it++) {
        if(it->impact(pos, dest) || pos == dest) {
            acceptable = false;
            break;
        }
    }
    return acceptable;
}

vector<Point> State::get_possible_next_postion() const {
    vector<Point> all_position;
    for(vector<const Turtle*>::const_iterator it = targets.begin(); it != targets.end(); it++) {
        vector<Point> positions = (*it)->inner_positions();
        all_position.insert(all_position.end(), positions.begin(), positions.end());
    }
    for(vector<Turtle>::const_iterator it = villains.begin(); it != villains.end(); it++) {
        vector<Point> positions = it->outer_positions();
        all_position.insert(all_position.end(), positions.begin(), positions.end());
    }
    vector<Point> acceptable_positions;
    for(vector<Point>::iterator it = all_position.begin(); it != all_position.end(); it++) {
        if(is_acceptable(*it)) {
            acceptable_positions.push_back(*it);
        }
    }
    return acceptable_positions;
}

double State::estimate_remain_distance() const {
    queue<vector<Point> > path_queue;
    for(vector<const Turtle*>::const_iterator it = targets.begin(); it != targets.end(); it++) {
        if(path_queue.empty()) {
            vector<Point> cur_path;
            cur_path.push_back((*it)->get_position());
            path_queue.push(cur_path);
        } else {
            size_t front_size = path_queue.front().size();
            while(path_queue.front().size() == front_size) {
                for(size_t i = 0; i <= front_size; i++) {
                    vector<Point> cur_path(path_queue.front());
                    cur_path.insert(cur_path.begin() + i, (*it)->get_position());
                    path_queue.push(cur_path);
                }
                path_queue.pop();
            }
        }
    }
    double min_dist = -1;
    while(!path_queue.empty()) {
        double dist = 0;
        const Point* pre_pos = &pos;
        for(vector<Point>::iterator it = path_queue.front().begin(); it != path_queue.front().end(); it++) {
            dist += it->euclidean_distance(*pre_pos);
            pre_pos = &(*it);
        }
        /* cout << dist; */
        /* cout << endl; */
        if(min_dist == -1 || min_dist > dist) {
            min_dist = dist;
        }
        path_queue.pop();
    }
    /* cout << min_dist << endl; */
    return min_dist;
}

Heuristic::Heuristic(const std::vector<Turtle> & targets, const std::vector<Turtle> &villains) {
    /* cout << targets.size() << endl; */
    /* cout << villains.size() << endl; */
    open_state_ptr_list.push(new State(targets, villains));
    /* cout << "Init address: " << open_state_ptr_list.top() << endl; */
}

Heuristic::~Heuristic() {
    while(!open_state_ptr_list.empty()) {
        delete open_state_ptr_list.top();
        open_state_ptr_list.pop();
    }
    for(vector<const State*>::const_iterator it = closed_state_ptr_list.begin();
            it != closed_state_ptr_list.end();
            ++it) {
        delete *it;
    }
}

bool Heuristic::is_visited(const Point& p) const {
    for(vector<const State*>::const_iterator it = closed_state_ptr_list.begin();
            it != closed_state_ptr_list.end();
            it++) {
        if((*it)->get_position() == p) {
            return true;
        }
    }
    return false;
}

const vector<Point> Heuristic::calculate_path() {
    const State * current_state = open_state_ptr_list.top();
    while(!current_state->is_final()) {
        cout << current_state->get_position() << endl;
        open_state_ptr_list.pop();
        vector<Point> next_positions = current_state->get_possible_next_postion();
        for(vector<Point>::iterator it = next_positions.begin(); it != next_positions.end(); it++) {
            if(!is_visited(*it)){
                open_state_ptr_list.push(new State(current_state, *it));
            }
        }
        closed_state_ptr_list.push_back(current_state);
        current_state = open_state_ptr_list.top();
    }
    vector<Point> result_path;
    /* if(current_state->is_final()) { */
    while(!current_state->is_init()) {
        result_path.insert(result_path.begin(), current_state->get_position());
        current_state = current_state->get_pre_state();
    }
    /* } */
    return result_path;
}

double Heuristic::g(const State * s) {
    return s->get_path_length();
}

double Heuristic::h(const State * s) {
    return s->estimate_remain_distance();
}

turtlesim::Pose Environment::turtlesim_pose;

void poseCallback(const turtlesim::Pose::ConstPtr & pose_message) {
    Environment::turtlesim_pose.x = pose_message->x;
    Environment::turtlesim_pose.y = pose_message->y;
    Environment::turtlesim_pose.theta = pose_message->theta;
}

Environment::Environment(int argc, char *argv[]) {
    //Initialize a new ROS node
    ros::init(argc, argv, "hw3");

    //Main access point to communications with ROS system.
    ros::NodeHandle n;

    //Publishing to a message of type geometry_msgs::Twist on topic /turtle1/cmd_vel
    //Max of 10 messages in buffer
    velocity_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);

    //Subscribing to /turtle1/pose topic with master
    //Max of 10 messages in queue.
    //ROS will call poseCallBack when a new message arrives.
    pose_subscriber = n.subscribe("/turtle1/pose", 10, poseCallback);

    kill_client = n.serviceClient<turtlesim::Kill>("kill");
    spawn_client = n.serviceClient<turtlesim::Spawn>("spawn");

}

bool Environment::kill_turtle(string name) {
    turtlesim::Kill kill_srv;
    kill_srv.request.name = name;
    bool success = kill_client.call(kill_srv);
    if(success) {
        cout << "Killed turtle: " << name << endl;
    } else {
        cout << "Failed to kill turtle: " << name << endl;
    }
    return success;
}

bool Environment::spawn_turtle(double x, double y, double theta, std::string name) {
    turtlesim::Spawn spawn_srv;
    spawn_srv.request.x = x;
    spawn_srv.request.y = y;
    spawn_srv.request.theta = theta;
    spawn_srv.request.name = name;
    return spawn_client.call(spawn_srv);
}

bool Environment::spawn_turtle(const Turtle &turtle, double theta) {
    bool success = spawn_turtle(turtle.get_position().get_x(),
                                turtle.get_position().get_y(),
                                theta, turtle.get_name());
    if(success) {
        cout << "Spawn turtle: " << turtle << endl;
    } else {
        cout << "Failed to spawn turtle: " << turtle.get_name() << endl;
    }
    return success;
}

void Environment::init_turtles() {
    cout << "Initial turtles" << endl;
    kill_turtle("turtle1");
    spawn_turtle(0, 0, PI / 4, "turtle1");
    std::srand(117);
    for (int i = 1; i <= 3; ++i) {
        double x = 2 + double(rand()) / RAND_MAX * 8;
        double y = 2 + double(rand()) / RAND_MAX * 8;
        ostringstream name_os;
        name_os << 'T' << i;
        targets.push_back(Turtle(x, y, 0.05, name_os.str()));
    }
    for(vector<Turtle>::const_iterator it = targets.begin(); it != targets.end(); it++) {
        spawn_turtle(*it, PI / 2);
    }
    for (int i = 1; i <= 4; ++i) {
        double x = 2 + double(rand()) / RAND_MAX * 8;
        double y = 2 + double(rand()) / RAND_MAX * 8;
        ostringstream name_os;
        name_os << 'X' << i;
        villains.push_back(Turtle(x, y, 0.1, name_os.str()));
    }
    for(vector<Turtle>::const_iterator it = villains.begin(); it != villains.end(); it++) {
        spawn_turtle(*it, -PI / 2);
    }
    ros::Rate r(10);
    for (int i = 0; i < 5; ++i) {
        ros::spinOnce();
        //Enforces loop rate by sleeping the cycle
        r.sleep();
    }
}

void Environment::set_speed(const Point &dest, geometry_msgs::Twist &vel_msg) {
    double remain_distance = dest.euclidean_distance(turtlesim_pose.x, turtlesim_pose.y);
    if(remain_distance > 0.1) {
        vel_msg.linear.x = 1;
    } else if (remain_distance < 0.000001) {
        vel_msg.linear.x = 0;
    } else {
        vel_msg.linear.x = remain_distance * 10;
    }
    double rotate_radian = Point(turtlesim_pose.x, turtlesim_pose.y).radian(dest) - turtlesim_pose.theta;
    if(rotate_radian < -PI) {
        rotate_radian = 2 * PI + rotate_radian;
    } else if (rotate_radian > PI) {
        rotate_radian = 2 * PI - rotate_radian;
    }
    vel_msg.angular.z = remain_distance > 0.01 ? 20 * rotate_radian : 0;
}

void Environment::move_turtle1(const Point &dest) {
    cout << "Move to " << dest << endl;
    ros::Rate loop_rate(100);
    geometry_msgs::Twist vel_msg;
    vel_msg.linear.x = 0;
    vel_msg.linear.y = 0;
    vel_msg.linear.z = 0;
    vel_msg.angular.x = 0;
    vel_msg.angular.y = 0;
    vel_msg.angular.z = 0;
    do {
        set_speed(dest, vel_msg);
        velocity_publisher.publish(vel_msg);
        ros::spinOnce();
        loop_rate.sleep();
        check_capture();
    } while (vel_msg.linear.x != 0);
}

void Environment::check_capture() {
    Point pos(turtlesim_pose.x, turtlesim_pose.y);
    for(vector<Turtle>::iterator it = targets.begin(); it != targets.end(); it++) {
        if(it->is_alive() && it->impact(pos)) {
            kill_turtle(it->get_name());
            it->kill();
        }
    }
    for(vector<Turtle>::const_iterator it = villains.begin(); it != villains.end(); it++) {
        if(it->impact(pos)) {
            kill_turtle("turtle1");
            cout << "Turtle1 has been captured!!!!" << endl;
            exit(-1);
        }
    }
}
