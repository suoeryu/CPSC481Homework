#include <iostream>
#include <cmath>
#include <queue>
#include <cstdlib>
#include <string>
#include <ctime>

using namespace std;

#include "turtlesim/Kill.h"
#include "turtlesim/Spawn.h"
#include "turtlesim/TeleportAbsolute.h"
#include "std_srvs/Empty.h"
#include "hw3.h"

int main(int argc, char *argv[]) {
    Environment env(argc, argv);
    cout << argc << endl;
    for (int i = 0; i < argc; ++i) {
        cout << argv[i] << endl;
    }
    unsigned seed = time(NULL);
    int targets_num = 3;
    int villains_num = 4;
    cout << seed << targets_num << villains_num << endl;
    env.random_init_turtles(seed, targets_num, villains_num);
    Heuristic heuristic(env.get_targets(), env.get_villains());
    vector<Point> move_path = heuristic.calculate_path();
    if(move_path.empty()) {
        cout << "Cannot find out solution!!!" << endl;
    } else {
        double total_distance = 0;
        for(vector<Point>::iterator it = move_path.begin(); it != move_path.end(); it++) {
            total_distance += env.move_turtle1(*it);
        }
        cout << "Total moved " << total_distance << endl;
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
    : center(center_x, center_y),
      bottom_left(center_x - center_x * range_factor, center_y - center_y * range_factor),
      top_right(center_x + center_x * range_factor, center_y + center_y * range_factor) {
    double inner_factor = 0.03;
    double outer_factor = 0.05;

    inner_points.push_back(center);
    inner_points.push_back(Point(center.x, bottom_left.y + inner_factor));
    inner_points.push_back(Point(center.x, top_right.y - inner_factor));
    inner_points.push_back(Point(bottom_left.x + inner_factor, bottom_left.y + inner_factor));
    inner_points.push_back(Point(bottom_left.x + inner_factor, (bottom_left.y + top_right.y) / 2));
    inner_points.push_back(Point(bottom_left.x + inner_factor, top_right.y - inner_factor));
    inner_points.push_back(Point(top_right.x - inner_factor, bottom_left.y + inner_factor));
    inner_points.push_back(Point(top_right.x - inner_factor, (bottom_left.y + top_right.y) / 2));
    inner_points.push_back(Point(top_right.x - inner_factor, top_right.y - inner_factor));

    outer_points.push_back(Point(bottom_left.x - outer_factor, bottom_left.y - outer_factor));
    outer_points.push_back(Point(bottom_left.x - outer_factor, top_right.y + outer_factor));
    outer_points.push_back(Point(top_right.x + outer_factor, bottom_left.y - outer_factor));
    outer_points.push_back(Point(top_right.x + outer_factor, top_right.y + outer_factor));
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
        if(!((*it)->impact(pre_ptr->get_position(), pos))) {
        /* if(!(*it)->impact(pos)) { */
            targets.push_back(*it);
        }
    }
}

bool State::is_acceptable(const Point &dest) const {
    if(pos == dest){
        return false;
    }
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
    vector<Point> remain_path;
    remain_path.push_back(pos);
    for(vector<const Turtle*>::const_iterator it = targets.begin(); it != targets.end(); it++) {
        const Point& turtle_pos = (*it)->get_position();
        double min_added_distance = -1;
        vector<Point>::iterator it_p = remain_path.begin();
        vector<Point>::iterator insert_it = it_p;
        do {
            it_p++;
            double added_distance;
            if(it_p == remain_path.end()) {
                added_distance = (it_p - 1)->euclidean_distance(turtle_pos);
            } else {
                double distance1 = (it_p - 1)->euclidean_distance(turtle_pos);
                double distance2 = turtle_pos.euclidean_distance(*it_p);
                double orig_distance = (it_p - 1)->euclidean_distance(*it_p);
                added_distance = distance1 + distance2 - orig_distance;
            }
            if(min_added_distance < 0 || min_added_distance > added_distance) {
                min_added_distance = added_distance;
                insert_it = it_p;
            }
        } while (it_p != remain_path.end());
        remain_path.insert(insert_it, turtle_pos);
    }
    double result = 0;
    for(vector<Point>::iterator it = remain_path.begin(); it + 1 != remain_path.end(); it++) {
        result += it->euclidean_distance(*(it + 1));
    }
    return result;
}

Heuristic::Heuristic(const std::vector<Turtle> & targets, const std::vector<Turtle> &villains) {
    open_state_ptr_list.push(new State(targets, villains));
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

bool Heuristic::is_visited(const Point &from, const Point& to) const {
    for(vector<const State*>::const_iterator it = closed_state_ptr_list.begin();
            it != closed_state_ptr_list.end();
            it++) {
        if((*it)->get_pre_state() == NULL) {
            continue;
        } else if((*it)->get_pre_state()->get_position() == from && (*it)->get_position() == to) {
            return true;
        }
    }
    return false;
}

const vector<Point> Heuristic::calculate_path() {
    const State * current_state = open_state_ptr_list.top();
    while(!current_state->is_final() && !open_state_ptr_list.empty()) {
        open_state_ptr_list.pop();
        vector<Point> next_positions = current_state->get_possible_next_postion();
        for(vector<Point>::iterator it = next_positions.begin(); it != next_positions.end(); it++) {
            if(!is_visited(current_state->get_position(), *it)) {
                open_state_ptr_list.push(new State(current_state, *it));
            }
        }
        closed_state_ptr_list.push_back(current_state);
        current_state = open_state_ptr_list.top();
    }
    vector<Point> result_path;
    if(current_state->is_final()) {
        while(!current_state->is_init()) {
            result_path.insert(result_path.begin(), current_state->get_position());
            current_state = current_state->get_pre_state();
        }
    }
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

    reset_client = n.serviceClient<std_srvs::Empty>("reset");
    kill_client = n.serviceClient<turtlesim::Kill>("kill");
    spawn_client = n.serviceClient<turtlesim::Spawn>("spawn");

}

bool Environment::reset() {
    std_srvs::Empty reset_srv;
    return reset_client.call(reset_srv);
}

bool Environment::kill_turtle(string name) {
    turtlesim::Kill kill_srv;
    kill_srv.request.name = name;
    bool success = kill_client.call(kill_srv);
    if(!success) {
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

void Environment::random_init_turtles(unsigned seed, int targets_num, int villains_num) {
    cout << "Initial turtles" << endl;
    reset();
    kill_turtle("turtle1");
    spawn_turtle(0, 0, PI / 4, "turtle1");
    /* cout << "Seed: " << seed << endl; */
    std::srand(seed);
    for (int i = 1; i <= targets_num; ++i) {
        double x = 2 + double(rand()) / RAND_MAX * 9;
        double y = 2 + double(rand()) / RAND_MAX * 9;
        ostringstream name_os;
        name_os << 'T' << i;
        targets.push_back(Turtle(x, y, 0.05, name_os.str()));
    }
    for (int i = 1; i <= villains_num; ++i) {
        double x = 2 + double(rand()) / RAND_MAX * 9;
        double y = 2 + double(rand()) / RAND_MAX * 9;
        ostringstream name_os;
        name_os << 'X' << i;
        villains.push_back(Turtle(x, y, 0.1, name_os.str()));
    }
    for(vector<Turtle>::const_iterator it = targets.begin(); it != targets.end(); it++) {
        spawn_turtle(*it, PI / 2);
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
    const double error = 0.0001;
    double remain_distance = dest.euclidean_distance(turtlesim_pose.x, turtlesim_pose.y);
    if(remain_distance > 0.1) {
        vel_msg.linear.x = 1;
    } else if (remain_distance < error) {
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
    vel_msg.angular.z = remain_distance > 0.1 ? 30 * rotate_radian : 0;
}

double Environment::move_turtle1(const Point &dest) {
    cout << "Move to " << dest << endl;
    double total_distance = 0;
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
        double t0 = ros::Time::now().toSec();
        velocity_publisher.publish(vel_msg);
        ros::spinOnce();
        loop_rate.sleep();
        double t1 = ros::Time::now().toSec();
        total_distance += vel_msg.linear.x * (t1-t0);
        check_capture();
    } while (vel_msg.linear.x != 0);
    return total_distance;
}

void Environment::check_capture() {
    Point pos(turtlesim_pose.x, turtlesim_pose.y);
    for(vector<Turtle>::iterator it = targets.begin(); it != targets.end(); it++) {
        if(it->is_alive() && it->impact(pos)) {
            kill_turtle(it->get_name());
            it->kill();
            cout << "Turtle1 has captured " << it->get_name() << endl;
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
