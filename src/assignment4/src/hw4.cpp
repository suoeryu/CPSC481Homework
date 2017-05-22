#include <iostream>
#include <cmath>

using namespace std;

#include "hw4.h"

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "hw4");
    ros::NodeHandle n;
    Turtle turtle1("turtle1", n);
    Turtle t1("T1", n);
    Turtle t2("T2", n);
    Turtle t3("T3", n);
    Turtle x1("X1", n);
    Turtle x2("X2", n);
    Turtle x3("X3", n);
    ros::Rate loop_rate(10);
    for (int i = 0; i < 5; ++i) {
        ros::spinOnce();
        loop_rate.sleep();
    }
    TurtleDriver td(&turtle1, 4, n);
    td.add_target(&t1);
    td.add_target(&t2);
    td.add_target(&t3);
    td.add_villian(&x1);
    td.add_villian(&x2);
    td.add_villian(&x3);
    td.capture_targets();
    return 0;
}

double Point::distance(const Point& p) const {
    return sqrt(pow(p.x - x, 2) + pow(p.y - y, 2));
}

double Point::direct(const Point& p) const {
    return atan2(p.y - y, p.x - x);
}

bool Point::is_in_boundary() const {
    return x > 0 && x < 11 && y > 0 && y < 11;
}

ostream& operator<<(ostream& out, const Point& p) {
    out << "(" << p.x << ", " << p.y << ")";
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

bool Rectangle::is_point_in(const Point& p) const {
    return compute_outcode(p) == INSIDE;
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

vector<Point> Rectangle::get_outer_corners() const {
    vector<Point> result;
    result.push_back(Point(bottom_left.x - 0.2, bottom_left.y - 0.2));
    result.push_back(Point(top_right.x + 0.2, bottom_left.y - 0.2));
    result.push_back(Point(top_right.x + 0.2, top_right.y + 0.2));
    result.push_back(Point(bottom_left.x - 0.2, top_right.y + 0.2));
    return result;
}

Point Rectangle::get_nearest_outer_corner(const Point& p) const {
    vector<Point> corners = get_outer_corners();
    double min_distance = -1;
    Point result;
    for(vector<Point>::iterator it = corners.begin(); it != corners.end(); it++) {
        if(it->is_in_boundary()) {
            double d = it->distance(p);
            if(min_distance < 0 || d < min_distance) {
                min_distance = d;
                result = *it;
            }
        }
    }
    return result;
}

ostream& operator<<(ostream& out, const Rectangle& r) {
    out << "Rectangle: [" << r.bottom_left  << ", " << r.top_right << "]";
}

Turtle::Turtle(string name, ros::NodeHandle& n) : name(name) {
    string topic = "/" + name + "/pose";
    ros::topic::waitForMessage<turtlesim::Pose>(topic);
    pose_subscriber = n.subscribe(topic, 10, &Turtle::callBack, this);
    tmp_time = -1;
    speed = 0;
}

void Turtle::callBack(const turtlesim::Pose::ConstPtr& msg) {
    double cur_time = ros::Time::now().toSec();
    position.x = msg->x;
    position.y = msg->y;
    theta = msg->theta;

    if(tmp_time == -1) {
        tmp_pos = position;
        tmp_time = cur_time;
    } else if(cur_time - tmp_time > 0.01) {
        double dist_x = position.x - tmp_pos.x;
        double dist_y = position.y - tmp_pos.y;

        speed = tmp_pos.distance(position) / (cur_time - tmp_time);
        /* cout << speed << endl; */
        direction = atan2(dist_y, dist_x);
        if(cur_time - tmp_time > 0.01) {
            tmp_pos = position;
            tmp_time = cur_time;
        }
    }
}

Point Turtle::predict_pos(double dur_time) const {
    double dist = speed * dur_time;
    double dx = position.x + dist * cos(direction);
    double dy = position.y + dist * sin(direction);
    double theta0 = position.direct(Point(0, 0));
    double theta1 = position.direct(Point(11, 0));
    double theta2 = position.direct(Point(11, 11));
    double theta3 = position.direct(Point(0, 11));
    if(direction >= theta0 && direction < theta1) {
        if(dy < 0) {
            double ly = -dy;
            dy = ly;
            dx = dx + 2 * ly / tan(direction);
        }
    } else if (direction >= theta1 && direction < theta2) {
        if(dx > 11) {
            double lx = (dx - 11);
            dx = 11 - lx;
            dy = dy - 2 * lx * tan(direction);
        }
    } else if (direction >= theta2 && direction < theta3) {
        if(dy > 11) {
            double ly = dy - 11;
            dy = 11 - ly;
            dx = dx - 2 * ly / tan(direction);
        }
    } else {
        if(dx < 0) {
            double lx = -dx;
            dx = lx;
            dy = dy + 2 * lx * tan(direction);
        }
    }
    if(dx <= 0) {
        dx += 0.1;
    } else if (dx >= 11) {
        dx -= 0.1;
    }
    if(dy <= 0) {
        dy += 0.1;
    } else if (dy >= 11) {
        dy -= 0.1;
    }
    return Point(dx, dy);
}

Rectangle Turtle::predict_scope(double dur_time) const {
    double x1 = position.x, y1 = position.y;
    double x2 = x1, y2 = y1;
    double distance = speed * dur_time;
    double dx = x1 + distance * sin(direction);
    double dy = y1 + distance * cos(direction);
    double theta0 = position.direct(Point(0, 0));
    double theta1 = position.direct(Point(11, 0));
    double theta2 = position.direct(Point(11, 11));
    double theta3 = position.direct(Point(0, 11));
    if(direction >= theta0 && direction < theta1) {
        if(dy < 0) {
            double y_inc = -dy;
            double x_inc = y_inc / tan(direction);
            dx = dx + x_inc;
            dy = dy + y_inc;
            x2 = dx + x_inc;
            y2 = dy + y_inc;
        }
    } else if (direction >= theta1 && direction < theta2) {
        if(dx > 11) {
            double x_inc = 11 - dx;
            double y_inc = x_inc * tan(direction);
            dx = dx + x_inc;
            dy = dy + y_inc;
            x2 = dx + x_inc;
            y2 = dy + y_inc;
        }
    } else if (direction >= theta2 && direction < theta3) {
        if(dy > 11) {
            double y_inc = 11 - dy;
            double x_inc = y_inc / tan(direction);
            dx = dx + x_inc;
            dy = dy + y_inc;
            x2 = dx + x_inc;
            y2 = dy + y_inc;
        }
    } else {
        if(dx < 0) {
            double x_inc = -dx;
            double y_inc = x_inc * tan(direction);
            dx = dx + x_inc;
            dy = dy + y_inc;
            x2 = dx + x_inc;
            y2 = dy + y_inc;
        }
    }
    return Rectangle(fmin(x1, fmin(x2, dx)) - 0.5, fmin(y1, fmax(y2, dy)) - 0.5,
                     fmax(x1, fmax(x2, dx)) + 0.5, fmax(y1, fmax(y2, dy)) + 0.5);
}

ostream& operator<<(ostream& out, const Turtle& turtle) {
    out << turtle.name << ": " << turtle.position << " Theta: " << turtle.theta;
}

TurtleDriver::TurtleDriver(const Turtle* turtle, double speed, ros::NodeHandle& n): turtle(turtle), speed(speed) {
    velocity_publisher = n.advertise<geometry_msgs::Twist>("/" + turtle->getName() + "/cmd_vel", 10);
}

void TurtleDriver::add_villian(const Turtle* v) {
    villains.push_back(v);
}

void TurtleDriver::add_target(const Turtle* t) {
    targets.push_back(t);
}

void TurtleDriver::capture_targets() {
    while(!targets.empty()) {
        double min_distance = -1;
        vector<const Turtle*>::iterator min_it;
        for(vector<const Turtle*>::iterator it = targets.begin(); it != targets.end(); it++) {
            double d = turtle->getPosition().distance((*it)->getPosition());
            if(min_distance < 0 ||  d < min_distance) {
                min_distance = d;
                min_it = it;
            }
        }
        capture(*min_it);
        cout << "Capture " << (*min_it)->getName() << endl;
        targets.erase(min_it);
    }
}

void TurtleDriver::capture(const Turtle* target) {
    /* cout << *turtle << endl; */
    /* cout << "Target: " << *target << endl; */
    /* cout << turtle->getPosition().direct(target->getPosition()) << endl; */
    /* cout << turtle->getPosition().distance(target->getPosition()) << endl; */
    ros::Rate loop_rate(100);
    geometry_msgs::Twist vel_msg;
    vel_msg.linear.x = 0;
    vel_msg.linear.y = 0;
    vel_msg.linear.z = 0;
    vel_msg.angular.x = 0;
    vel_msg.angular.y = 0;
    vel_msg.angular.z = 0;
    int i = 0;
    double distance;
    Point dest;
    while((distance = turtle->getPosition().distance(target->getPosition())) > 0.4) {
        double dur_time = distance / speed;
        dest = target->predict_pos(dur_time);
        dest = calculate_tmp_dest(dest, dur_time);
        set_vel_msg(vel_msg, dest);
        velocity_publisher.publish(vel_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    vel_msg.angular.z = 0;
    vel_msg.linear.x = 0;
    velocity_publisher.publish(vel_msg);
    ros::spinOnce();
    loop_rate.sleep();
}

Point TurtleDriver::calculate_tmp_dest(const Point& dest, double dur_time) {
    Point cur_pos = turtle->getPosition();
    vector<Rectangle> villain_scopes;
    for(vector<const Turtle*>::iterator it = villains.begin(); it != villains.end(); it++) {
        Rectangle scope = (*it)->predict_scope(dur_time);
        if(scope.is_point_in(cur_pos)){
            return scope.get_nearest_outer_corner(cur_pos);
        }
        if(scope.is_point_in(dest)){
            return cur_pos;
        }
        /* cout << scope << endl; */
        villain_scopes.push_back(scope);
    }
    if(is_captured(cur_pos, dest, villain_scopes)) {
        /* cout << "Captured!!" << endl; */
        double min_distance = -1;
        Point tmp_dest = Point(-1, -1);
        for(vector<Rectangle>::iterator rit = villain_scopes.begin(); rit != villain_scopes.end(); rit++) {
            vector<Point> corners = rit->get_outer_corners();
            for(vector<Point>::iterator pit = corners.begin(); pit != corners.end(); pit++) {
                if(pit->is_in_boundary()) {
                    /* cout << *pit << endl; */
                    if(!is_captured(turtle->getPosition(), *pit, villain_scopes)
                            && !is_captured(*pit, dest, villain_scopes)) {
                        double tmp_distance = pit->distance(cur_pos) + pit->distance(dest);
                        if(min_distance < 0 || tmp_distance < min_distance) {
                            min_distance = tmp_distance;
                            tmp_dest = *pit;
                            /* cout << "TMP: " << tmp_dest << endl; */
                        }
                    }
                }
            }
        }
        if(tmp_dest.is_in_boundary()) {
            return tmp_dest;
        } else {
            return cur_pos;
        }
    } else {
        return dest;
    }
}

bool TurtleDriver::is_captured(const Point& s, const Point& e, vector<Rectangle>& villain_scopes) {
    for(vector<Rectangle>::iterator it = villain_scopes.begin(); it != villain_scopes.end(); it++) {
        /* cout << *it << endl; */
        /* cout << s << e << endl; */
        if(it->is_linesegment_in(s, e)) {
            return true;
        }
    }
    return false;
}

void TurtleDriver::set_vel_msg(geometry_msgs::Twist& vel_msg, const Point& dest) {
    if(dest.distance(turtle->getPosition()) < 0.01 || !dest.is_in_boundary()) {
        vel_msg.linear.x = 0;
        vel_msg.angular.z = 0;
    } else {
        vel_msg.linear.x = speed;
        double direction = turtle->getPosition().direct(dest);
        double rotate_theta = direction - turtle->getTheta();
        if (rotate_theta > PI) {
            rotate_theta =  rotate_theta - 2 * PI;
        } else if (rotate_theta < -PI) {
            rotate_theta =  rotate_theta + 2 * PI;
        }
        vel_msg.angular.z = 20 * rotate_theta;
        /* vel_msg.angular.z = abs(rotate_speed) > 10 ? 10 * abs(rotate_speed) / rotate_speed : rotate_speed; */
        /* cout << vel_msg.angular.z << endl; */

    }
}
