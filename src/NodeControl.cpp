#include <ros/ros.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <cmath>

std::vector<geometry_msgs::Point> waypoints;
int current_waypoint = 0;
geometry_msgs::Point current_position;

void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
    current_position = msg->pose.pose.position;
}

float distanceToGoal(const geometry_msgs::Point &goal) {
    return sqrt(pow(goal.x - current_position.x, 2) + pow(goal.y - current_position.y, 2));
}

float computeSteeringAngle(const geometry_msgs::Point &goal) {
    return atan2(goal.y - current_position.y, goal.x - current_position.x);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "waypoint_follower");
    ros::NodeHandle nh;

    ros::Publisher drive_pub = nh.advertise<ackermann_msgs::AckermannDriveStamped>("/drive", 10);
    ros::Subscriber odom_sub = nh.subscribe("/odom", 10, odomCallback);

    // Define waypoints (for example, add actual coordinates)
    geometry_msgs::Point p1, p2, p3;
    p1.x = 1.0; p1.y = 1.0;
    p2.x = 2.0; p2.y = 2.0;
    p3.x = 3.0; p3.y = 3.0;
    waypoints = {p1, p2, p3};

    ros::Rate loop_rate(10);

    while (ros::ok() && current_waypoint < waypoints.size()) {
        ackermann_msgs::AckermannDriveStamped drive_msg;

        geometry_msgs::Point goal = waypoints[current_waypoint];
        
        if (distanceToGoal(goal) < 0.1) {  // Close to the waypoint
            current_waypoint++;  // Move to the next waypoint
        } else {
            drive_msg.drive.speed = 1.0;  // Set desired speed
            drive_msg.drive.steering_angle = computeSteeringAngle(goal);  // Set steering angle
            drive_pub.publish(drive_msg);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
