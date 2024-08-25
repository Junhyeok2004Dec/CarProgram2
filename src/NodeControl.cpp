#include <ros/ros.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <vector>
#include <functional>

// Waypoints
std::vector<geometry_msgs::Point> waypoints = [] {
    std::vector<geometry_msgs::Point> wps;
    for (int i = 0; i < 5; ++i) {
        geometry_msgs::Point p;
        p.x = i * 1.0;  // 0,1,2,3,4
        p.y = i * 1.0;  
        wps.push_back(p);
    }
    return wps;
}();

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
