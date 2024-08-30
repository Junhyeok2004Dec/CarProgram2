#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <cmath>
#include <map>

std::map<uint32_t, geometry_msgs::PointStamped> waypoint_map;
uint32_t last_seq = 0;

void pointCallback(const geometry_msgs::PointStamped::ConstPtr &msg) {
    waypoint_map[msg->header.seq] = *msg;
    ROS_INFO("Received waypoint %d: x = %f, y = %f", msg->header.seq, msg->point.x, msg->point.y);
}

float distanceToGoal(const geometry_msgs::Point &goal, const geometry_msgs::Point &current_position) {
    return sqrt(pow(goal.x - current_position.x, 2) + pow(goal.y - current_position.y, 2));
}

float computeSteeringAngle(const geometry_msgs::Point &goal, const geometry_msgs::Point &current_position) {
    return atan2(goal.y - current_position.y, goal.x - current_position.x);
}

void odomCallback(const nav_msgs::Odometry::ConstPtr &msg, ros::Publisher &drive_pub) {
    if (!waypoint_map.empty() && waypoint_map.count(last_seq + 1)) {
        geometry_msgs::Point current_position = msg->pose.pose.position;
        auto target = waypoint_map[last_seq + 1];
        float dist = distanceToGoal(target.point, current_position);

        ROS_INFO("Current Position: x = %f, y = %f", current_position.x, current_position.y);
        ROS_INFO("Distance to target: %f", dist);

        if (dist < 0.1) {
            last_seq++;
            ROS_INFO("Reached waypoint %d, moving to next", last_seq);
        } else {
            ackermann_msgs::AckermannDriveStamped drive_msg;
            drive_msg.drive.speed = 1.0;
            drive_msg.drive.steering_angle = computeSteeringAngle(target.point, current_position);
            ROS_INFO("Steering angle: %f", drive_msg.drive.steering_angle);
            drive_pub.publish(drive_msg);
        }
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "point_follower");
    ros::NodeHandle nh;

    ros::Publisher drive_pub = nh.advertise<ackermann_msgs::AckermannDriveStamped>("/drive", 100);
    ros::Subscriber point_sub = nh.subscribe("/target_point", 100, pointCallback);
    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("/odom", 100, boost::bind(odomCallback, _1, boost::ref(drive_pub)));
  
    ros::Rate loop_rate(100);  // 10 Hz loop rate

    ros::spin();
    return 0;
}
