#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <cmath>
#include <vector>

static float COLLISION_RANGE = 1.4f; 

std::vector<geometry_msgs::PointStamped> waypoints;
ackermann_msgs::AckermannDriveStamped drive_data; //현재 주행 상태 get
int current_waypoint_index = 0;




void pointCallback(const geometry_msgs::PointStamped::ConstPtr &msg) {
    waypoints.push_back(*msg);
    ROS_INFO("Received waypoint %ld: x = %f, y = %f", waypoints.size(), msg->point.x, msg->point.y);
}

void driveCallback(const ackermann_msgs::AckermannDriveStamped::ConstPtr &msg) {
    drive_data = *msg;
    ROS_INFO("Received Position Steering %f rad , Speed = %f" , drive_data.drive.steering_angle, drive_data.drive.speed);

    
    
}

float distanceToGoal(const geometry_msgs::Point &goal, const geometry_msgs::Point &current_position) {
    return sqrt(pow(goal.x - current_position.x, 2) + pow(goal.y - current_position.y, 2));
}

float computeSteeringAngle(const geometry_msgs::Point &goal, const geometry_msgs::Point &current_position) {
    return atan2(goal.y - current_position.y, goal.x - current_position.x);
}

//::TODO :: deadman-switch 생성 -> 일정 시간 이상동안 도달하지 않으면 다음 waypoint로 파견.
void odomCallback(const nav_msgs::Odometry::ConstPtr &msg, ros::Publisher &drive_pub) {
    if (current_waypoint_index < waypoints.size()) {
        geometry_msgs::Point current_position = msg->pose.pose.position;
        auto target = waypoints[current_waypoint_index].point;
        float dist = distanceToGoal(target, current_position);

        ROS_INFO("Current Position: x = %f, y = %f", current_position.x, current_position.y);
        ROS_INFO("Distance to target: %f", dist);

        if (dist < COLLISION_RANGE) {
            current_waypoint_index++; // 도착했습니다!
            ROS_INFO("Reached waypoint %d, moving to next", current_waypoint_index);
        }
        
         else {
            ackermann_msgs::AckermannDriveStamped drive_msg;

            drive_msg.drive.speed = 1.0;
            drive_msg.drive.steering_angle = computeSteeringAngle(target, current_position);           
            ROS_INFO("Steering angle: %f", drive_msg.drive.steering_angle);
            drive_pub.publish(drive_msg);

            if(drive_msg.drive.steering_angle == drive_data.drive.steering_angle) {
                ROS_INFO("yeeh!");
            }
        }
    } else {


        if(waypoints.size() > 1) { // yes waypoints
        } else {
            // No waypoints;
            ROS_INFO("NO waypoints recognized");
        }

    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "point_follower");
    ros::NodeHandle nh;

    ros::Publisher drive_pub = nh.advertise<ackermann_msgs::AckermannDriveStamped>("/drive", 100);
    
    ros::Subscriber drive_sub = nh.subscribe("/drive", 100, driveCallback);
    ros::Subscriber point_sub = nh.subscribe("/clicked_point", 100, pointCallback);
    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("/odom", 100, boost::bind(odomCallback, _1, boost::ref(drive_pub)));



    ros::spin();
    return 0;
}
