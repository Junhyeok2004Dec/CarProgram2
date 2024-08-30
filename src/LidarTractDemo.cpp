#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h>
#include <tf/transform_datatypes.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <cmath>
#include <vector>



geometry_msgs::Point current_position, destination;

geometry_msgs::Point destination; // 도착지점 -> scan_angle 방위의 scan_distance 거리 
geometry_msgs::Point currentPOS; // 현재지점 -> 제곧내

static float COLLISION_RANGE = 1.0f; 

ackermann_msgs::AckermannDriveStamped drive_data; //현재 주행 상태 get
int current_waypoint_index = 0;

std_msgs::Float32 scan_length_msg, scan_angle_msg;


geometry_msgs::Point calcPositionFromScan(const std_msgs::Float32 &distance, const std_msgs::Float32 &angle)  {

    geometry_msgs::Point dest;
    dest.x = current_position.x + distance * cos(angle);
    dest.y = current_position.y + distance * sin(angle);

    return dest;


    

}

void distanceCallback(const std_msgs::Float32& msg) {
    scan_length_msg = *msg;

}

void angleCallback(const std_msgs::Float32 &msg) {
    scan_angle_msg = *msg;
}

//void pointCallback(const geometry_msgs::PointStamped::ConstPtr &msg) {
//    waypoints.push_back(*msg);
//    ROS_INFO("Received waypoint %ld: x = %f, y = %f", waypoints.size(), msg->point.x, msg->point.y);
//}

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


    current_position = msg->pose.pose.position;
    //auto target = waypoints[current_waypoint_index].point;

    auto target = calcPositionFromScan(scan_sub_length, scan_sub_angle);
    float dist = distanceToGoal(target, current_position);

    ROS_INFO("Current Position: x = %f, y = %f", current_position.x, current_position.y);
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

}


int main(int argc, char **argv) {
    ros::init(argc, argv, "lidar_tractor");
    ros::NodeHandle nh;


    //init
   // destination = {0.0, 0.0};
    //currentPOS = {-1.0, -1.0};

    ros::Publisher drive_pub = nh.advertise<ackermann_msgs::AckermannDriveStamped>("/drive", 100);
    
    ros::Subscriber drive_sub = nh.subscribe("/drive", 100, driveCallback);
    //ros::Subscriber point_sub = nh.subscribe("/clicked_point", 100, pointCallback);
    ros::Subscriber scan_sub_length = nh.subscribe("/scan_demo_distance", 10, distanceCallback);
    ros::Subscriber scan_sub_angle = nh.subscribe("/scan_demo_angle", 10, angleCallback);
    
    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("/odom", 100, boost::bind(odomCallback, _1, boost::ref(drive_pub)));

    ros::spin();
    return 0;
}
