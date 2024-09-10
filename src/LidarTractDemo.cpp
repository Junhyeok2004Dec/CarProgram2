#include <ros/ros.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <cmath>

#define COLLISION_RANGE 0.2  // 20cm 충돌
#define COMMUNICATE_RATE 10  // 10hz
#define BASIC_VELOCITY 1.0 // 1.0meterps

class LidarTractor {
public:
    LidarTractor() {
        ros::NodeHandle nh;

        scan_sub_ = nh.subscribe("/scan", 10, &LidarTractor::scanCallback, this);
        distance_sub_ = nh.subscribe("/scan_demo_distance", 10, &LidarTractor::distanceCallback, this);
        angle_sub_ = nh.subscribe("/scan_demo_angle", 10, &LidarTractor::angleCallback, this);
        
        odom_sub_ = nh.subscribe("/odom", 10, &LidarTractor::odomCallback, this);

        drive_pub_ = nh.advertise<ackermann_msgs::AckermannDriveStamped>("/drive", 10);
    }

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg) {
        for (size_t i = 0; i < scan_msg->ranges.size(); ++i) {
            float range = scan_msg->ranges[i];

            ROS_INFO("%f",range);


            if (range < COLLISION_RANGE) { 
                obstacle_detected_ = true;
                ROS_WARN("Cauction! going back!");
                stopVehicle();
                
            }
        }
        obstacle_detected_ = false;  // No obstacles detected
    }

    void distanceCallback(const std_msgs::Float32::ConstPtr& msg) {
        target_distance_ = msg->data;
    }

    void angleCallback(const std_msgs::Float32::ConstPtr& msg) {
        target_angle_ = msg->data;
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        if (target_distance_ > 0 && !obstacle_detected_) {
            // Current vehicle position
            geometry_msgs::Point current_position = msg->pose.pose.position;

            // Calculate target point
            float target_x = current_position.x + target_distance_ * cos(target_angle_);
            float target_y = current_position.y + target_distance_ * sin(target_angle_);

            // Calculate steering angle
            float steering_angle = atan2(target_y - current_position.y, target_x - current_position.x);

            // Move vehicle towards target point
            ackermann_msgs::AckermannDriveStamped drive_msg;
            drive_msg.drive.speed = 0.8;  // Basic speed
            drive_msg.drive.steering_angle = steering_angle*1.05;
            drive_pub_.publish(drive_msg);  
        }
    }

private:
    ros::Subscriber scan_sub_;
    ros::Subscriber distance_sub_;
    ros::Subscriber angle_sub_;
    ros::Subscriber odom_sub_;
    ros::Publisher drive_pub_;

    float target_distance_ = -1;
    float target_angle_ = -1;
    bool obstacle_detected_ = false;

    

    void stopVehicle() {
            



        ros::Time start_time = ros::Time::now();


        ackermann_msgs::AckermannDriveStamped stop_msg;

        ROS_INFO("BACK");

        stop_msg.drive.speed = -0.1;
        stop_msg.drive.steering_angle = 0.0;
        
        drive_pub_.publish(stop_msg);

        while (ros::ok() && (ros::Time::now() - start_time).toSec() < 2.0)
            
        ROS_INFO("STOP");
        stop_msg.drive.speed = 0.0; // 정지
        drive_pub_.publish(stop_msg);


    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "lidar_tractor");

    LidarTractor lidar_tractor;

    ros::spin();
    return 0;
}
