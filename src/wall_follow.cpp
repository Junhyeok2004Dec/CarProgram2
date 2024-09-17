#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <vector>
#include <algorithm>
#include <numeric>
#include <limits>

class WallFollow {
public:
    WallFollow() {
        ros::NodeHandle nh;

        // Initialize subscribers and publishers
        lap_sub = nh.subscribe("/lap", 10, &WallFollow::lapCallback, this);
        scan_sub = nh.subscribe("/scan", 10, &WallFollow::scanCallback, this);
        drive_pub = nh.advertise<ackermann_msgs::AckermannDriveStamped>("/drive", 10);

        // Initialize control parameters
        max_speed = 1.2;
        min_speed = 0.1;
        kp = 0.5;
        ki = 0.05;
        kd = 0.5;
        integral_error = 0;
        last_error = 0;
        max_integral = 1.0;

        // Initialize distances
        left_distance = 0;
        right_distance = 0;
        front_distance = 0;

        // Timer to publish distances at 10Hz
        timer = nh.createTimer(ros::Duration(0.1), &WallFollow::publishDistances, this);
    }

    void lapCallback(const std_msgs::String::ConstPtr& msg) {
        lap = msg->data;
    }

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg) {

        bool stopVehicle = false;
        int flag = false; // stopVehicle
        if (lap != "1" && lap != "0") {

            stopVehicle = true;
            flag = true;
            ROS_INFO("rrt is running");
            return;
        }

        std::vector<float> ranges = scan_msg->ranges;
        int range_size = ranges.size();

        // Extract ranges using 270-degree LiDAR convention
        std::vector<float> front_ranges(ranges.begin() + range_size / 2 - 15, ranges.begin() + range_size / 2 + 15);
        std::vector<float> left_ranges(ranges.begin() + 3 * range_size / 4 - 15, ranges.begin() + 3 * range_size / 4 + 15);
        std::vector<float> right_ranges(ranges.begin() + range_size / 4 - 15, ranges.begin() + range_size / 4 + 15);

        // Filter out invalid values using lambda
        auto isValid = [](float x) { return x > 0.1; };
        front_ranges.erase(std::remove_if(front_ranges.begin(), front_ranges.end(), [&](float x) { return !isValid(x); }), front_ranges.end());
        left_ranges.erase(std::remove_if(left_ranges.begin(), left_ranges.end(), [&](float x) { return !isValid(x); }), left_ranges.end());
        right_ranges.erase(std::remove_if(right_ranges.begin(), right_ranges.end(), [&](float x) { return !isValid(x); }), right_ranges.end());

        front_distance = !front_ranges.empty() ? median(front_ranges) : std::numeric_limits<float>::infinity();
        left_distance = !left_ranges.empty() ? median(left_ranges) : std::numeric_limits<float>::infinity();
        right_distance = !right_ranges.empty() ? median(right_ranges) : std::numeric_limits<float>::infinity();

        // Driving logic
        float steering_angle = 0.0;
        float speed = max_speed;

        // Calculate error
        float error = left_distance - right_distance;

        // Update integral error
        integral_error += error;
        integral_error = clamp(integral_error, -max_integral, max_integral); // Manual clamping

        // Calculate derivative error
        float derivative_error = error - last_error;
        last_error = error;

        // PID control
        steering_angle = kp * error + ki * integral_error + kd * derivative_error;

        // Adjust speed and direction based on the front distance
        if (front_distance < 2.5) {
            if (right_distance > left_distance) {
                steering_angle = -0.8;  // Turn right
            } else {
                steering_angle = 0.8;   // Turn left
            }

            // Adjust speed based on front distance
            speed = std::max(min_speed, std::min(max_speed, front_distance / 2));
            integral_error = 0;
            last_error = 0;
        }

        // Limit steering angle
        steering_angle = clamp(steering_angle, -0.8f, 0.8f); // Manual clamping

        // Publish drive message
        ackermann_msgs::AckermannDriveStamped drive_msg;
        drive_msg.header.stamp = ros::Time::now();
        drive_msg.header.frame_id = "base_link";


        if(stopVehicle) {
            if(flag) {
                speed = 0;
                flag = false; // 한 번만 실행되게 설계, RRT topic을 받기 위함
            }
        }

        drive_msg.drive.steering_angle = steering_angle;
        drive_msg.drive.speed = speed;
        drive_pub.publish(drive_msg);
    }

    void publishDistances(const ros::TimerEvent&) {
        if (lap != "1" && lap != "0") {
            //ROS_INFO("rrt is running");
            return;
        }
        ROS_INFO("Front: %.2f m, Left: %.2f m, Right: %.2f m", front_distance, left_distance, right_distance);
    }

private:
    ros::Subscriber lap_sub;
    ros::Subscriber scan_sub;
    ros::Publisher drive_pub;
    ros::Timer timer;

    float max_speed;
    float min_speed;
    float kp;
    float ki;
    float kd;
    float integral_error;
    float last_error;
    float max_integral;

    float left_distance;
    float right_distance;
    float front_distance;
    std::string lap;

    // Manual clamp function
    float clamp(float value, float min_value, float max_value) {
        return std::max(min_value, std::min(max_value, value));
    }

    float median(std::vector<float>& v) {
        size_t n = v.size() / 2;
        std::nth_element(v.begin(), v.begin() + n, v.end());
        return v[n];
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "wall_follow_node");
    WallFollow wall_follower;
    ros::spin();
    return 0;
}
