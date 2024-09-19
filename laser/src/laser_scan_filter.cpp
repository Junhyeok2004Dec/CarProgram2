#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <cmath>
#include <limits>

class LaserScanFilter
{
public:
    LaserScanFilter() : nh_("~"), accumulated_distance_(0.0)
    {
        nh_.param("angle_min", user_angle_min_, -M_PI/18.0);
        nh_.param("angle_max", user_angle_max_, M_PI/18.0);
        nh_.param("left_wall_angle", left_wall_angle_, -M_PI/4.0);  // 왼쪽 벽 각도 (기본값 -45도)
        nh_.param("right_wall_angle", right_wall_angle_, M_PI/4.0);  // 오른쪽 벽 각도 (기본값 45도)
        nh_.param("distance_threshold", distance_threshold_, 10.0);  // 누적 거리 임계값 (미터)

        scan_sub_ = nh_.subscribe("/scan", 1, &LaserScanFilter::laserCallback, this);
        scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>("/filtered_scan", 1);
        distance_pub_ = nh_.advertise<std_msgs::Float32>("/closest_obstacle", 1);
        wall_distance_pub_ = nh_.advertise<std_msgs::Float32>("/wall_distance", 1);
        threshold_reached_pub_ = nh_.advertise<std_msgs::Bool>("/distance_threshold_reached", 1);
    }

    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
    {
        sensor_msgs::LaserScan filtered_scan = *scan;

        filtered_scan.angle_min = std::max(static_cast<float>(user_angle_min_), scan->angle_min);
        filtered_scan.angle_max = std::min(static_cast<float>(user_angle_max_), scan->angle_max);

        int min_index = static_cast<int>((filtered_scan.angle_min - scan->angle_min) / scan->angle_increment);
        int max_index = static_cast<int>((filtered_scan.angle_max - scan->angle_min) / scan->angle_increment);

        filtered_scan.ranges.assign(scan->ranges.begin() + min_index, scan->ranges.begin() + max_index + 1);
        if (!scan->intensities.empty()) {
            filtered_scan.intensities.assign(scan->intensities.begin() + min_index, scan->intensities.begin() + max_index + 1);
        }

        double min_distance = std::numeric_limits<double>::max();
        for (const auto& range : filtered_scan.ranges) {
            if (range > scan->range_min && range < scan->range_max && range < min_distance) {
                min_distance = range;
            }
        }

        // 왼쪽과 오른쪽 벽 거리 계산
        int left_index = static_cast<int>((left_wall_angle_ - scan->angle_min) / scan->angle_increment);
        int right_index = static_cast<int>((right_wall_angle_ - scan->angle_min) / scan->angle_increment);
        
        double left_wall_distance = scan->ranges[left_index];
        double right_wall_distance = scan->ranges[right_index];

        // 벽 사이의 거리 계산
        double wall_distance = left_wall_distance + right_wall_distance;

        // 누적 거리 계산 (이동 거리를 추정하기 위해 간단한 방법 사용)
        static ros::Time last_time = ros::Time::now();
        ros::Time current_time = ros::Time::now();
        double dt = (current_time - last_time).toSec();
        accumulated_distance_ += wall_distance * dt;  // 간단한 추정, 실제로는 더 정확한 방법이 필요할 수 있습니다.
        last_time = current_time;

        // 결과 발행
        scan_pub_.publish(filtered_scan);

        std_msgs::Float32 distance_msg;
        distance_msg.data = min_distance;
        distance_pub_.publish(distance_msg);

        std_msgs::Float32 wall_distance_msg;
        wall_distance_msg.data = wall_distance;
        wall_distance_pub_.publish(wall_distance_msg);

        // 누적 거리가 임계값에 도달했는지 확인
        std_msgs::Bool threshold_reached_msg;
        threshold_reached_msg.data = (accumulated_distance_ >= distance_threshold_);
        threshold_reached_pub_.publish(threshold_reached_msg);

        ROS_INFO("Closest obstacle: %.2f m, Wall distance: %.2f m, Accumulated distance: %.2f m", 
                 min_distance, wall_distance, accumulated_distance_);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber scan_sub_;
    ros::Publisher scan_pub_;
    ros::Publisher distance_pub_;
    ros::Publisher wall_distance_pub_;
    ros::Publisher threshold_reached_pub_;
    double user_angle_min_;
    double user_angle_max_;
    double left_wall_angle_;
    double right_wall_angle_;
    double distance_threshold_;
    double accumulated_distance_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "laser_scan_filter");
    LaserScanFilter laser_filter;
    ros::spin();
    return 0;
}

