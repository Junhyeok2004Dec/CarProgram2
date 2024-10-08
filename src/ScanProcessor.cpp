#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h>
#include <vector>
#include <algorithm>
#include <cmath> // For std::abs function

class ScanProcessor {
public:
    ScanProcessor() {
        ros::NodeHandle nh;

        scan_sub_ = nh.subscribe("/scan", 10, &ScanProcessor::scanCallback, this);

        distance_pub_ = nh.advertise<std_msgs::Float32>("/scan_demo_distance", 10);
        angle_pub_ = nh.advertise<std_msgs::Float32>("/scan_demo_angle", 10);

    }

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg) {
        std::vector<float> valid_ranges;
        std::vector<float> angles;

        // 유효한 범위 데이터 및 해당 각도 추출
        for (size_t i = 0; i < scan_msg->ranges.size(); ++i) {
            float range = scan_msg->ranges[i];
            if (range >= scan_msg->range_min && range <= scan_msg->range_max) {
                valid_ranges.push_back(range);
                angles.push_back(scan_msg->angle_min + i * scan_msg->angle_increment);
            }
        }

        // 벽이 인식되지 않는 가장 큰 구간 찾기 (예: 최대 거리가 가장 큰 부분 찾기)
        auto max_it = std::max_element(valid_ranges.begin(), valid_ranges.end());

        // 최대 거리가 있을 경우, 이를 /scan_demo_distance 및 /scan_demo_angle 토픽에 퍼블리시
        if (max_it != valid_ranges.end()) {
            size_t index = std::distance(valid_ranges.begin(), max_it);
            float max_distance = *max_it;
            float max_angle = angles[index];

            // 최대 거리 퍼블리시
            std_msgs::Float32 distance_msg;
            distance_msg.data = max_distance;
            distance_pub_.publish(distance_msg);

            // 최대 거리의 각도 퍼블리시
            std_msgs::Float32 angle_msg;
            angle_msg.data = max_angle;
            angle_pub_.publish(angle_msg);
        }
    }

private:
    ros::Subscriber scan_sub_;
    ros::Publisher distance_pub_;
    ros::Publisher angle_pub_;
};

int main(int argc, char** argv) {
    // ROS 노드 초기화
    ros::init(argc, argv, "scan_processor");

    // ScanProcessor 클래스 인스턴스 생성
    ScanProcessor scan_processor;

    // ROS 스핀
    ros::spin();

    return 0;
}
