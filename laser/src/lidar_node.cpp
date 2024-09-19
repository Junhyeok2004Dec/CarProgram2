#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_broadcaster.h>

class SimulatorLidarNode {
public:
    SimulatorLidarNode() : nh_("~") {
        scan_sub_ = nh_.subscribe("/scan", 1, &SimulatorLidarNode::scanCallback, this);
        ROS_INFO("Simulator Lidar Init");
    }

private:
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg) {
        ROS_INFO("Received laser scan with %zu ranges", scan_msg->ranges.size());
        
        // 여기에서 라이더 데이터를 처리하거나 시각화할 수 있습니다.
        // 예를 들어, 첫 번째와 마지막 유효한 거리 측정값을 출력해봅시다.
        float first_valid = -1, last_valid = -1;
        for (size_t i = 0; i < scan_msg->ranges.size(); ++i) {
            if (scan_msg->ranges[i] >= scan_msg->range_min && scan_msg->ranges[i] <= scan_msg->range_max) {
                if (first_valid < 0) first_valid = scan_msg->ranges[i];
                last_valid = scan_msg->ranges[i];
            }
        }
        ROS_INFO("First valid range: %.2f, Last valid range: %.2f", first_valid, last_valid);

        // TF 브로드캐스트 (선택사항)
        static tf::TransformBroadcaster br;
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
        transform.setRotation(tf::Quaternion(0, 0, 0, 1));
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "laser"));
    }

    ros::NodeHandle nh_;
    ros::Subscriber scan_sub_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "simulator_lidar_node");
    SimulatorLidarNode node;
    ros::spin();
    return 0;
}

