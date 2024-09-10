#include <ros/ros.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <cmath>

ros::Publisher drive_pub;
const float DESIRED_DISTANCE_LEFT = 0.4;  // 왼쪽 벽과의 이상적인 거리 (m)
const float Kp = 1.0;  // P 제어 계수
const float L = 0.3;  // 차량의 전장 (축간 거리, m)

// 레이저 스캔 데이터를 처리하는 콜백 함수
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
    // 레이저 스캔의 각도 범위 및 인덱스 계산
    int ranges_size = scan->ranges.size();
    float angle_min = scan->angle_min;  // 최소 각도 (rad)
    float angle_max = scan->angle_max;  // 최대 각도 (rad)
    float angle_increment = scan->angle_increment;  // 각도 해상도 (rad)

    // 왼쪽 90도에 해당하는 각도를 찾아서 인덱스로 변환 (왼쪽 90도는 π/2 rad)
    float left_angle = M_PI / 2;  // 왼쪽 90도 (rad)
    int left_idx = (int)((left_angle - angle_min) / angle_increment);

    // 인덱스가 유효한지 확인
    if (left_idx >= 0 && left_idx < ranges_size) {
        float left_distance = scan->ranges[left_idx];  // 왼쪽 거리를 가져옴

        // 레이저 데이터가 유효한지 확인 (무한대 값 등 처리)
        if (std::isfinite(left_distance)) {

            ackermann_msgs::AckermannDriveStamped drive_msg;

            // 벽과의 거리를 유지하기 위해 조향 각도를 조절
            float error = DESIRED_DISTANCE_LEFT - left_distance;  // 벽과의 거리 차이 (오차)
            
            // 차량이 회전해야 할 조향 각도를 계산 (atan2를 사용하여 각도를 계산)
            float steering_angle = Kp * atan2(error, L);  // atan2(error, L)로 각도 계산

            drive_msg.drive.speed = 1.0;  // 기본 주행 속도
            drive_msg.drive.steering_angle = steering_angle;  // 벽을 따라가기 위한 조향


            ROS_INFO("%f",drive_msg.drive.steering_angle);
            drive_pub.publish(drive_msg);
        } else {
            ROS_WARN("Invalid left distance detected.");
        }
    } else {

    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "wall_following_drive");
    ros::NodeHandle nh;

    // /drive 토픽 퍼블리셔
    drive_pub = nh.advertise<ackermann_msgs::AckermannDriveStamped>("/drive", 10);

    // /scan 토픽 서브스크라이버
    ros::Subscriber scan_sub = nh.subscribe("/scan", 10, scanCallback);

    // 주기적으로 콜백 함수가 호출되도록 루프를 설정 (ros::spin 사용)
    ros::spin();  // ROS 이벤트 처리 루프 시작, 콜백 함수가 호출됨

    return 0;
}
