#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <visualization_msgs/Marker.h>
#include <cmath>

// 전역 변수로 거리와 각도를 저장
float current_distance = 0.0;
float current_angle = 0.0;

ros::Subscriber distance_sub;  // 거리 topic
ros::Subscriber angle_sub;     // 각도(라이다 각도, 일반각-라디안) topic


ros::Publisher marker_pub;  // 화살표로 표시 (RVIZ)

void distanceCallback(const std_msgs::Float32::ConstPtr& msg)
{
    current_distance = msg->data;
}

void angleCallback(const std_msgs::Float32::ConstPtr& msg)
{
    current_angle = msg->data;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "/marker_publisher");
    ros::NodeHandle nh;

    marker_pub = nh.advertise<visualization_msgs::Marker>("/visualization_marker", 1);

    distance_sub = nh.subscribe("/scan_demo_distance", 10, distanceCallback);
    angle_sub = nh.subscribe("/scan_demo_angle", 10, angleCallback);

    ros::Rate r(10); // 주기: 10Hz

    while (ros::ok())
    {
        // 마커 메시지 생성
        visualization_msgs::Marker marker;
        marker.header.frame_id = "/base_link";  // RViz의 기준 프레임을 설정
        marker.header.stamp = ros::Time::now();

        marker.ns = "/scan_demo";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;

        // 화살표의 크기 설정
        marker.scale.x = current_distance;  // 화살표의 길이 (거리 값으로 설정)
        marker.scale.y = 0.1;               // 화살표의 폭
        marker.scale.z = 0.1;               // 화살표의 높이

        // 화살표의 색상 설정
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;

        // 화살표의 위치와 방향 설정
        marker.pose.position.x = 0.0;
        marker.pose.position.y = 0.0;
        marker.pose.position.z = 0.0;

        // 각도를 기반으로 오리엔테이션(방위)을 설정
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = sin(current_angle / 2.0);
        marker.pose.orientation.w = cos(current_angle / 2.0);

        // 마커 퍼블리시
        marker_pub.publish(marker);

        ros::spinOnce();
        r.sleep();
    }
    return 0;
}
