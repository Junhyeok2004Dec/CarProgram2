#include <ros/ros.h>    
#include <ackermann_msgs/AckermannDriveStamped.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "straight_Left");
    ros::NodeHandle nh;

    ros::Publisher drive_pub = nh.advertise<ackermann_msgs::AckermannDriveStamped>("/drive", 10);
    ros::Rate loop_rate(10);  // 10 Hz loop rate

    ackermann_msgs::AckermannDriveStamped drive_msg;
    
    // 양수 steering_angle이 왼쪽
    drive_msg.drive.speed = 1.0;  // Set a speed of 1.0 m/s

    drive_msg.drive.steering_angle = 0.0;  // No steering, straight

    ros::Time start_time = ros::Time::now();

    while (ros::ok() && (ros::Time::now() - start_time).toSec() < 5.0) {
        drive_pub.publish(drive_msg);
        ros::spinOnce();
        loop_rate.sleep();

    }
    drive_msg.drive.steering_angle = 0.524;  //Turn right 30deg

    start_time = ros::Time::now();

    while (ros::ok() && (ros::Time::now() - start_time).toSec() < 5.0) {
        drive_pub.publish(drive_msg);
        ros::spinOnce();
        loop_rate.sleep();

    }

    // Stop the car after 5 seconds

    drive_msg.drive.speed = 0.0;
    
    start_time = ros::Time::now();

    while (ros::ok() && (ros::Time::now() - start_time).toSec() < 5.0) {
    

    drive_pub.publish(drive_msg);
    ros::spinOnce();
    loop_rate.sleep();
    }
    return 0;
}
