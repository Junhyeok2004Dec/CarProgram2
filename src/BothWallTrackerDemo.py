#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class WallFollow:
    def __init__(self):
        rospy.init_node('wall_follow_node', anonymous=True)
        
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.drive_pub = rospy.Publisher('/drive', AckermannDriveStamped, queue_size=10)
        
        self.desired_distance = 1.0  # 벽과의 목표 거리 (미터)
        self.max_speed = 1.0
        self.min_speed = 0.1

        # 거리 정보를 저장할 변수들
        self.left_distance = 0
        self.right_distance = 0
        self.front_distance = 0

        # 주기적으로 거리 정보를 출력하는 타이머
        rospy.Timer(rospy.Duration(0.1), self.publish_distances)  # 10Hz로 거리 정보 출력

    def scan_callback(self, scan_msg):
        ranges = np.array(scan_msg.ranges)
        
        # 270도 라이다 기준 (0도가 정면, 90도가 왼쪽, -90도가 오른쪽)
        front_indices = slice(len(ranges) // 2 - 15, len(ranges) // 2 + 15)  # 정면 30도 범위
        left_indices = slice(len(ranges) * 3 // 4 - 15, len(ranges) * 3 // 4 + 15)  # 왼쪽 30도 범위
        right_indices = slice(len(ranges) // 4 - 15, len(ranges) // 4 + 15)  # 오른쪽 30도 범위

        front_valid = ranges[front_indices][ranges[front_indices] > 0.1]
        left_valid = ranges[left_indices][ranges[left_indices] > 0.1]
        right_valid = ranges[right_indices][ranges[right_indices] > 0.1]

        self.front_distance = np.mean(front_valid) if len(front_valid) > 0 else float('inf')
        self.left_distance = np.mean(left_valid) if len(left_valid) > 0 else float('inf')
        self.right_distance = np.mean(right_valid) if len(right_valid) > 0 else float('inf')

        # 기본 주행 로직
        error = self.desired_distance - self.left_distance
        steering_angle = -error  # 음수 값을 사용하여 왼쪽으로 조향
        steering_angle = max(-0.4, min(0.4, steering_angle))  # 조향각 제한
        
        # 속도 계산
        speed = self.max_speed
        if self.front_distance < 1.1:
            speed = self.min_speed
        elif self.front_distance < 2.1:
            speed = self.min_speed + (self.max_speed - self.min_speed) * (self.front_distance - 1.0)
        
        # 주행 명령 발행
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "base_link"
        drive_msg.drive.steering_angle = steering_angle
        drive_msg.drive.speed = speed
        
        self.drive_pub.publish(drive_msg)

    def publish_distances(self, event):
        rospy.loginfo(f"Front: {self.front_distance:.2f}m, Left: {self.left_distance:.2f}m, Right: {self.right_distance:.2f}m")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        wall_follower = WallFollow()
        wall_follower.run()
    except rospy.ROSInterruptException:
        pass