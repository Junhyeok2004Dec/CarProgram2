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
        
        self.max_speed = 3.0
        self.min_speed = 0.05
        self.kp = 0.5  # 비례 제어 상수
        self.ki = 0.05  # 적분 제어 상수
        self.kd = 0.5  # 미분 제어 상수
        self.integral_error = 0  # 적분 오차 누적값
        self.last_error = 0  # 이전 오차 (미분 제어용)
        self.max_integral = 1.0  # 적분 항 최대값 (와인드업 방지)

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

        self.front_distance = np.median(front_valid) if len(front_valid) > 0 else float('inf')
        self.left_distance = np.median(left_valid) if len(left_valid) > 0 else float('inf')
        self.right_distance = np.median(right_valid) if len(right_valid) > 0 else float('inf')

        # 주행 로직
        steering_angle = 0  # 기본적으로 직진
        speed = self.max_speed  # 기본 속도는 최대 속도

        # 좌우 거리 차이를 이용한 조향 제어
        error = self.left_distance - self.right_distance
        
        # 적분 오차 업데이트
        self.integral_error += error
        # 와인드업 방지
        self.integral_error = max(-self.max_integral, min(self.max_integral, self.integral_error))

        # 미분 오차 계산
        derivative_error = error - self.last_error
        self.last_error = error

        # PID 제어
        steering_angle = (self.kp * error + 
                          self.ki * self.integral_error + 
                          self.kd * derivative_error)

        # 전방 거리가 2m 미만일 때 속도 조절 및 회피
        if self.front_distance < 1.9:
            if self.right_distance > self.left_distance:
                steering_angle = -0.6  # 오른쪽으로 회전
            else:
                steering_angle = 0.6  # 왼쪽으로 회전
            
            # 전방 거리에 따른 속도 조절
            speed = max(self.min_speed, min(self.max_speed, self.front_distance / 2))
            
            # 장애물 회피 시 적분 오차 리셋
            self.integral_error = 0
            self.last_error = 0
        
        # 조향각 제한
        steering_angle = max(-0.8, min(0.8, steering_angle))
        
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
