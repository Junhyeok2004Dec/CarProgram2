import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np

class FollowGap:
    def __init__(self):
        rospy.init_node('follow_gap', anonymous=True)
        
        self.lidar_sub = rospy.Subscriber('/scan', LaserScan, self.lidar_callback)
        self.drive_pub = rospy.Publisher('/drive', AckermannDriveStamped, queue_size=10)

        self.bubble_radius = 10  # 안전 버블 반경
        self.max_speed = 1.0  # 최대 속도
        self.max_steering_angle = 0.5  # 최대 조향 각도
        self.steering_sensitivity = 0.8  # 조향 민감도 조절
        self.kp_steering = 1.0  # 조향 P 제어 게인

    def preprocess_lidar(self, ranges):
        proc_ranges = np.array(ranges)
        proc_ranges = np.convolve(proc_ranges, np.ones(7)/7, mode='same')  # 이동 평균
        proc_ranges = np.clip(proc_ranges, 0, 4)  # 거리 클리핑
        return proc_ranges

    def find_closest_point(self, ranges):
        return np.argmin(ranges)

    def find_max_gap(self, free_space_ranges):
        gaps = np.where(free_space_ranges > 0)[0]
        if len(gaps) == 0:
            return 0, len(free_space_ranges) - 1
        
        gaps_diff = np.diff(gaps)
        max_gap_start = gaps[0]
        max_gap_size = 0
        current_gap_start = gaps[0]

        for i in range(1, len(gaps_diff)):
            if gaps_diff[i] > 1:
                current_gap_size = gaps[i] - current_gap_start
                if current_gap_size > max_gap_size:
                    max_gap_size = current_gap_size
                    max_gap_start = current_gap_start
                current_gap_start = gaps[i]

        current_gap_size = gaps[-1] - current_gap_start
        if current_gap_size > max_gap_size:
            max_gap_size = current_gap_size
            max_gap_start = current_gap_start

        return max_gap_start, max_gap_start + max_gap_size

    def find_best_point(self, start_idx, end_idx, ranges):
        return start_idx + np.argmax(ranges[start_idx:end_idx])

    def lidar_callback(self, data):
        total_angles = len(data.ranges)
        left_idx = total_angles // 3  # -90도
        right_idx = 2 * total_angles // 3  # 90도

        # -90도에서 90도 사이의 데이터만 사용
        proc_ranges = self.preprocess_lidar(data.ranges[left_idx:right_idx])
        print(f"Processed ranges: {proc_ranges}")

        # 가장 가까운 점 찾기
        closest_point = self.find_closest_point(proc_ranges)
        print(f"Closest point index: {closest_point}")

        # 안전 버블 적용
        start_bubble = max(0, closest_point - self.bubble_radius)
        end_bubble = min(len(proc_ranges) - 1, closest_point + self.bubble_radius)
        proc_ranges[start_bubble:end_bubble] = 0
        print(f"Ranges after applying bubble: {proc_ranges}")

        # 최대 간격 찾기
        start_idx, end_idx = self.find_max_gap(proc_ranges)
        print(f"Max gap: start {start_idx}, end {end_idx}")

        # 최적의 포인트 찾기
        best_point = self.find_best_point(start_idx, end_idx, proc_ranges)
        print(f"Best point index: {best_point}")

        # 목표 조향 각도 계산
        target_steering_angle = (best_point - len(proc_ranges) / 2) * (self.max_steering_angle / (len(proc_ranges) / 2))
        
        # P 제어를 통한 실제 조향 각도 계산
        steering_angle = self.kp_steering * target_steering_angle * self.steering_sensitivity
        steering_angle = np.clip(steering_angle, -self.max_steering_angle, self.max_steering_angle)
        
        # 속도 계산
        speed = self.max_speed * (1 - abs(steering_angle) / self.max_steering_angle)
        print(f"Steering angle: {steering_angle}, Speed: {speed}")

        # 드라이브 명령 발행
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = steering_angle
        drive_msg.drive.speed = speed
        self.drive_pub.publish(drive_msg)

if __name__ == '__main__':
    try:
        FollowGap()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
