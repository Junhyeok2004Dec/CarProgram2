#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class WallFollow:
    """ 
    Implement Wall Following on the car
    """
    def __init__(self):
        rospy.init_node('wall_follow_node', anonymous=True)

        lidarscan_topic = '/scan'
        drive_topic = '/drive'

        # Subscribers and Publishers
        self.scan_sub = rospy.Subscriber(lidarscan_topic, LaserScan, self.scan_callback)
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=10)

        # Set proportional gain
        self.kp = 0.7  # Adjust this value as needed

        # Desired distance to the wall
        self.desired_distance = 0.85

    def get_range(self, range_data, angle):
        """
        Simple helper to return the corresponding range measurement at a given angle. Make sure you take care of NaNs and infs.

        Args:
            range_data: single range array from the LiDAR
            angle: between angle_min and angle_max of the LiDAR

        Returns:
            range: range measurement in meters at the given angle
        """
        angle_index = int((angle - range_data.angle_min) / range_data.angle_increment)
        distance = range_data.ranges[angle_index]

        if np.isinf(distance) or np.isnan(distance):
            return float('inf')
        else:
            return distance

    def get_error(self, range_data, dist):
        """
        Calculates the error to the wall. Follow the wall to the left (going counter clockwise in the Levine loop).

        Args:
            range_data: single range array from the LiDAR
            dist: desired distance to the wall

        Returns:
            error: calculated error
        """
        left_angle = np.pi / 2  # 90 degrees to the left
        left_distance = self.get_range(range_data, left_angle)
        error = dist - left_distance
        return error

    def pid_control(self, error, velocity):
        """
        Based on the calculated error, publish vehicle control

        Args:
            error: calculated error
            velocity: desired velocity

        Returns:
            None
        """
        angle = self.kp * -error  # Positive error means turn left
        angle = max(-0.4, min(0.4, angle))  # Limit steering angle

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "base_link"
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = velocity

        self.drive_pub.publish(drive_msg)

    def scan_callback(self, msg):
        """
        Callback function for LaserScan messages. Calculate the error and publish the drive message in this function.

        Args:
            msg: Incoming LaserScan message

        Returns:
            None
        """
        error = self.get_error(msg, self.desired_distance)
        velocity = 0.6  # Constant velocity

        # Log the left distance
        rospy.loginfo(f"Error: {error:.2f}")

        self.pid_control(error, velocity)


def main():
    rospy.init_node('wall_follow_node', anonymous=True)
    print("WallFollow Initialized")
    wall_follow_node = WallFollow()
    rospy.spin()

    # Destroy the node explicitly
    wall_follow_node.drive_pub.unregister()
    rospy.signal_shutdown('Shutting down')


if __name__ == '__main__':
    main()
