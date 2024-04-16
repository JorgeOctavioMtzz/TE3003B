#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseStamped
import math

class LocalisationNode:
    def __init__(self):
        rospy.init_node('localisation')

        self.wheel_radius = 0.05  # Wheel radius in meters
        self.wheelbase = 0.19  # Distance between the wheels in meters

        # Subscribing to wheel encoder topics
        rospy.Subscriber("/wr", Float32, self.wr_callback)
        rospy.Subscriber("/wl", Float32, self.wl_callback)
        rospy.Subscriber("/pose", PoseStamped, self.pose_callback)

        # Publisher for odometry
        self.odom_pub = rospy.Publisher("/odom", Odometry, queue_size=50)

        # Initial position and orientation
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        # Time tracking for velocity calculations
        self.last_time = rospy.Time.now()

        # Last wheel positions (initially zero)
        self.last_wr = 0.0
        self.last_wl = 0.0

    def wr_callback(self, data):
        self.last_wr = data.data
        self.update_odometry()

    def wl_callback(self, data):
        self.last_wl = data.data
        self.update_odometry()

    def pose_callback(self, data):
        self.x = data.pose.position.x
        self.y = data.pose.position.y

        # Calculate orientation from quaternion
        orientation_q = data.pose.orientation
        self.th = math.atan2(2*(orientation_q.w*orientation_q.z + orientation_q.x*orientation_q.y),
                             1 - 2*(orientation_q.y**2 + orientation_q.z**2))

        self.update_odometry()

    def update_odometry(self):
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()
        if dt == 0:
            return

        # Calculate the wheel movements since the last update
        delta_wr = (self.last_wr - self.wheelbase/2) * self.wheel_radius
        delta_wl = (self.last_wl - self.wheelbase/2) * self.wheel_radius

        # Calculate average movement and change in orientation
        delta_distance = (delta_wr + delta_wl) / 2
        delta_th = (delta_wr - delta_wl) / self.wheelbase
        self.th += delta_th

        # Update the position
        delta_x = delta_distance * math.cos(self.th)
        delta_y = delta_distance * math.sin(self.th)
        self.x += delta_x
        self.y += delta_y

        # Create and publish the odometry message
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = Quaternion(0, 0, math.sin(self.th/2), math.cos(self.th/2))
        odom.child_frame_id = "base_link"
        odom.twist.twist.linear.x = delta_distance / dt
        odom.twist.twist.angular.z = delta_th / dt

        self.odom_pub.publish(odom)

        # Update the last_time for next calculation
        self.last_time = current_time

if __name__ == '__main__':
    node = LocalisationNode()
    rate = rospy.Rate(10)  # 10Hz
    while not rospy.is_shutdown():
        rate.sleep()

