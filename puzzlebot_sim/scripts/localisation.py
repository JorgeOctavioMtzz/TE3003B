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
        # rospy.Subscriber("/pose", PoseStamped, self.pose_callback)

        # Publisher for odometry
        self.odom_pub = rospy.Publisher("/odom", Odometry, queue_size=50)

        # Initial position and orientation
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        # Time tracking for velocity calculations
        self.last_time = rospy.Time.now()

        # Last wheel encoder readings (initially zero)
        self.last_wr = 0.0
        self.last_wl = 0.0

    def wr_callback(self, data):
        self.last_wr = data.data
        self.update_odometry()

    def wl_callback(self, data):
        self.last_wl = data.data
        self.update_odometry()

    ''' def pose_callback(self, data):
        self.x = data.pose.position.x
        self.y = data.pose.position.y
        orientation_q = data.pose.orientation
        self.th = math.atan2(2*(orientation_q.w*orientation_q.z + orientation_q.x*orientation_q.y),
                             1 - 2*(orientation_q.y**2 + orientation_q.z**2))

        self.update_odometry()
    '''
    def update_odometry(self):
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()
	#dt = 1/100
        if dt == 0:
            return

        # Calculate the wheel movements since the last update
        delta_wr = self.last_wr * self.wheel_radius #* dt
        delta_wl = self.last_wl * self.wheel_radius #* dt

        # Calculate velocities
        v = (delta_wr + delta_wl) / 2
        omega = (delta_wr - delta_wl) / self.wheelbase

        # Update the pose
        delta_th=omega*dt
        self.th += delta_th
        delta_x = v * dt
        delta_y= 0
        self.x += delta_x * math.cos(self.th)
        self.y += delta_x * math.sin(self.th)
        '''delta_x = v * math.cos(self.th) * dt
        delta_y = v * math.sin(self.th) * dt
        self.x += delta_x
        self.y += delta_y
        self.th += omega * dt'''

        # Create and publish the odometry message
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = Quaternion(*self.quaternion_from_euler(0, 0, self.th))
        odom.child_frame_id = "base_link"
        odom.twist.twist.linear.x = v * math.cos(self.th)
        odom.twist.twist.linear.y = v * math.sin(self.th)
        odom.twist.twist.angular.z = omega

        self.odom_pub.publish(odom)

        # Update the last_time for next calculation
        self.last_time = current_time

    def quaternion_from_euler(self, roll, pitch, yaw):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)

        q = Quaternion()
        q.w = cy * cr * cp + sy * sr * sp
        q.x = cy * sr * cp - sy * cr * sp
        q.y = cy * cr * sp + sy * sr * cp
        q.z = sy * cr * cp - cy * sr * sp

        return (q.x, q.y, q.z, q.w)
    
if __name__ == '__main__':
    node = LocalisationNode()
    rospy.spin()  # Use rospy.spin() to keep the node running
