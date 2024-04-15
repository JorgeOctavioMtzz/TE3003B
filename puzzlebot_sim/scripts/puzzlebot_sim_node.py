#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Float32

class PuzzlebotSimulator:
    def __init__(self):
        rospy.init_node('puzzlebot_sim_node')

        self.wheel_radius = 0.05  # 5 cm in meters
        self.wheelbase = 0.19  # 19 cm in meters

        self.pose_pub = rospy.Publisher('/pose', PoseStamped, queue_size=10)
        self.wr_pub = rospy.Publisher('/wr', Float32, queue_size=10)
        self.wl_pub = rospy.Publisher('/wl', Float32, queue_size=10)

        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)

        self.pose = PoseStamped()
        self.wr = Float32()
        self.wl = Float32()

        self.last_time = rospy.Time.now()

    def cmd_vel_callback(self, msg):
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()
        wz = msg.angular.z
        vx = msg.linear.x 
        vy = msg.linear.y
        

        wr = ((2 * (vx+vy) + wz * self.wheelbase) / (2 * self.wheel_radius))
        wl = ((2 * (vx+vy) - wz * self.wheelbase) / (2 * self.wheel_radius))

        self.pose.pose.position.x += vx * dt
        self.pose.pose.position.y += vy * dt
        self.pose.pose.orientation.z += wz * dt

        self.wr.data = wr
        self.wl.data = wl

        self.pose_pub.publish(self.pose)
        self.wr_pub.publish(self.wr)
        self.wl_pub.publish(self.wl)

        self.last_time = current_time

if __name__ == '__main__':
    try:
        puzzlebot_simulator = PuzzlebotSimulator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

