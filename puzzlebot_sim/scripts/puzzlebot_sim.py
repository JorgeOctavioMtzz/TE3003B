#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Float32
import math

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
	#dt = 1/100
        if dt == 0:
            return

        vx = msg.linear.x
        wz = msg.angular.z

        # Calculate the angular velocities for each wheel
        vr = vx + (self.wheelbase / 2) * wz
        vl = vx - (self.wheelbase / 2) * wz

        wr = vr / self.wheel_radius
        wl = vl / self.wheel_radius

        # Update robot pose
        # Update orientation to integrate around z-axis
        theta = self.pose.pose.orientation.z + wz * dt
        self.pose.pose.orientation.z = theta

        # Update position with respect to the orientation
        self.pose.pose.position.x += vx * math.cos(theta) * dt
        self.pose.pose.position.y += vx * math.sin(theta) * dt

        self.wr.data = wr
        self.wl.data = wl

        # Publish new states
        self.pose_pub.publish(self.pose)
        self.wr_pub.publish(self.wr)
        self.wl_pub.publish(self.wl)

        self.last_time = current_time

if __name__ == '__main__':
    try:
        puzzlebot_simulator = PuzzlebotSimulator()
        rate = rospy.Rate(100)  # 100 Hz
        while not rospy.is_shutdown():
            rospy.spin()  # Process incoming messages
            rate.sleep()       # leep to maintain the loop rate at 100 Hz
    except rospy.ROSInterruptException:
        pass
