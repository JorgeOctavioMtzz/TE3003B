#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
import tf2_ros

def odometry_callback(msg):
    # Create a JointState message
    joint_state = JointState()
    joint_state.header = msg.header  # Use the same timestamp and frame
    joint_state.name = ['wheel_coupler_joint', 'wheel_coupler_joint_2']
    # These values would ideally be calculated based on the robot's specific kinematics
    joint_state.position = [0, 0]  # Placeholder values
    joint_state.velocity = [msg.twist.twist.linear.x, msg.twist.twist.angular.z]
    joint_state.effort = []

    # Publish joint states
    pub.publish(joint_state)

    # Publish transform using tf2 (from 'odom' to 'base_link')
    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "odom"
    t.child_frame_id = "base_link"
    t.transform.translation.x = msg.pose.pose.position.x
    t.transform.translation.y = msg.pose.pose.position.y
    t.transform.translation.z = 0.0
    t.transform.rotation = msg.pose.pose.orientation
    
    br.sendTransform(t)
    

if __name__ == '__main__':
    rospy.init_node('joint_publisher')

    # Publisher for JointState
    pub = rospy.Publisher('joint_states', JointState, queue_size=10)
    
    # Transform broadcaster
    br = tf2_ros.TransformBroadcaster()

    # Subscriber to the odometry topic
    rospy.Subscriber("odom", Odometry, odometry_callback)
  

    rospy.spin()

   