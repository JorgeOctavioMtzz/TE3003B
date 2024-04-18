#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math

class ControllerNode:
    def __init__(self):
        rospy.init_node('controller')

        # Variables para la posición deseada
        self.xd = 1.0
        self.yd = 0.0
        self.thd = 0.0

        # Constantes del controlador P
        self.Kp_linear = 0.5
        self.Kp_angular = 1.0

        # Umbral para considerar que ha llegado al punto deseado
        self.position_tolerance = 0.1

        # Subscripción al odómetro para obtener la posición actual
        rospy.Subscriber("/odom", Odometry, self.odom_callback)

        # Publicación de las velocidades de control
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    def odom_callback(self, data):
        # Obtener la posición actual del robot
        x = data.pose.pose.position.x
        y = data.pose.pose.position.y
        orientation_q = data.pose.pose.orientation
        _, _, th = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])

        # Calcular el error en la posición
        ex = self.xd - x
        ey = self.yd - y
        el = math.sqrt(ex**2 + ey**2)

        # Detener el robot si está cerca del punto deseado
        if el < self.position_tolerance:
            twist = Twist()  # Velocidades nulas
        else:
            # Calcular el ángulo hacia la posición deseada
            thd = math.atan2(ey, ex)

            # Calcular las velocidades lineal y angular
            linear_error = self.Kp_linear * el
            angular_error = self.Kp_angular * (thd - th)

            # Crear el mensaje de velocidad
            twist = Twist()
            twist.linear.x = linear_error
            twist.angular.z = angular_error

        # Publicar la velocidad de control
        self.vel_pub.publish(twist)

if __name__ == '__main__':
    node = ControllerNode()
    rospy.spin()



