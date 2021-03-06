#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
import numpy as np
import time

class TurtleControl:
    def __init__(self):
        rospy.init_node("tb3control_node", anonymous=True)
        self.vel_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        rospy.Subscriber("/odom", Odometry, self.update_pose)
        rospy.Subscriber("/scan", LaserScan, self.update_scan)
        self.pose = Pose()
        self.rate = rospy.Rate(10)
        self.max_vel = 0.22
        self.max_ang = 2.84
    
    def update_pose(self, msg):
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, yaw) = euler_from_quaternion(orientation_list)
        self.pose.x = msg.pose.pose.position.x
        self.pose.y = msg.pose.pose.position.y
        self.pose.theta =  yaw

    def update_scan(self, msg):
        self.scan = msg
    
    def ref_distance(self, ref_pose):
        return np.sqrt(  (ref_pose.x - self.pose.x)**2 + (ref_pose.y - self.pose.y)**2)

    def linear_vel_control(self, ref_pose, kp = 1.5):
        distance = self.ref_distance(ref_pose)
        control = kp* distance
        if abs(control) > self.max_vel:
            control = self.max_vel*np.sign(control)
        return control

    def angular_vel_control(self, ref_pose, kp=6):
        angle_r = np.arctan2(ref_pose.y - self.pose.y,  ref_pose.x - self.pose.x )        
        control = kp*(angle_r - self.pose.theta)
        if abs(control) > self.max_ang:
            control = self.max_ang*np.sign(control)
        return control

    def move2ref(self, x_ref, y_ref):
        ref_pose = Pose()
        ref_pose.x = x_ref
        ref_pose.y = y_ref
        ref_tol = 0.01
        vel_msg = Twist()
        corrigir_trajetoria = False
        x_ref_desvio = x_ref
        y_ref_desvio = y_ref
        while self.ref_distance(ref_pose) >= ref_tol:
            vel_msg.linear.x = self.linear_vel_control(ref_pose)
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = self.angular_vel_control(ref_pose)

            while self.scan.ranges[0] < 0.5 or self.scan.ranges[10] < 0.5 or self.scan.ranges[-10] < 0.5:
                corrigir_trajetoria = True
                print('obstaculo detectado a frente ', self.scan.ranges[0]) #determina uma nova referencia de desvio
                if self.scan.ranges[5] > self.scan.ranges[-5]: #esquerda livre
                    print('virando a esquerda, anterior: ', vel_msg.angular.z)
                    x_ref_desvio -= 1
                    y_ref_desvio += 1
                    ref_pose.x = x_ref_desvio
                    ref_pose.y = y_ref_desvio
                else: #direita livre
                    print('virando a direita, anterior: ', vel_msg.angular.z)
                    x_ref_desvio += 1
                    y_ref_desvio -= 1
                    ref_pose.x = x_ref_desvio
                    ref_pose.y = y_ref_desvio

                vel_msg.linear.x = self.linear_vel_control(ref_pose)
                vel_msg.linear.y = 0
                vel_msg.linear.z = 0
                vel_msg.angular.x = 0
                vel_msg.angular.y = 0
                vel_msg.angular.z = self.angular_vel_control(ref_pose)
                
                if (self.scan.ranges[0] < 0.4 or \
                    self.scan.ranges[-10] < 0.4 or \
                    self.scan.ranges[10] < 0.4 \
                   ) and self.scan.ranges[180] > 1: # vai pra tras
                    print('re')
                    vel_msg.linear.x = -2
                
                self.vel_publisher.publish(vel_msg)
                self.rate.sleep()

            if corrigir_trajetoria == True: #retoma a trajetoria desejada pelo usuario
                corrigir_trajetoria = False
                ref_pose.x = x_ref
                ref_pose.y = y_ref

            self.vel_publisher.publish(vel_msg)

            self.rate.sleep()

        # stop
        vel_msg.linear.x = 0
        vel_msg.angular.z= 0
        self.vel_publisher.publish(vel_msg)

        rospy.loginfo("Finished")

if __name__ == '__main__':
    bot = TurtleControl()
    time.sleep(5)
    rospy.loginfo("Insira o valor de x:")
    x = int(input())
    rospy.loginfo("Insira o valor de y:")
    y = int(input())
    bot.move2ref(x,y)



