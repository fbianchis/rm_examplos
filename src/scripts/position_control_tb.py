#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import numpy as np


class TurtleBotControl:
    def __init__(self):
       
        rospy.init_node('turtlebotcontrol_node', anonymous=True) 

        self.vel_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        self.odom_subscriber = rospy.Subscriber("/odom", Odometry, self.get_odom)

        self.posicao = Odometry()

        self.rate = rospy.Rate(10)

        self.roll = 0.0
        self.pith = 0.0
        self.yaw = 0.0

    def get_odom(self, msg):
        
        self.posicao = msg
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        self.roll, self.pith, self.yaw = euler_from_quaternion(orientation_list)
        #rospy.loginfo("X->"+str(self.position.x)+"; Y->"+str(self.position.y)+"; Z->"+str(self.position.z))
        #rospy.loginfo("R->"+str(roll)+"; P->"+str(pith)+"; Y->"+str(yaw))
    
    def ref_distance(self, ref_pose):
    
        return np.sqrt(((ref_pose.pose.pose.position.x - self.posicao.pose.pose.position.x)**2) + ((ref_pose.pose.pose.position.y - self.posicao.pose.pose.position.y)**2))

    def linear_vel_control(self, ref_pose, kp = 1.5):

        distance = self.ref_distance(ref_pose)

        return kp * distance
    
    def angular_vel_control(self, ref_pose, kp = 6):

        angle_r = np.arctan2(ref_pose.pose.pose.position.y - self.posicao.pose.pose.position.y, ref_pose.pose.pose.position.x - self.posicao.pose.pose.position.x)
    
        return kp*(angle_r - self.yaw)
    
    def move2ref (self, x_ref, y_ref):
        ref_pose = Odometry()
        ref_pose.pose.pose.position.x = x_ref
        ref_pose.pose.pose.position.y = y_ref
        ref_tol = 0.01

        vel_msg = Twist()

        while self.ref_distance(ref_pose) >= ref_tol:

            vel_msg.linear.x = self.linear_vel_control(ref_pose)
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = self.angular_vel_control(ref_pose)

            self.vel_publisher.publish(vel_msg)

            self.rate.sleep()

        vel_msg.linear.x = 0
        vel_msg.angular.z = 0

        self.vel_publisher.publish(vel_msg)

        rospy.loginfo("Finished")

if __name__ == '__main__':
    bot = TurtleBotControl()
    bot.move2ref(-10,-20)
    
    