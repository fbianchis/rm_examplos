#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import numpy as np

class TurtleControl:
    def __init__(self):

        rospy.init_node("turtlecontrol_node", anonymous=True)

        self.vel_publisher = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)

        self.pose_subscriver = rospy.Subscriber("/turtle1/pose", Pose, self.update_pose)

        self.pose = Pose()
        
        self.rate = rospy.Rate(10)

    def update_pose(self, msg):

        self.pose = msg

    def ref_distance(self, ref_pose):

        return np.sqrt(((ref_pose.x - self.pose.x)**2) + ((ref_pose.y - self.pose.y)**2))

    def linear_vel_control(self, ref_pose, kp = 3):

        distance = self.ref_distance(ref_pose)

        return kp * distance

    def angular_vel_control(self, ref_pose, kp = 6):

        angle_r = np.arctan2(ref_pose.y - self.pose.y, ref_pose.x - self.pose.x)
    
        return kp*(angle_r - self.pose.theta)

    def move2ref (self, x_ref, y_ref):
        ref_pose = Pose()
        ref_pose.x = x_ref
        ref_pose.y = y_ref
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
    bot = TurtleControl()
    bot.move2ref(2,2)
