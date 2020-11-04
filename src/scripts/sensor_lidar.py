#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
from tf.transformations import euler_from_quaternion
import numpy as np

class Lidar:
    def __init__(self):
        rospy.init_node("sensor_lidar_node", anonymous=True)
        rospy.Subscriber("/scan", LaserScan, self.update)
        self.vel_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.odom_subscriber = rospy.Subscriber("/odom", Odometry, self.get_odom)
        self.posicao = Odometry()
        self.lidar = LaserScan()
        self.rate = rospy.Rate(10)
        self.roll = 0.0
        self.pith = 0.0
        self.yaw = 0.0
    
    def update(self, msg):
        self.lidar = msg

    def get_odom(self, msg):
        
        self.posicao = msg
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        self.roll, self.pith, self.yaw = euler_from_quaternion(orientation_list)

    def rotate(self, angle):
        vel_msg = Twist()
        while (abs(angle - self.yaw) > 0.1):
            print(np.rad2deg(self.yaw))
            vel_msg.angular.z = 0.5
            self.vel_publisher.publish(vel_msg)
            self.rate.sleep()
        
        vel_msg.angular.z = 0.0
        self.vel_publisher.publish(vel_msg)
        self.plotGrafico(np.rad2deg(angle))
            

    def plotGrafico (self, titulo):
        plt.close()
        angulos = np.deg2rad(np.linspace(0,360,360))
        plt.figure()
        ax = plt.subplot(111,projection = 'polar')
        ax.plot(angulos, self.lidar.ranges)
        plt.title(str(titulo))
        plt.show()
        
        #plt.savefig('~/Imagens/A'+str(int(titulo))+'.png')
        #plt.close()


if __name__ == '__main__':
    try:
        sensor = Lidar()
        a = 1
        while(a != '2' ):
            a = input()

        sensor.rotate(np.deg2rad(-90))
        
            

    except rospy.ROSInterruptException:
        pass
