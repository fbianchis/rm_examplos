#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import random
import numpy as np

def variaVelocidadeAngular(angulo):
    return (random.random()-0.5)*angulo


def movimenta ():

    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

    rospy.init_node('movimenta_no', anonymous=True)

    rate = rospy.Rate(0.4)

    vetor = Twist()

    vetor.linear.x = 2
    vetor.linear.y = 0
    vetor.linear.z = 0

    while(not rospy.is_shutdown()):
        vetor.angular.z = variaVelocidadeAngular(np.pi)

        pub.publish(vetor)
        
        rate.sleep()

if __name__ == '__main__':
    try:
        movimenta()
    except rospy.ROSInterruptException:
        pass
