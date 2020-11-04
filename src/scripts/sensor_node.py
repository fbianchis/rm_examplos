#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def sensor():
    
    pub = rospy.Publisher('sensor_topic', String, queue_size=10)

    rospy.init_node('sensor_node', anonymous=True)

    rate = rospy.Rate(1)

    i = 0

    while(not rospy.is_shutdown()):
        sensor_msg = "value %s" %i
        
        rospy.loginfo(sensor_msg)

        pub.publish(sensor_msg)
        
        rate.sleep()

        i += 1

if __name__ == '__main__':
    try:
        sensor()
    except rospy.ROSInterruptException:
        pass
