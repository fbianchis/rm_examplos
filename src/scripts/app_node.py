#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def update_msg(mensagem):

    rospy.loginfo(rospy.get_caller_id()+"Mensagem: %s", mensagem.data)

def app():

    rospy.init_node('app_node', anonymous=True)

    rospy.Subscriber('sensor_topic', String, update_msg)

    rospy.spin()

if __name__ == '__main__':
    app()