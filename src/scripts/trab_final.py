#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
import numpy as np


class TurtleBotControl:
    def __init__(self):
       
        rospy.init_node('turtlebotcontrol_node', anonymous=True) 

        self.vel_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        self.odom_subscriber = rospy.Subscriber("/odom", Odometry, self.get_odom)

        self.lidar_subscriver = rospy.Subscriber("/scan", LaserScan, self.get_lidar)

        self.posicao = Odometry()

        self.lidar = LaserScan()

        self.rate = rospy.Rate(10)
        
        
        self.controle = True

        self.roll = 0.0
        self.pith = 0.0
        self.yaw = 0.0

        rospy.sleep(5)

    def get_lidar(self, msg):
        
        self.lidar = msg

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
        
        #angle_r = np.arctan2(ref_pose.pose.pose.position.y - self.posicao.pose.pose.position.y, ref_pose.pose.pose.position.x - self.posicao.pose.pose.position.x)
        menorAngulo, menorDistancia = self.retornaAnguloMenorDistancia() 
        i = 0

        
        if(menorDistancia < 0.5):
            
            if(menorAngulo >= 0 and menorAngulo <= 60):
                #print("entre 0 e 90")
                print(menorAngulo*(-1)+np.rad2deg(self.yaw)+10)
                angle_r = np.deg2rad(menorAngulo*(-1))
                self.controle = False
                return angle_r
            elif(menorAngulo >= 300 and menorAngulo <=359):
                #print("entre -1 e -90")
                #print("Angulo Menor ->"+str(menorAngulo - 360))
                print(menorAngulo*(-1)+np.rad2deg(self.yaw))
                angle_r = np.deg2rad((menorAngulo - 360)*(-1)+10)
                #print("Angulo Objetivo ->"+str(np.rad2deg(angle_r)))
                self.controle = False
                return angle_r
            

        angle_r = np.arctan2(ref_pose.pose.pose.position.y - self.posicao.pose.pose.position.y, ref_pose.pose.pose.position.x - self.posicao.pose.pose.position.x)
        return kp*(angle_r - self.yaw)

        #if i:
            #print("evitando colisão")
        #else: 
            #print("controle")
    
    
    def retornaAnguloMenorDistancia (self):
        
        return self.lidar.ranges.index(min(self.lidar.ranges)), min(self.lidar.ranges)

    def move2ref (self, x_ref, y_ref):
        ref_pose = Odometry()
        ref_pose.pose.pose.position.x = x_ref
        ref_pose.pose.pose.position.y = y_ref
        ref_tol = 0.01

        vel_msg = Twist()

        while self.ref_distance(ref_pose) >= ref_tol:
            if self.controle :
            
                vel_msg.linear.x = self.linear_vel_control(ref_pose) if self.linear_vel_control(ref_pose) <= 0.22 else 0.22
                vel_msg.linear.y = 0
                vel_msg.linear.z = 0

                vel_msg.angular.x = 0
                vel_msg.angular.y = 0
                vel_msg.angular.z = self.angular_vel_control(ref_pose) if self.angular_vel_control(ref_pose) <= 2.84 else 2.84
            else:
                vel_msg.linear.x = 0
                vel_msg.angular.z = 0
                self.vel_publisher.publish(vel_msg)
                anguloObjetivo = self.angular_vel_control(ref_pose) + self.yaw
                i = 0
                #rospy.loginfo("Angulo Objetivo-> "+ str(anguloObjetivo))
                #rospy.loginfo("YAW-> "+ str(self.yaw))
                #rospy.loginfo("Sub-> "+ str(abs(anguloObjetivo - self.yaw)))
                while abs(anguloObjetivo - self.yaw) >= 0.1:
                    #rospy.loginfo("YAW-> "+ str(np.rad2deg(self.yaw)))
                    if (anguloObjetivo - self.yaw < 0):
                        vel_msg.angular.z = -0.5
                    else: 
                        vel_msg.angular.z = 0.5
                    self.vel_publisher.publish(vel_msg)
                    self.rate.sleep()
                while i < 5:
                    vel_msg.angular.z = 0
                    vel_msg.linear.x  = 0.22 
                    self.vel_publisher.publish(vel_msg)
                    self.rate.sleep()
                    i += 1
                self.controle = True
                
                

            self.vel_publisher.publish(vel_msg)
            self.rate.sleep()

        vel_msg.linear.x = 0
        vel_msg.angular.z = 0

        self.vel_publisher.publish(vel_msg)

        rospy.loginfo("Finished")

if __name__ == '__main__':
    bot = TurtleBotControl()
    
    while True:
        x_ref = float(input("Informe a coordenada destino em x: "))
        y_ref = float(input("Informe a coordenada destino em y: "))
        bot.move2ref(x_ref,y_ref)
        continua = input("Digite Sim para executar novamente: ")
        if(continua != "Sim"):
            break
    
    
    