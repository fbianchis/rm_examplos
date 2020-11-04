import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
import numpy as np



class TurtleControl:
    def __init__(self):
        rospy.init_node("quat2euler_node", anonymous=True)
        self.vel_publisher = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)
        self.odom_subscriber = rospy.Subscriber("/odom", Odometry, self.orientation)
        self.odometry =Odometry()
        self.rate = rospy.Rate(10)
        self.yaw=0
        self.roll=0
        self.pitch=0
    
    def orientation(self, msg):
        self.odometry=msg
        orientation_q=self.odometry.pose.pose.orientation
        orientation_list=[orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (self.roll, self.pitch, self.yaw)=euler_from_quaternion(orientation_list)

    def ref_distance(self, ref_pose):
        return np.sqrt(((ref_pose.pose.pose.orientation.x - self.odometry.pose.pose.orientation.x)**2) + ((ref_pose.pose.pose.orientation.y - self.odometry.pose.pose.orientation.y)**2))

    def linear_vel_control (self, ref_pose, kp=1.5):
        distancia = self.ref_distance(ref_pose)
        return kp* distancia

    def angular_vel_control(self, ref_pose, kp=6):
        angle_r = np.arctan2(ref_pose.y-self.odometry.pose.pose.orientation.y ,ref_pose.x-self.odometry.pose.pose.orientation.x )
        return kp* (angle_r - self.yaw)

    def move2ref(self, x_ref, y_ref):
        ref_pose = Odometry()
        ref_pose.pose.pose.orientation.x = x_ref
        ref_pose.pose.pose.orientation.y = y_ref
        ref_tol = 0.01

        vel_msg = Twist()
        while self.ref_distance(ref_pose)>= ref_tol:
            vel_msg.linear.x= self.linear_vel_control(ref_pose)
            vel_msg.linear.y=0
            vel_msg.linear.z=0

            vel_msg.angular.x=0
            vel_msg.angular.y=0
            vel_msg.angular.z=self.angular_vel_control(ref_pose)

            self.vel_publisher.publish(vel_msg)

            self.rate.sleep()

        vel_msg.linear.x =0
        vel_msg.angular.z=0

        self.vel_publisher.publish(vel_msg)

        rospy.loginfo("finished")