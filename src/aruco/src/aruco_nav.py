#!/usr/bin/python3
import rospy
import math
import numpy as np
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import Twist

class ArucoNav():
    def __init__(self):
        self.kP = 0.1
        self.target = 90
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.get_odom)
        self.control_pub = rospy.Publisher("/cmd_vel", Twist, self.rotate_robot)
        self.rotate_angle = 90
        self.count = 0
        self.thresh = 3


    def get_odom(self, msg):
        position, orientation = msg.pose.pose.position, msg.pose.pose.orientation
        self.pos_x, self.pos_y, self.pos_z = position.x, position.y, position.z
        or_x, or_y, or_z, or_w = orientation.x, orientation.y, orientation.z, orientation.w
        self.roll, self.pitch, self.yaw = euler_from_quaternion([or_x, or_y, or_z, or_w])
        self.yaw = np.rad2deg(self.yaw)
        self.rotate_robot()

    def rotate_robot(self):
        control_command = Twist()
        if self.count == 0:
            self.target_yaw = self.yaw + self.rotate_angle  # Target yaw in degree
            self.count += 1
        if (self.target_yaw - self.yaw) > self.thresh:
            angle_diff = np.deg2rad(self.target_yaw - self.yaw)
            control_command.angular.z = self.kP * (angle_diff)
            self.control_pub.publish(control_command)
        else:
            print("EXIT------------------------------------------------------------------------------------")
            exit()

if __name__ == '__main__':
    rospy.init_node("ArucoNav")
    node = ArucoNav()
    rospy.spin()
