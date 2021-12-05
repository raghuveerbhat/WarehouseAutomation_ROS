#!/usr/bin/python3
import rospy
import math
import numpy as np
import rospy
import cv_bridge
import cv2
import math
import time
from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import Twist

class ArucoNav():
    def __init__(self):
        self.kP = 0.001
        self.target = 90
        self.rotate_angle = 90
        self.count = 0
        self.thresh = 1
        self.received_cam_intrinsics = False
        self.marker_size  = 0.3
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
        self.aruco_parameters = cv2.aruco.DetectorParameters_create()
        self.bridge = cv_bridge.CvBridge()
        rospy.set_param('aruco_operation', 0)
        self.count = 0
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.get_odom)
        self.control_pub = rospy.Publisher("/cmd_vel", Twist, self.rotate_robot)
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, callback=self.image_callback)
        self.caminfo_sub = rospy.Subscriber('/camera/rgb/camera_info', CameraInfo, callback=self.caminfo_callback)

    def caminfo_callback(self, caminfo_msg):
        if self.received_cam_intrinsics:
            pass
        else:
            if caminfo_msg.K == [0, 0, 0, 0, 0, 0, 0, 0, 0]:
                rospy.logwarn("CameraInfo message has K matrix all zeros")
            else:
                self.dist_coeff = caminfo_msg.D
                self.cam_matrix = np.zeros((3, 3))
                for i in range(0, 3):
                    for j in range(0, 3):
                        self.cam_matrix[i][j] = caminfo_msg.K[i * 3 + j]
                self.received_cam_intrinsics = True

    def get_odom(self, msg):
        position, orientation = msg.pose.pose.position, msg.pose.pose.orientation
        self.pos_x, self.pos_y, self.pos_z = position.x, position.y, position.z
        or_x, or_y, or_z, or_w = orientation.x, orientation.y, orientation.z, orientation.w
        self.roll, self.pitch, self.yaw = euler_from_quaternion([or_x, or_y, or_z, or_w])
        self.yaw = np.rad2deg(self.yaw)

    def rotate_robot(self, angle_error):
        control_command = Twist()
        if abs(angle_error) > self.thresh:
            print(angle_error)
            control_command.angular.z = self.kP * angle_error
            self.control_pub.publish(control_command)
        else:
            rospy.set_param('aruco_operation', 0)
            self.control_pub.publish(control_command)
            print("COMPLETED")

    def corner_fix(self, corner):
        corner = np.array(corner)[0]
        tl,tr,br,bl = corner[0], corner[1], corner[2], corner[3]
        tl[0] = self.img_width - tl[0]
        tr[0] = self.img_width - tr[0]
        br[0] = self.img_width - br[0]
        bl[0] = self.img_width - bl[0]
        corner_new = [[tl,tr,br,bl]]
        return np.array(corner_new)

    def image_callback(self, img_msg):
        if self.received_cam_intrinsics:
            img = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
            gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            self.img_width = gray_img.shape[1]
            corners, ids, reg = cv2.aruco.detectMarkers(image=cv2.flip(gray_img, 1), dictionary=self.aruco_dict, parameters=self.aruco_parameters)
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.marker_size, self.cam_matrix, self.dist_coeff)
            if rospy.has_param('/path_id'):
                path_id = rospy.get_param('/path_id')
                if (len(corners) > 0) and (path_id > 0):
                    rospy.set_param('aruco_operation', 1)
                    if path_id in ids:
                        idx = np.where(ids==path_id)[0][0]
                        corner, rvec, tvec = corners[idx], rvecs[idx], tvecs[idx] 
                        corner = self.corner_fix(corner)
                        img = cv2.aruco.drawDetectedMarkers(img, [corner])
                        # img = cv2.aruco.drawAxis(img, self.cam_matrix, self.dist_coeff, rvec, tvec, self.marker_size)
                        yaw = 1 * math.atan2(tvecs[idx][0][0], tvecs[idx][0][2])
                        yaw = yaw / math.pi * 180
                        zDist = math.sqrt(tvec[0][0] ** 2 + tvec[0][1] ** 2 + tvec[0][2] ** 2)
                        self.rotate_robot(yaw)
                        cv2.putText(img, "%d tag - %.2f cm - %.2f deg" % (idx, (zDist * 100), yaw), tuple(corner[0][1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0))
                        
            cv2.imshow("frame",img)
            k = cv2.waitKey(1)
            if k == ord("q"):
                exit(1)

if __name__ == '__main__':
    rospy.init_node("ArucoNav")
    node = ArucoNav()
    rospy.spin()
