#!/usr/bin/python3
import sys
import copy
import numpy as np
import cv2
import rospy
from geometry_msgs.msg import Pose, PoseArray, Point, Quaternion, PoseStamped
from nav_msgs.msg import Path
from tf.msg import tfMessage
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from move_base_msgs.msg import MoveBaseGoal
from geometry_msgs.msg import Twist
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from sensor_msgs.msg import LaserScan


class PathPlanner(object):
    def __init__(self):
        self.map_set = False
        self.recevied_pose = False
        self.check = 0
        self.kP = 0.2
        self._latest_scan = None
        self.rest_point = Pose(Point(-1.9504327774047852, -4.748626708984375, 0), Quaternion(0, 0, -0.7032243532516592, 0.7109680084179495))
        # self.rest_point = [-1.9504327774047852, -4.748626708984375, 0, 0, 0, -0.7032243532516592, 0.7109680084179495]
        self.landmarks = [[1.4473416805267334, 0.41364067792892456, 0.0, 0.0, 0.0, -0.011503674603633503, 0.9999338305461085],
                            [1.0017170906066895, -1.3638901710510254, 0.0, 0.0, 0.0, 0.010521600528978107, 0.9999446464291454],
                            [0.9813948273658752, -3.1724913120269775, 0.0, 0.0, 0.0, 0.0013030230571348166, 0.999999151065096],
                            [0.9532657861709595, -4.968116283416748, 0.0, 0.0, 0.0, -0.0022372289058545302, 0.9999974974002799],      
                            [1.4260408878326416, -6.819857120513916, 0.0, 0.0, 0.0, -0.0015038960997956234, 0.9999988691476211],      
                            [1.5627377033233643, -8.728625297546387, 0.0, 0.0, 0.0, -0.02648123511897974, 0.9996493106017597]]
        self.map_subscriber = rospy.Subscriber("/map", OccupancyGrid, self.map_callback, queue_size=1)
        self.pose_subscriber = rospy.Subscriber("/estimatedpose", PoseStamped, self.pose_callback,queue_size=1)
        self.path_pub = rospy.Publisher("/predicted_path", Path, queue_size=1)
        self.control_pub = rospy.Publisher("/cmd_vel", Twist, self.control_robot, queue_size=1)
        self.rate = rospy.Rate(30)
        self.rate2 = rospy.Rate(10)
        self.main_process()

    def map_callback(self, map):
        if not self.map_set:
            self.occupancy_map = map
            self.set_map_params(self.occupancy_map)
            self.img = np.array(self.occupancy_map.data).reshape(self.map_height, self.map_width)
            self.img = cv2.flip(self.img, 1)
            self.img[np.where(self.img==0)]=0
            self.img[np.where(self.img==-1)]=255
            self.img[np.where(self.img==100)]=255
            self.img = self.img.astype(np.uint8)
            self.cost_map = cv2.dilate(self.img, np.ones((5,5), 'uint8'), iterations = 1)
            self.img = cv2.addWeighted(self.img, 1, self.cost_map, 1, 0).astype(np.uint8)
            self.draw_img = copy.deepcopy(self.img)
            self.map_set = True

    def set_map_params(self, occupancy_map):
        self.map_width = occupancy_map.info.width
        self.map_height = occupancy_map.info.height
        self.map_resolution = occupancy_map.info.resolution # in m per pixel
        self.map_data =  occupancy_map.data
        self.map_origin_x = (occupancy_map.info.origin.position.x +
                             (self.map_width / 2.0) * self.map_resolution)
        self.map_origin_y = ( occupancy_map.info.origin.position.y +
                              (self.map_height / 2.0) * self.map_resolution)
        rospy.loginfo("Sensor model map set.")

    def get_grid_pos(self, pose):
        pos_x, pos_y, pos_z = pose.position.x, pose.position.y, pose.position.z
        grid_x = self.map_width - int((pos_x - self.occupancy_map.info.origin.position.x) / self.occupancy_map.info.resolution)
        grid_y = int((pos_y - self.occupancy_map.info.origin.position.y) / self.occupancy_map.info.resolution)
        return grid_x, grid_y

    def pos_from_grid(self, pose):
        grid_x, grid_y = pose
        pos_x = ((self.map_width-grid_x)*self.occupancy_map.info.resolution) + self.occupancy_map.info.origin.position.x
        pos_y = (grid_y*self.occupancy_map.info.resolution) + self.occupancy_map.info.origin.position.y
        return pos_y, pos_x

    def pose_callback(self, estimated_pose):
        self.recevied_pose = True
        self.estimated_pose = copy.deepcopy(estimated_pose)

    def in_map_bounds(self, node):
        if (node[1]>=self.map_width) or (node[1] <= 0) or (node[0] <= 0) or (node[0] >= self.map_height):
            return False
        else:
            return True

    def get_adjacent_nodes(self, node):
        x, y = node
        traversable_nodes = []
        possible_nodes = [[x-1, y+1], [x, y+1], [x+1, y+1], [x+1, y], [x+1, y-1], [x, y-1], [x-1, y-1], [x-1, y]]
        for node in possible_nodes:
            if (self.img[node[1], node[0]]!=0) or (not self.in_map_bounds(node)):
                pass
            else:
                traversable_nodes.append(node)
        # for pt in traversable_nodes:
        #     self.draw_img = cv2.circle(self.draw_img, tuple(pt), 1, (120,120,0), -1)
        return traversable_nodes

    def get_cost(self, adjacent_nodes, goal_node):
        costs = []
        for adj_node in adjacent_nodes:
            dist_goal = ((goal_node[0]-adj_node[0])**2 + (goal_node[1]-adj_node[1])**2)**0.5
            costs.append(dist_goal)
        return costs
    
    def get_best_node(self, start_node, goal_node):
        msg = Path()
        msg.header.frame_id='map'
        msg.header.stamp=rospy.Time.now()
        self.best_path = [start_node]
        self.temp_node = copy.deepcopy(start_node)
        count = 0
        while ((self.temp_node[0]!= goal_node[0]) or (self.temp_node[1]!=goal_node[1])) and (count<100000):
            adj_nodes = self.get_adjacent_nodes(self.temp_node)
            costs = self.get_cost(adj_nodes, goal_node)
            if len(costs)>0:
                # if np.min(costs)<=1:
                #     break
                while len(costs)>0:
                    best_cost_idx = np.argmin(costs)
                    if adj_nodes[best_cost_idx] in self.best_path:
                        costs.pop(best_cost_idx)
                        adj_nodes.pop(best_cost_idx)
                    else:
                        best_next_node = adj_nodes[best_cost_idx]
                        self.best_path.append(best_next_node)
                        break
            else:
                break
            self.temp_node = copy.deepcopy(best_next_node)
            count+=1
        if count>=10000:
            print("EXCEEDED CHECK")
            pass
        else:
            for pt in self.best_path:
                map_pt = self.pos_from_grid(pt)
                pose = PoseStamped()
                pose.pose.position.x = map_pt[1]
                pose.pose.position.y = map_pt[0]
                msg.poses.append(pose)
                self.draw_img = cv2.circle(self.draw_img, tuple(pt), 1, (0,255,0), -1)
        return msg

    def get_angle(self, pt1, pt2):
        m = np.arctan2(pt2[0]-pt1[0], pt2[1]-pt1[1]) 
        return np.rad2deg(m)

    def control_robot(self, estimated_pose, goal_pose):
        control_command = Twist()
        grid_error_x, grid_error_y = estimated_pose[0]-goal_pose[0], estimated_pose[1]-goal_pose[1]
        estimated_pose_map, goal_pose_map = self.pos_from_grid(estimated_pose), self.pos_from_grid(goal_pose)
        x_error =  goal_pose_map[0] - estimated_pose_map[0]
        y_error =  goal_pose_map[1] - estimated_pose_map[1]

        estimated_orientation = self.estimated_pose.pose.orientation
        or_x, or_y, or_z, or_w = estimated_orientation.x, estimated_orientation.y, estimated_orientation.z, estimated_orientation.w
        _, _, estimated_pose_yaw = euler_from_quaternion([or_x,or_y,or_z,or_w])
        estimated_pose_yaw = np.rad2deg(estimated_pose_yaw)
        goal2, goal1 = self.pos_from_grid(self.best_path[-1]), self.pos_from_grid(self.best_path[0])
        goal_yaw = self.get_angle(goal1, goal2)
        angle_error = (goal_yaw - estimated_pose_yaw)
        angle_error = np.deg2rad(angle_error)
        print(x_error, y_error, goal_yaw, estimated_pose_yaw, goal_yaw - estimated_pose_yaw)


        if abs(grid_error_x)>2 or abs(grid_error_y) > 2:
            control_command.linear.x = self.kP*x_error
            control_command.linear.y = self.kP*y_error
            control_command.angular.z = self.kP*angle_error
            self.control_pub.publish(control_command)
        else:
            control_command.linear.x = 0
            control_command.linear.y = 0
            control_command.linear.z = 0
            control_command.angular.z = 0
            for i in range(10):
                self.control_pub.publish(control_command)
            print("REACHED REST POINT")

    def main_process(self):
        self.path_planning = False
        while not rospy.is_shutdown():
            if self.map_set:
                self.draw_img = cv2.merge((copy.deepcopy(self.img), copy.deepcopy(self.img), copy.deepcopy(self.img)))
                if rospy.has_param('path_id'):
                    grid_x, grid_y = self.get_grid_pos(self.rest_point)
                    self.grid_goal = [grid_x, grid_y]
                    self.draw_img = cv2.rectangle(self.draw_img, (grid_x-5, grid_y-5), (grid_x+5, grid_y+5), (0, 255, 0), 1)
                if self.recevied_pose:
                    grid_x, grid_y = self.get_grid_pos(self.estimated_pose.pose)
                    self.start_node = [grid_x, grid_y]
                    msg = self.get_best_node(self.start_node, self.grid_goal)
                    self.path_pub.publish(msg)
                    self.draw_img = cv2.rectangle(self.draw_img, (grid_x-5, grid_y-5), (grid_x+5, grid_y+5), (255, 255, 255), 1)
                    if rospy.has_param("completed") and not self.path_planning:
                        completed = rospy.get_param('completed')
                        # if completed and len(self.best_path)>3:
                        if completed:
                            if len(self.best_path)>3:
                                self.control_robot(self.start_node, self.best_path[min(10, len(self.best_path)-1)])
                            else:
                                self.path_planning = True
                                control_command = Twist()
                                control_command.linear.x = 0
                                control_command.angular.z = 0
                                self.rate2.sleep()
                                self.control_pub.publish(control_command)
                                print("REACHED REST POINT")

                cv2.imshow("# TODO: ",self.draw_img)
                cv2.waitKey(1)
            self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node("path_planner")
    node = PathPlanner()
    rospy.spin()
