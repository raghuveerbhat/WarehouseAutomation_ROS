#!/usr/bin/python3
import actionlib
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseFeedback, MoveBaseResult
import time

class NavigationStack():
	def __init__(self):
		rospy.set_param('path_id', 0)
		self.client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
		rospy.loginfo("Waiting for move base server")
		self.client.wait_for_server()
		self.path = 0
		self.rest_point = [-1.9504327774047852, -4.748626708984375, 0, 0, 0, -0.7032243532516592, 0.7109680084179495]
		self.landmarks = [[1.4473416805267334, 0.41364067792892456, 0.0, 0.0, 0.0, -0.011503674603633503, 0.9999338305461085],
							[1.0017170906066895, -1.3638901710510254, 0.0, 0.0, 0.0, 0.010521600528978107, 0.9999446464291454],
							[0.9813948273658752, -3.1724913120269775, 0.0, 0.0, 0.0, 0.0013030230571348166, 0.999999151065096],
							[0.9532657861709595, -4.968116283416748, 0.0, 0.0, 0.0, -0.0022372289058545302, 0.9999974974002799],      
							[1.4260408878326416, -6.819857120513916, 0.0, 0.0, 0.0, -0.0015038960997956234, 0.9999988691476211],      
							[1.5627377033233643, -8.728625297546387, 0.0, 0.0, 0.0, -0.02648123511897974, 0.9996493106017597]]
		while not rospy.has_param('aruco_operation'):
			print("WAITING FOR ARUCO TO SET PARAM")
		self.sync_count = 0
		self.process()

	def return_goal(self, landmark):
		goal = MoveBaseGoal()
		goal.target_pose.header.frame_id = 'map'
		goal.target_pose.pose.position.x = landmark[0]
		goal.target_pose.pose.position.y = landmark[1]
		goal.target_pose.pose.position.z = landmark[2]
		goal.target_pose.pose.orientation.x = landmark[3]
		goal.target_pose.pose.orientation.y = landmark[4]
		goal.target_pose.pose.orientation.z = landmark[5]
		goal.target_pose.pose.orientation.w = landmark[6]
		return goal

	def process(self):
		for i, landmark in enumerate(self.landmarks):
			rospy.set_param('path_id', 0)
			rest_goal = self.return_goal(self.rest_point)	# Go to rest point 
			self.client.send_goal(rest_goal)
			self.client.wait_for_result()
			print("REST POINT REACHED")
			time.sleep(5)     	

			goal = self.return_goal(landmark)	# Go to 1st landmark
			self.client.send_goal(goal)
			self.client.wait_for_result()
			rospy.set_param('path_id', i+1)
			time.sleep(5)
			print("GOAL {} REACHED".format(i))
			while (not rospy.is_shutdown()) and (rospy.get_param('aruco_operation') == 1):
				print("WAITING FOR ARUCO TO SET PARAM")
			print("ARUCO OPERATION COMPLETED")

			



if __name__ == '__main__':
	rospy.init_node("navigation_node")
	node = NavigationStack()
	rospy.spin()
