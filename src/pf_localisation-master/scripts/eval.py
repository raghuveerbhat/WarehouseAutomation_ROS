#!/usr/bin/python3

"""
This is the main entry point for the particle filter exercise node. It
subscribes to laser, map, and odometry and creates an instance of
pf.PFLocaliser() to do the localisation.
"""

import rospy
from geometry_msgs.msg import ( PoseStamped, PoseWithCovarianceStamped,
								PoseArray, Quaternion )
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt


class ParticleFilterLocalisationNodeEval(object):
	def __init__(self):
		self.estimatedPose = None
		self.groundtruthPose = None
		self.Pose_error = []
		self.Orientation_error = []
		self.interval = []
		self.groundtruthList = []
		self.counter = 0

	def evaluate(self):
		while(self.counter<61):
			self.counter = self.counter+1
			rospy.sleep(1.0)
			self.estimatedPose = None
			self.groundtruthPose = None
			self.estimatedPose = rospy.wait_for_message("/estimatedpose",PoseStamped)
			if self.estimatedPose != None:
				self.groundtruthPose = rospy.wait_for_message("/ground_truth/state",Odometry)
				if self.groundtruthPose != None:
					self.interval.append(self.counter)
					print("EstimatedPose", self.estimatedPose)
					print("groundtruthPose", self.groundtruthPose)

					'''TODO : Akshay

					#calcualte the difference between estimatedPose and groundtruthpose for only [x,y]
					and average it out. store the average error in  Pose_error.
					#In same way, difference between oreintation should be stored in orientation error

					'''

		'''
		#TODO : Akshay
		#plot using matplotlib, plot the error graph using list created error VS interval and save
		'''

if __name__ == '__main__':
	# --- Main Program  ---
	rospy.init_node("pf_localisation")
	node = ParticleFilterLocalisationNodeEval()
	node.evaluate()
