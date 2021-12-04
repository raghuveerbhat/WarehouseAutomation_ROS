from geometry_msgs.msg import Pose, PoseArray, Point, Quaternion
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from . pf_base import PFLocaliserBase
import math
import rospy

from . util import rotateQuaternion, getHeading
from random import random

import os
from time import time
import copy
import numpy as np
from numpy.random import random
import matplotlib.pyplot as plt
import json
from pf_localisation.localization import LocalizationTechniques

class PFLocaliser(PFLocaliserBase):

	def __init__(self):
		super(PFLocaliser, self).__init__()
		self.base_path = os.path.dirname(__file__)[:-20]
		with open(self.base_path+"/config/config.json", "r") as jsonfile:
			self.config = json.load(jsonfile)
		# Motion model parameters
		self.ideal_motion_model = self.config['ideal_motion_model']
		if not self.ideal_motion_model:
			self.ODOM_ROTATION_NOISE = self.config['odom_rotation_noise	']											# Odometry rotation noise added
			self.ODOM_TRANSLATION_NOISE = self.config['odom_translation_noise']										# Odometry translation noise added
			self.ODOM_DRIFT_NOISE = self.config['odom_drift_noise']													# Odometry drift noise added

		# Sensor model parameters
		self.NUMBER_PREDICTED_READINGS = self.config['num_predicted_readings'] 										# Number of readings to predict

		# # Particle initialization Parameters
		self.mcl_technique = self.config['mcl_technique']															# Technique of localization to run
		self.resample_technique = Resample(self.config['resample_technique'])    									# Class to handle resampling of data
		self.center_estimate_method = self.config['center_estimate_method']											# Method of estimating pose
		self.estimate_method = EstimatePose(self.center_estimate_method)											# Class to estimate pose

		self.plot_graph = False
		self.data_list = []
		self.count = 0
		self.init_flag=False

	def initialise_particle_cloud(self, initialpose):
		"""
		Set particle cloud to initialpose plus noise

		Called whenever an initialpose message is received (to change the
		starting location of the robot), or a new occupancy_map is received.
		self.particlecloud can be initialised here. Initial pose of the robot
		is also set here.

		:Args:
			| initialpose: the initial pose estimate
		:Return:
			| (geometry_msgs.msg.PoseArray) poses of the particles
		"""
		self.loc_tech = LocalizationTechniques(self.config, self.sensor_model, self.occupancy_map)
		self.particlecloud = [Pose()]
		self.odom_initialised=False
		self.init_flag = False
		pose = self.loc_tech.initialize_point_cloud(initialpose)
		self.init_flag = True
		return pose

	def calculate_weights(self, scan):
		"""
		Uses the sensor model to obtain the weight for each particle.
		:Args:
			 | scan (sensor_msgs.msg.LaserScan): laser scan to use for update
		Return:
			| Normalized weights of each particle
		"""
		# Calculate weights of each particle based on observation
		weights = []
		aug_w_avg = 0
		for i, pose in enumerate(self.particlecloud.poses):
			wt = self.sensor_model.get_weight(scan, pose)
			weights.append(wt)
			pos = self.particlecloud.poses[i].position
			orientation = self.particlecloud.poses[i].orientation
			pos_x, pos_y, pos_z = pos.x, pos.y, pos.z
			or_x, or_y, or_z, or_w = orientation.x, orientation.y, orientation.z, orientation.w
			aug_w_avg += wt
		if self.loc_tech.mcl_technique == 'augmented':
			self.loc_tech.param_aug_w_avg = (aug_w_avg / self.loc_tech.num_particles)
		weights = np.array(weights)
		weights/=weights.sum()
		return weights

	def update_particle_cloud(self, scan):
		"""
		This should use the supplied laser scan to update the current
		particle cloud. i.e. self.particlecloud should be updated.

		:Args:
			| scan (sensor_msgs.msg.LaserScan): laser scan to use for update

		"""
		self.resampled_data, self.resampled_weights = [], []
		self.current_scan = copy.deepcopy(scan)
		self.weights = self.calculate_weights(self.current_scan)
		indexes = self.resample_technique.resample(self.weights)
		self.updated_poses, self.updated_data, self.resampled_data, self.resampled_weights = self.loc_tech.update_particle_cloud(self.weights, indexes, self.particlecloud)
		print(self.resampled_weights.shape)

	def l2_norm(self, actual, current):
		return np.sum((np.array(actual)-np.array(current))**2)**0.5

	def estimate_pose(self):
		"""
		This should calculate and return an updated robot pose estimate based
		on the particle cloud (self.particlecloud).

		Create new estimated pose, given particle cloud
		E.g. just average the location and orientation values of each of
		the particles and return this.

		Better approximations could be made by doing some simple clustering,
		e.g. taking the average location of half the particles after
		throwing away any which are outliers

		:Return:
			| (geometry_msgs.msg.Pose) robot's estimated pose.
		 """

		estimated_pose = self.estimate_method.update(self.resampled_data, self.resampled_weights)
		self.particlecloud.poses = copy.deepcopy(self.loc_tech.create_random_particles(self.updated_poses))
		if self.plot_graph:
			true_pos_x = self.last_odom_pose.pose.pose.position.x + self.sensor_model.map_origin_x
			true_pos_y = self.last_odom_pose.pose.pose.position.y + self.sensor_model.map_origin_y
			true_pos_z = self.last_odom_pose.pose.pose.position.z
			or_l1_error = abs(getHeading(new_or) - getHeading(self.last_odom_pose.pose.pose.orientation))
			l2_dist = self.l2_norm([pose_[0], pose_[1], pose_[2]], [true_pos_x, true_pos_x, true_pos_z])
			self.data_list.append([or_l1_error, l2_dist])
			if self.count%100 == 0:
				np.save("weighted.npy",np.array(self.data_list))
		return estimated_pose

class Resample:
	def __init__(self, method='systematic_resample'):
		self.method = method

	def resample(self, weights):
		indexes = None
		if self.method == 'systematic_resample':
			indexes = self.systematic_resample(weights)
		elif self.method == 'multinomial':
			indexes = self.multinomial_resample(weights)
		return indexes

	def systematic_resample(self, weights):
		""" Performs the systemic resampling algorithm used by particle filters.
		This algorithm separates the sample space into N divisions. A single random
		offset is used to to choose where to sample from for all divisions. This
		guarantees that every sample is exactly 1/N apart.
		Parameters
		----------
		weights : list-like of float
			list of weights as floats
		Returns
		-------
		indexes : ndarray of ints
			array of indexes into the weights defining the resample. i.e. the
			index of the zeroth resample is indexes[0], etc.
		"""
		N = len(weights)
		# make N subdivisions, and choose positions with a consistent random offset
		positions = (np.random.random() + np.arange(N)) / N
		indexes = np.zeros(N, 'i')
		cumulative_sum = np.cumsum(weights)
		i, j = 0, 0
		while i < N:
			if positions[i] < cumulative_sum[j]:
				indexes[i] = j
				i += 1
			else:
				j += 1
		return indexes

	def multinomial_resample(self, weights):
		"""
		Parameters
		----------
		weights : list-like of float
			list of weights as floats
		Returns
		-------
		indexes : ndarray of ints
			array of indexes into the weights defining the resample. i.e. the
			index of the zeroth resample is indexes[0], etc.
		"""
		cum_sum = np.cumsum(weights)
		cum_sum[-1] = 1.  # avoid round-off errors: ensures sum is exactly one
		return np.searchsorted(cum_sum, np.random.random(len(weights)))

class K_Means:
	def __init__(self, data, weights, clusters = 2, max_iterations = 10):
		self.X1 = data
		self.weights = weights
		self.clusters = clusters
		self.max_iterations = max_iterations
		self.cluster_centers = self.X1[np.random.randint(0, self.X1.shape[0], self.clusters)]
		self.cluster_stats = np.zeros((self.clusters, 2))
		self.dists = np.zeros((self.X1.shape[0], self.clusters))
		self.mean_dist = np.zeros(self.max_iterations)
		self.max_idx = 0
		self.cluster_size = 0

		for self.itr in range(self.max_iterations):
			self.calc_distance_to_cluster_centers()
			self.assign_clusters_and_update_centers_and_calc_mean_dist()

	def calc_distance_to_cluster_centers(self):
		''' dist = ((x1-x2)**2 + (y1-y2)**2)**0.5 '''
		for i in range(self.clusters):
			self.dists[:, i] = np.sum((self.X1- self.cluster_centers[i])**2, axis=1)**0.5

	def assign_clusters_and_update_centers_and_calc_mean_dist(self):
		'''
			1) Each point is assigned to the closest cluster center
			2) Update the cluster center to be the mean of the cluster
		'''
		self.cluster_idx = np.argmin(self.dists, axis=1)
		self.dist = []
		self.cluster_dict = {}
		self.max_idx, max_len = 0, 0
		for i in range(self.clusters):
			cluster = self.X1[np.where(self.cluster_idx==i)]
			cluster_weights = self.weights[np.where(self.cluster_idx==i)]
			self.cluster_centers[i] = np.mean(cluster, axis=0)
			self.cluster_stats[i] = [np.mean(cluster_weights), np.std(cluster_weights)]
			self.dist.append(np.mean(self.dists[np.where(self.cluster_idx==i), i]))
			if len(cluster) > max_len:
				max_len = len(cluster)
				self.max_idx = i
		self.mean_dist[self.itr] = np.mean(np.array(self.dist).reshape(-1,1))

class EstimatePose:
	def __init__(self, method, sliding_window=5):
		self.method = method
		self.past_poses = []
		self.sliding_window = sliding_window

	def get_mean_pose(self, pose):
		self.past_poses.append(pose)
		if len(self.past_poses) > self.sliding_window:
			self.past_poses.pop(0)
		mean_pose = np.mean(self.past_poses, axis=0)
		return mean_pose

	def update(self, data, weights=None):
		pose = [0,0,0,0,0,0,0]
		if self.method=='weighted':
			pose = self.estimate_pose_weighted(data, weights)
		elif self.method=='kmeans':
			pose = self.estimate_pose_kmeans(data, weights)
		weighted_pose = self.get_mean_pose(pose)
		new_pos = Point(weighted_pose[0], weighted_pose[1], weighted_pose[2])
		new_or = Quaternion(weighted_pose[3], weighted_pose[4], weighted_pose[5], weighted_pose[6])
		estimated_pose = Pose(new_pos, new_or)
		return estimated_pose

	def estimate_pose_weighted(self, data, weights):
		"""
		This should calculate and return an updated robot pose estimate based
		on the particle cloud (self.particlecloud).

		Create new estimated pose, given particle cloud
		E.g. just average the location and orientation values of each of
		the particles and return this.

		Better approximations could be made by doing some simple clustering,
		e.g. taking the average location of half the particles after
		throwing away any which are outliers

		:Return:
			| (geometry_msgs.msg.Pose) robot's estimated pose.
		"""
		X, Y, Z = data[:,0], data[:,1], data[:,2]
		AX,AY,AZ,AW = data[:,3], data[:,4], data[:,5], data[:,6]
		weight_sum = sum(weights)
		pose = np.sum(data * np.expand_dims(weights,1), axis=0) / weight_sum
		self.cluster_size = data.shape[0]
		return pose

	def estimate_pose_kmeans(self, data, weights):
		"""
		This should calculate and return an updated robot pose estimate based
		on the particle cloud (self.particlecloud).

		Create new estimated pose, given particle cloud
		E.g. just average the location and orientation values of each of
		the particles and return this.

		Better approximations could be made by doing some simple clustering,
		e.g. taking the average location of half the particles after
		throwing away any which are outliers

		:Return:
			| (geometry_msgs.msg.Pose) robot's estimated pose.
		"""
		# Extract data from particle cloud list to perform K means clustering
		# K means clustering on data to find clusters
		self.kmeans = K_Means(data, weights, clusters=2, max_iterations=5)

		# Choose pose as mean pose of max cluster
		pose = self.kmeans.cluster_centers[self.kmeans.max_idx]
		self.cur_stats = self.kmeans.cluster_stats[self.kmeans.max_idx]
		self.cluster_size = self.kmeans.X1[np.where(self.kmeans.cluster_idx==self.kmeans.max_idx)].shape[0]

		# Return estimated pose
		return pose
