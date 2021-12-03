from geometry_msgs.msg import Pose, PoseArray, Point, Quaternion
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from . pf_base import PFLocaliserBase
import math
import rospy

from . util import rotateQuaternion, getHeading
from random import random

from time import time
import copy
import numpy as np
from numpy.random import random
import matplotlib.pyplot as plt


class PFLocaliser(PFLocaliserBase):

	def __init__(self):
		# ----- Call the superclass constructor
		super(PFLocaliser, self).__init__()

		# ----- Set motion model parameters
		self.ideal_motion_model = True
		if not self.ideal_motion_model:
			self.ODOM_ROTATION_NOISE = 0.1
			self.ODOM_TRANSLATION_NOISE = 0.05
			self.ODOM_DRIFT_NOISE = 0.05

		# ----- Sensor model parameters
		self.NUMBER_PREDICTED_READINGS = 20     # Number of readings to predict
		self.random_noise_pos_init = 0.7         # Noise added to position during init
		self.random_noise_or_init = 360         # Noise added to orientation during init (degree)
		self.random_noise_pos_update = 0.01	    # Noise added to position during updates
		self.random_noise_or_update = 5        # Noise added to orienation during updates (degree)
		self.num_particles = 1000               # Number of particles
		self.min_particles, self.max_particles = 30, 45              	# Number of particles
		self.resample_technique = Resample('systematic_resample')    		# Class to handle resampling of data
		self.center_estimate_method = 'weighted'
		self.estimate_method = EstimatePose(self.center_estimate_method)	# Method of estimation
		self.error_count = 0
		self.sensor_model.count = 0
		self.particle_init_method = 'normal'
		self.adaptive_mcl = "fixed_random_particle"		# Toggle random particle spawn
		if self.adaptive_mcl == "fixed_random_particle":
			self.random_pts_pct = 1
		elif self.adaptive_mcl == 'augmented':
			self.particle_init_method = 'normal'
			self.num_particles = 750
			self.min_particles = 750
			self.param_aug_w_slow = 0
			self.param_aug_w_fast = 0
			self.param_aug_w_avg = 0
			self.param_aug_alpha_slow = 0.4
			self.param_aug_alpha_fast = 0.7
			self.particles_to_randomize = 0
			self.random_noise_or_init_radians = 2*math.pi
			self.random_noise_or_update_radians = math.pi/6
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
		# Initialize data with header
		self.particlecloud = [Pose()]
		self.odom_initialised=False
		new_pos,new_q = None, None
		self.map_states = []
		index = 0
		self.init_flag = False
		for h in range(0, self.sensor_model.map_height):
			for w in range(0, self.sensor_model.map_width):
				if (self.sensor_model.map_data[index] >= 0.0) :
					p = Point()
					p.x = w * self.sensor_model.map_resolution
					p.y = h * self.sensor_model.map_resolution
					p.z = 0
					self.map_states.append(p)
				index += 1

		pose = PoseArray()
		pose.header = initialpose.header

		# Get position and orientation of initial mouse click on rviz
		pos = initialpose.pose.pose.position
		orientation = initialpose.pose.pose.orientation
		pos_x, pos_y, pos_z = pos.x, pos.y, pos.z
		or_x, or_y, or_z, or_w = orientation.x, orientation.y, orientation.z, orientation.w

		# Create N particles around initial mouse click with random noise
		# Noise is added along x and y dimensions of position (2d robot)
		# Noise is added in degrees only to yaw, since robot has only 1 axis of rotation
		for i in range(self.num_particles):
			if self.particle_init_method=='normal':
				if self.adaptive_mcl == 'augmented':
					new_pos = Point(np.random.normal(pos_x, self.random_noise_pos_init),\
									np.random.normal(pos_y, self.random_noise_pos_init),\
									pos_z)
				else:
					new_pos = Point(np.random.normal(pos_x, self.random_noise_pos_init),\
									np.random.normal(pos_y, self.random_noise_pos_init),\
									pos_z)
			elif self.particle_init_method=='uniform':
				new_pos = Point(np.random.uniform(0, self.sensor_model.map_width * self.sensor_model.map_resolution),\
								np.random.uniform(0, self.sensor_model.map_height * self.sensor_model.map_resolution),\
								pos_z)
			elif self.particle_init_method=='map':
				idx = np.random.randint(0, len(self.map_states))
				new_pos = self.map_states[idx]
			roll, pitch, yaw = 0, 0, np.deg2rad(np.random.uniform(0, self.random_noise_or_init))
			q = quaternion_from_euler(roll, pitch, yaw)
			new_q = Quaternion(q[0], q[1], q[2], q[3])
			pose.poses.append(Pose(new_pos, new_q))
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
		self.data = []
		self.param_aug_w_avg = 0
		for i, pose in enumerate(self.particlecloud.poses):
			wt = self.sensor_model.get_weight(scan, pose)
			weights.append(wt)
			pos = self.particlecloud.poses[i].position
			orientation = self.particlecloud.poses[i].orientation
			pos_x, pos_y, pos_z = pos.x, pos.y, pos.z
			or_x, or_y, or_z, or_w = orientation.x, orientation.y, orientation.z, orientation.w
			self.data.append([pos_x, pos_y, pos_z, or_x, or_y, or_z, or_w])
			if self.adaptive_mcl == 'augmented':
				self.param_aug_w_avg += wt / self.num_particles
		# Normalize weights
		weights = np.array(weights)
		weights/=weights.sum()

		self.normalized_weights = copy.deepcopy(weights) / np.max(copy.deepcopy(weights))

		return weights

	def update_particle_cloud(self, scan):
		"""
		This should use the supplied laser scan to update the current
		particle cloud. i.e. self.particlecloud should be updated.

		:Args:
			| scan (sensor_msgs.msg.LaserScan): laser scan to use for update

		"""
		self.current_scan = copy.deepcopy(scan)
		# Compute normalized weights for each particle
		self.weights = self.calculate_weights(self.current_scan)
		# print(np.mean(weight_temp), np.std(weight_temp))
		# Resample particles using weight distribution
		indexes = self.resample_technique.resample(self.weights)
		# Update each particle by adding noise, and resampling using indexes,
		# obtained with systematic resampling
		poses, new_idxs = [], []
		self.mean_weight, self.std_weight = np.mean(self.weights), np.std(self.weights)
		removed_count = 0

		if self.adaptive_mcl == "augmented":
			random_particles = []
			self.param_aug_w_slow += self.param_aug_alpha_slow * (self.param_aug_w_avg - self.param_aug_w_slow)
			self.param_aug_w_fast += self.param_aug_alpha_fast * (self.param_aug_w_avg - self.param_aug_w_fast)
			self.particles_to_randomize = max(0, 1.0 - self.param_aug_w_fast / self.param_aug_w_slow)
			if self.particles_to_randomize > 0.25:
				self.particles_to_randomize = 0.25
			for i in range(int(self.num_particles * self.particles_to_randomize)):
				random_point = int(np.random.uniform(0, len(self.map_states)-1))
				random_pos = self.map_states[random_point]
				random_or = rotateQuaternion(self.particlecloud.poses[0].orientation,np.random.uniform(0,2*math.pi))
				random_particles.append(Pose(random_pos,random_or))
			for idx in indexes:
				# Get position and orientation of particle
				pos_x, pos_y, pos_z = self.data[idx][0], self.data[idx][1], self.data[idx][2]
				or_x, or_y, or_z, or_w = self.data[idx][3], self.data[idx][4], self.data[idx][5], self.data[idx][6]
				new_idxs.append(idx)
				if len(random_particles) != 0 and np.random.random_sample() > (1 - self.particles_to_randomize):
					random_pose = random_particles.pop()
					poses.append(random_pose)
				else:
					new_pos = Point(np.random.normal(pos_x, self.random_noise_pos_update),\
									np.random.normal(pos_y, self.random_noise_pos_update),\
									pos_z)
					orientation = Quaternion(or_x, or_y, or_z, or_w)
					new_or = rotateQuaternion(orientation,np.random.normal(getHeading(orientation),self.random_noise_or_update_radians)-getHeading(orientation))
					poses.append(Pose(new_pos, new_or))
		elif not self.adaptive_mcl == "augmented":
			for idx in indexes:
				# Get position and orientation of particle
				if (self.weights[idx] < self.mean_weight - self.std_weight) and (self.weights.shape[0]-removed_count > self.min_particles):
					removed_count+=1
				else:
					new_idxs.append(idx)
					pos_x, pos_y, pos_z = self.data[idx][0], self.data[idx][1], self.data[idx][2]
					or_x, or_y, or_z, or_w = self.data[idx][3], self.data[idx][4], self.data[idx][5], self.data[idx][6]
					# Add small noise to the particles x and y dimension (2D robot)
					new_pos = Point(np.random.normal(pos_x, self.random_noise_pos_update),\
									np.random.normal(pos_y, self.random_noise_pos_update),\
									pos_z)

					# Convert quaternions to euler angles for ease of noise parameter tuning
					# Noise added only to yaw since robot has only 1 axis of rotation
					roll, pitch, yaw = euler_from_quaternion([or_x, or_y, or_z, or_w])
					yaw = np.deg2rad(np.random.normal(np.rad2deg(yaw), self.random_noise_or_update))

					# Convert angles back to quaternion form
					q = quaternion_from_euler(roll, pitch, yaw)
					new_q = Quaternion(q[0], q[1], q[2], q[3])
					poses.append(Pose(new_pos, new_q))

		# Update poses with new particles
		self.particlecloud.poses = copy.deepcopy(poses)
		self.data = np.array(self.data)[new_idxs]
		self.normalized_weights = np.array(self.normalized_weights)[new_idxs]
		# Recalculate weights with updated poses for weighted averaging
		if self.center_estimate_method == 'weighted':
			self.weights = self.calculate_weights(self.current_scan)
			self.data = np.array(self.data)

	def add_random_points(self, percentage=15):
		particles_to_add = percentage*0.01 * self.estimate_method.cluster_size
		for i in range(int(particles_to_add)):
			if len(self.particlecloud.poses)>=self.max_particles:
				break
			else:
				if (self.particle_init_method == 'uniform') or (self.particle_init_method=='normal'):
					new_pos = Point(np.random.uniform(0, self.sensor_model.map_width * self.sensor_model.map_resolution),\
									np.random.uniform(0, self.sensor_model.map_height * self.sensor_model.map_resolution),\
									0)
				elif self.particle_init_method == 'map':
					idx = np.random.randint(0, len(self.map_states))
					new_pos = self.map_states[idx]

				roll, pitch, yaw = 0, 0, np.deg2rad(np.random.uniform(0, self.random_noise_or_init))
				q = quaternion_from_euler(roll, pitch, yaw)
				new_q = Quaternion(q[0], q[1], q[2], q[3])
				self.particlecloud.poses.append(Pose(new_pos, new_q))

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
		if self.data.shape[0]>0:
			pose_ = self.estimate_method.update(self.data, self.normalized_weights)
			# print("STATS:", self.estimate_method.cur_stats)
		else:
			pose_ = [0,0,0,0,0,0,0]
		# Create Ros Pose data type
		new_pos = Point(pose_[0],pose_[1], pose_[2])
		new_or = Quaternion(pose_[3],pose_[4], pose_[5], pose_[6])
		pose = Pose(new_pos, new_or)
		if self.plot_graph:
			true_pos_x = self.last_odom_pose.pose.pose.position.x + self.sensor_model.map_origin_x
			true_pos_y = self.last_odom_pose.pose.pose.position.y + self.sensor_model.map_origin_y
			true_pos_z = self.last_odom_pose.pose.pose.position.z

			or_l1_error = abs(getHeading(new_or) - getHeading(self.last_odom_pose.pose.pose.orientation))
			l2_dist = self.l2_norm([pose_[0], pose_[1], pose_[2]], [true_pos_x, true_pos_x, true_pos_z])
			self.data_list.append([or_l1_error, l2_dist])
			if self.count%100 == 0:
				np.save("weighted.npy",np.array(self.data_list))
		if self.adaptive_mcl == "fixed_random_particle":
			self.add_random_points(percentage=self.random_pts_pct)
		return pose

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
	def __init__(self, method):
		self.method = method

	def update(self, data, weights=None):
		pose = None
		if self.method=='weighted':
			pose = self.estimate_pose_weighted(data, weights)
		elif self.method=='kmeans':
			pose = self.estimate_pose_kmeans(data, weights)
		return pose

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
		# new_pos = Point(pose[0],pose[1], pose[2])
		# new_or = Quaternion(pose[3],pose[4], pose[5], pose[6])
		# pose = Pose(new_pos, new_or)
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
