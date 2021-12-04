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

class LocalizationTechniques():
	def __init__(self, config, sensor_model, occupancy_map):
		self.config = config
		self.sensor_model = sensor_model
		self.occupancy_map = occupancy_map
		# Particle initialization Parameters
		self.noise_pos_init, self.noise_or_init = self.config['noise_pos_init'], self.config['noise_or_init']		# Noise added to position during init
		self.noise_pos_update, self.noise_or_update = self.config['noise_pos_update'], self.config['noise_or_update'] # Noise added to position during update
		self.num_particles = self.config['num_particles']           											# Total Number of particles initialized
		self.min_particles, self.max_particles = self.config['min_particles'], self.config['max_particles']           # Min and max number of particles during update
		self.particle_init_method = self.config['particle_init_method']											# Method of initialization of particles
		self.mcl_technique = self.config['mcl_technique']															# Technique of localization to run
		self.random_pts_pct = self.config['random_pts_pct']														# Percentage of random points to add
		# self.error_count = 0
		self.sensor_model.count = 0
		if self.mcl_technique == 'augmented':
			self.param_aug_w_slow = 0
			self.param_aug_w_fast = 0
			self.param_aug_w_avg = 0
			self.param_aug_alpha_slow = self.config['augmented_mcl_aug_alpha_slow']
			self.param_aug_alpha_fast = self.config['augmented_mcl_aug_alpha_fast']
			self.particles_to_randomize = 0

	def return_state_from_pose(self, pose):
		pos, orientation = pose.position, pose.orientation
		pos_x, pos_y, pos_z = pos.x, pos.y, pos.z
		or_x, or_y, or_z, or_w = orientation.x, orientation.y, orientation.z, orientation.w
		return [pos_x, pos_y, pos_z, or_x, or_y, or_z, or_w]

	def create_map_states(self):
		"""Create list of set of points that lie in the map"""
		index = 0
		self.map_states = []
		for h in range(0, self.sensor_model.map_height):
			for w in range(0, self.sensor_model.map_width):
				if (self.sensor_model.map_data[index] >= 0.0) :
					p = Point()
					p.x = w * self.sensor_model.map_resolution + self.occupancy_map.info.origin.position.x
					p.y = h * self.sensor_model.map_resolution + self.occupancy_map.info.origin.position.y
					p.z = 0
					self.map_states.append(p)
				index += 1

	def initialize_point_cloud(self, initialpose):
		self.create_map_states()						# Create map states
		pose = PoseArray()								# Create pose array
		pose.header = initialpose.header				# Initialize header from received input
		pos = initialpose.pose.pose.position			# Get position
		orientation = initialpose.pose.pose.orientation	# Get Orientation
		pos_x, pos_y, pos_z = pos.x, pos.y, pos.z
		or_x, or_y, or_z, or_w = orientation.x, orientation.y, orientation.z, orientation.w
		for i in range(self.num_particles):
			if self.particle_init_method=='normal':		# Initialize particles position  using normal distribution
					new_pos = Point(np.random.normal(pos_x, self.noise_pos_init),\
									np.random.normal(pos_y, self.noise_pos_init),\
									pos_z)
			elif self.particle_init_method=='uniform': 	# Initialize particles position  using uniform distribution
					new_pos = Point(np.random.uniform(0, self.sensor_model.map_width * self.sensor_model.map_resolution),\
									np.random.uniform(0, self.sensor_model.map_height * self.sensor_model.map_resolution),\
									pos_z)
			elif self.particle_init_method=='map': 		# Initialize particles position using map states
				idx = np.random.randint(0, len(self.map_states))
				new_pos = self.map_states[idx]

			roll, pitch, yaw = 0, 0, np.deg2rad(np.random.uniform(0, self.noise_or_init)) # Initialize particles orientation using map states
			q = quaternion_from_euler(roll, pitch, yaw)									  # Convert from euler to quarternion
			new_q = Quaternion(q[0], q[1], q[2], q[3])
			pose.poses.append(Pose(new_pos, new_q))
		return pose

	def generate_random_poses(self, num_particles):
		random_particles = []
		for i in range(int(num_particles)):
			idx = np.random.randint(0, len(self.map_states))
			new_pos = self.map_states[idx]
			roll, pitch, yaw = 0, 0, np.deg2rad(np.random.uniform(0, self.noise_or_init))
			q = quaternion_from_euler(roll, pitch, yaw)
			new_q = Quaternion(q[0], q[1], q[2], q[3])
			random_particles.append(Pose(new_pos, new_q))
		return np.array(random_particles)

	def create_random_particles(self, poses):
		if self.mcl_technique == 'augmented':
			self.param_aug_w_slow += self.param_aug_alpha_slow * (self.param_aug_w_avg - self.param_aug_w_slow)
			self.param_aug_w_fast += self.param_aug_alpha_fast * (self.param_aug_w_avg - self.param_aug_w_fast)
			self.particles_to_randomize = max(0, 1.0 - self.param_aug_w_fast / self.param_aug_w_slow)
			if self.particles_to_randomize > (self.random_pts_pct/100):
				self.particles_to_randomize = self.random_pts_pct/100
			num_particles_to_add = int(len(poses) * self.particles_to_randomize)
			print("RANDOM_PARTICLES: ", num_particles_to_add)
			random_particles = self.generate_random_poses(num_particles_to_add)
			if random_particles.shape[0]>0:
				poses = np.concatenate((poses[:-num_particles_to_add], random_particles), axis=0)

		elif self.mcl_technique == 'fixed_random_particle':
			if len(poses) < self.max_particles:
				self.particles_to_randomize = self.random_pts_pct/100
				num_particles_to_add = len(poses) * self.particles_to_randomize
				num_particles_to_add = min(int(len(poses) * self.particles_to_randomize), self.max_particles - len(poses))	# Do not exceed max particles
				random_particles = self.generate_random_poses(num_particles_to_add)
				poses = np.concatenate((poses, random_particles), axis=0)
		return poses

	def update_particle_cloud(self, weights, indexes, particlecloud):
		self.weights, self.indexes, self.particlecloud = weights, indexes, particlecloud
		updated_poses, updated_data, resampled_data, resampled_weights = [], [], [], []
		for idx in self.indexes:
			pos_x, pos_y, pos_z, or_x, or_y, or_z, or_w = self.return_state_from_pose(self.particlecloud.poses[idx])
			new_pos = Point(np.random.normal(pos_x, self.noise_pos_update),\
							np.random.normal(pos_y, self.noise_pos_update),\
							pos_z)
			roll, pitch, yaw = euler_from_quaternion([or_x, or_y, or_z, or_w])
			yaw = np.deg2rad(np.random.normal(np.rad2deg(yaw), self.noise_or_update))	# Add only noise to yaw (2D)
			q = quaternion_from_euler(roll, pitch, yaw)
			new_q = Quaternion(q[0], q[1], q[2], q[3])
			updated_pose = Pose(new_pos, new_q)
			resampled_data.append([pos_x, pos_y, pos_z, or_x, or_y, or_z, or_w])
			resampled_weights.append(self.weights[idx])
			updated_data.append(self.return_state_from_pose(updated_pose))
			updated_poses.append(updated_pose)
		if (self.mcl_technique=='augmented'):
			# print(resampled_weights)
			resampled_weights /= max(resampled_weights)		# Normalize the weights to use for estimated pose
			sorted_ids = resampled_weights.argsort()[::-1]		# Sort the data according to maximum weights
			updated_poses = np.array(updated_poses)[sorted_ids]			# Sort the data according to maximum weights
			updated_data = np.array(updated_data)[sorted_ids]				# Sort the data according to maximum weights
			resampled_data = np.array(resampled_data)[sorted_ids]			# Sort the data according to maximum weights
			resampled_weights = np.array(resampled_weights)[sorted_ids]	# Sort the data according to maximum weights

		if (self.mcl_technique=='fixed_random_particle') and (len(updated_poses) > self.max_particles):	# Remove extra particles
			mean, std = np.mean(resampled_weights), np.std(resampled_weights)
			ids_to_keep = np.where(resampled_weights >= mean-std)
			updated_poses = np.array(updated_poses)[ids_to_keep]
			updated_data = np.array(updated_data)[ids_to_keep]
			resampled_data = np.array(resampled_data)[ids_to_keep]
			resampled_weights = np.array(resampled_weights)[ids_to_keep]
			resampled_weights /= max(resampled_weights)

		return np.array(updated_poses), np.array(updated_data), np.array(resampled_data), np.array(resampled_weights)
