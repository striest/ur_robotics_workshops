import numpy as np
from numpy import sin, cos, pi

from link import Link, FixedLink

class Arm:
	"""
	2D arm class that is comprised of a list of links. (And exactly two degrees of freedom for plotting).
	"""
	def __init__(self, link_list):
		self.links = []
		self.control_links = []
		for link in link_list:
			self.links.append(link)
			if isinstance(link, Link):
				self.control_links.append(link)

	def set_joint_space(self, angles):
		assert len(angles) == len(self.control_links), 'Tried to assign {} angles to a {}DOF arm'.format(len(angles), len(self.control_links))
		for angle, link in zip(angles, self.control_links):
			link.set_angle(angle)

	def get_joint_space(self):
		return np.array([link.angle for link in self.control_links])

	def get_joint_poses(self):
		poses = [np.zeros(3)]
		for link in self.links:
			### IMPLEMENT YOUR CODE HERE ###
		return np.stack(poses, axis=0)

	def get_end_effector_pose(self):
		return self.get_joint_poses()[-1]

	def get_joint_poses_from(self, angles):
		old_angles = self.get_joint_space()
		self.set_joint_space(angles)
		out = self.get_joint_poses()
		self.set_joint_space(old_angles)
		return out

	def get_end_effector_pose_from(self, angles):
		return self.get_joint_poses_from(angles)[-1]

	def __repr__(self):
		out = ''
		for link in self.links:
			out += str(link) + '\n'
		return out
