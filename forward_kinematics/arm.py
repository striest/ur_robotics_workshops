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
		"""
		Sets the joints of the arm (from bottom to top) to the angles specified in the argument. Note that the number of angles must equal the number of joints in the arm.

		Args:
			angles: an iterable (such as a list or array) of angles

		Returns:
			nothing
		"""
		assert len(angles) == len(self.control_links), 'Tried to assign {} angles to a {}DOF arm'.format(len(angles), len(self.control_links))
		for angle, link in zip(angles, self.control_links):
			link.set_angle(angle)

	def get_joint_space(self):
		"""
		Gets the current joint angles of the arm

		Args:
			nothing

		Returns:
			A numpy array of all the joint angles of the arm
		"""
		return np.array([link.angle for link in self.control_links])

	def get_joint_poses(self):
		"""
		Uses the current joint angles to return the poses of ALL the links in the arm. This function should be implemented as a part of the forward kinematics workshop.

		Args:
			nothing

		Returns:
			A (n+1)x3 numpy array where each row is the pose (x, y, theta) of the origin of the n-th link of the arm. The (n+1)-th row is the pose of the end-effector.
		"""
		#WORKSHOP IMPLEMENTATION HERE
		poses = [np.zeros(3)]
		for link in self.links:
			#Implementation here
			prev = poses[-1]
			x = prev[0]
			y = prev[1]
			th = prev[2]
			th_new = th + link.angle
			x_new = x + link.length*cos(th_new)
			y_new = y + link.length*sin(th_new)
			curr = np.array([x_new, y_new, th_new])
			poses.append(curr)
			#End implementation
		return np.stack(poses, axis=0)

	def get_end_effector_pose(self):
		"""
		Uses the current joint angles to return the pose of just the end-effector.

		Args:
			nothing

		Returns:
			A 1D numpy array containing the pose of the end-effector (x, y, theta)
		"""
		return self.get_joint_poses()[-1]

	def get_joint_poses_from(self, angles):
		"""
		Computes the poses of all the links of the arm (and end-effector) from the joint angles in the argument.

		Args:
			angles: The list of joint angles to compute pose from.

		Returns:
			A (n+1)x3 numpy array where each row is the pose (x, y, theta) of the orig    in of the n-th link of the arm. The (n+1)-th row is the pose of the end-effector. (Equivalent to get_joint_poses)
		"""
		old_angles = self.get_joint_space()
		self.set_joint_space(angles)
		out = self.get_joint_poses()
		self.set_joint_space(old_angles)
		return out

	def get_end_effector_pose_from(self, angles):
		"""
		Uses the current joint angles to return the pose of just the end-effector.

		Args:
			angles: The list of joint angles to compute pose from.

		Returns:
			A 1D numpy array containing the pose of the end-effector (x, y, theta)
		"""
		return self.get_joint_poses_from(angles)[-1]

	def __repr__(self):
		out = ''
		for link in self.links:
			out += str(link) + '\n'
		return out
