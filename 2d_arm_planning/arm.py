import numpy as np
from math import sin, cos, pi

class Arm:
	"""
	2D arm class
	"""
	def __init__(self, link_lengths):
		self.links = np.array(link_lengths)
		self.joints = [pi/2]
		self.joints.extend([0] * (len(self.links)-1))
		self.joints = np.array(self.joints)

	def update_joints(self, joint_update):
		"""
		Expect a list of joint angles
		"""
		for i in range(len(joint_update)):
			if i > len(self.joints) - 1:
				break
			self.joints[i+1] = joint_update[i]

	@property
	def config_point(self):
		return np.array(self.joints[1:])

	@property
	def joint_locations(self):
		return self.joint_locations_from(self.config_point)

	@property
	def end_effector(self):
		return self.joint_locations[-1]

	def joint_locations_from(self, angs):
		locs = np.zeros((1 ,3)) #x, y, theta
		for link, ang in zip(self.links, np.append(self.joints[0], angs)):
			state = locs[-1]
			th_new = state[2] + ang
			x_new = state[0] + link * cos(th_new)
			y_new = state[1] + link * sin(th_new)
			locs = np.append(locs, np.array([[x_new, y_new, th_new]]), axis=0)
		return locs
		
	def end_effector_from(self, angs):
		return self.joint_locations_from(angs)[-1]

	def __repr__(self):
		return 'Links = {}'.format(self.links) + '\nJoints = {}'.format(self.joints)

if __name__ == '__main__':
	arm = Arm([1, 2, 3])
	arm.joints[1] = 0.3
	arm.joints[2] = -0.3
	print(arm)
	print(arm.joint_locations)
	print(arm.joint_locations_from(np.array([0.2, 0.3])))
	print(arm.end_effector_from(np.array([0.2, 0.3])))
	arm.update_joints(np.array([0.2, 0.3]))
	print(arm.joint_locations)
	print(arm.end_effector)
