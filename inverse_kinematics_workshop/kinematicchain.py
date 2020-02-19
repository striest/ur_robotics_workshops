import numpy as np
from math import pi
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class KinematicChain:
	"""
	Basically a collection of homogenous transform matrices to compute position and orientation of an end-effector in joint space.
	"""
	def __init__(self, links):
		"""
		Expects some collection of links in order.
		"""
		self.links = []
		self.control_links = []
		for link in links:
			self.append_link(link)

	def append_link(self, link):
		"""
		Adds a new link to the end of the chain.
		"""
		self.links.append(link)

		if not (type(link) is FixedLink):
			self.control_links.append(link)

	def update_control(self, controls):
		"""
		Assigns the thetas in controls to each link in the chain.
		"""
		assert len(controls) == len(self.control_links), 'Length of control vector doesn\'t match the length of the chain. Expected {} control inputs.'.format(len(self.control_links))
		for link, t in zip(self.control_links, controls):
			link.theta = t

	def compute_htms(self):
		"""
		Computes the HTMs from origin to the position/orientation of each joint.
		Returns as a 3D tensor of dim [n_links+1 x 4 x 4], where the HTMS[i] is the transform from base to joint i.
		"""
		base = np.eye(4)
		htms = [base]
		for link in self.links:
			htms.append(np.dot(htms[-1], link.htm()))
		return np.stack(htms, axis=0)

	def end_effector_position(self):
		"""
		Gives the position of the end-effector as a 3x1 array containing [x_e, y_e, z_e]
		"""
		return self.compute_htms()[-1, 0:3, 3]

	@property
	def control(self):
		controls = np.array([link.theta for link in self.control_links])
		return controls

	def render(self, fig = None, ax = None):
		if fig is None and ax is None:
			fig = plt.figure()
			ax = fig.add_subplot(111, projection='3d')
		htms = self.compute_htms()
		ax.plot(htms[:, 0, 3], htms[:, 1, 3], htms[:, 2, 3])
		ax.set_xlabel('X')
		ax.set_ylabel('Y')
		ax.set_zlabel('Z')

		ax.scatter(htms[-1, 0, 3], htms[-1, 1, 3], htms[-1, 2, 3], c='g', label='end-effector (x={:.2f}, y={:.2f},z={:.2f})'.format(htms[-1, 0, 3], htms[-1, 1, 3], htms[-1, 2, 3]))

		return fig, ax

class Link:
	"""
	One element in a kinematic chain. We can parameterize self as a homogenous transform matrix.
	Note: It's a bit out-of-scope to do all types of joints, so we'll restrict all of our joints to be revolute.
	"""
	def __init__(self, a, alpha, d):
		"""
		Initialize a link with fixed alpha, a, d (DH params). Theta is our control, so it will vary.
		"""
		self.alpha = alpha
		self.a = a
		self.d = d
		self.theta = 0

	def htm(self):
		"""
		Induces the HTM for this link (i.e. T(i-1, i)).
		"""
		htm = np.zeros((4, 4))
		htm[0, 0] = np.cos(self.theta)
		htm[0, 1] = -np.sin(self.theta) * np.cos(self.alpha)
		htm[0, 2] = np.sin(self.theta) * np.sin(self.alpha)
		htm[0, 3] = self.a * np.cos(self.theta)
		htm[1, 0] = np.sin(self.theta)
		htm[1, 1] = np.cos(self.theta) * np.cos(self.alpha)
		htm[1, 2] = -np.cos(self.theta) * np.sin(self.alpha)
		htm[1, 3] = self.a * np.sin(self.theta)
		htm[2, 1] = np.sin(self.alpha)
		htm[2, 2] = np.cos(self.alpha)
		htm[2, 3] = self.d
		htm[3, 3] = 1

		return htm

	def __repr__(self):
		return "a = {}, alpha = {}, d = {}, theta = {}".format(self.a, self.alpha, self.d, self.theta)

class FixedLink(Link):
	"""
	A link that doesn't have a degree of freedom on its z rotation.
	"""
	def __init__(self, a, alpha, d, theta):
		super(FixedLink, self).__init__(a, alpha, d)
		self.theta = theta

