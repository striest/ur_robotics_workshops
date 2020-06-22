import numpy as np
from numpy import pi

class Link:
	"""
	Basic class for a single link in a 2-dimensional arm. Parameterized as a rotation, then translation.

	Contains the following fields:
	Length: the length of the link
	Angle: the current angle of the link
	Min/max angle: limits on the rotation this link allows.
	"""
	def __init__(self, length, min_angle=0.0, max_angle=2*pi, angle = None):
		assert (angle >= min_angle) and (angle <= max_angle), 'angle not within joint bounds'
		self.length = length
		self.min_angle = min_angle
		self.max_angle = max_angle

		if angle:
			self.angle = angle
		else:
			self.angle = (min_angle + max_angle)/2

	def set_angle(self, angle):
		"""
		Sets the angle to the angle specified, within the joint limits of the arm.
		"""
		self.angle = np.clip(angle, self.min_angle, self.max_angle)

	def __repr__(self):
		return 'Link length = {:.2f}, link angle = {:.2f} min/max angle = {}/{}'.format(self.length, self.angle, self.min_angle, self.max_angle)

class FixedLink:
	"""
	Basic link class that has no degrees of freedom.
	"""
	def __init__(self, length, angle):
		self.length = length
		self.angle = angle

	def __repr__(self):
		return 'Fixed link length = {:.2f}, link angle = {:.2f}'.format(self.length, self.angle)
	
