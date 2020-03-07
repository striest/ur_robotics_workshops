import os
import subprocess
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D

from kinematicchain import KinematicChain

class InverseKinematicsSolver:
	"""
	Class that performs inverse kinematics by gradient descent (kinda) on the error between end-effector and target point.
	"""

	def __init__(self, kinematic_chain, lr=1e-5, dt=0.01, dt_max=0.01, convergence = 1e-4):
		self.chain = kinematic_chain
		self.control_dim = len(self.chain.control_links)
		self.lr = lr
		self.dt = dt
		self.dt_max = dt_max
		self.convergence = convergence
		self.target = None

	def numeric_jacobian(self):
		"""
		Computes the Jacobian of the kinematic chain numerically. (i.e. perturb each control param and return the change in end effector position)
		Also, this is just for position, no orientation.
		J = de/dt
		de = J*dt
		dt = J_inv*de
		"""
		"""
		TODO: Implement this function according to the spec. Note that dt should be used as the perturbation.
		"""

	def set_target(self, target):
		"""
		Set a target point for the chain to reach. Expects some iterable containing 3 floats.
		"""
		self.target = np.array([target[0], target[1], target[2]])

	def sample_target(self, bounds = {'x':(-5, 5), 'y':(-5, 5), 'z':(0, 10)}):
		self.target = np.array([np.random.uniform(*bounds['x']), np.random.uniform(*bounds['y']), np.random.uniform(*bounds['z'])])

	def error(self):
		return (self.target - self.chain.end_effector_position()).T

	def step(self):
		"""
		TODO: implement one step of the Jacobian-based update method. This involves getting the Jacobian of the current config, inverting it,
		and using the J^-1 and the error vector to compute a control step.
		"""

	def converged(self):
		return np.sum(self.error() ** 2) ** 0.5 < self.convergence

	def make_video(self, render_every=5, resample_every = 500, steps = 5000):
		frames = []
		subprocess.call(['mkdir', 'video'])
		cnt = 0
		for i in range(steps):
			if cnt % resample_every == 0 or self.converged():
				print('resample')
				self.sample_target()
				cnt = 0
			self.step()
			print('Frame', i, end='\r\r')
			if i % render_every == 0:
				self.render()
				plt.savefig('video/frame{:05d}.png'.format(i//render_every))
			cnt += 1

		os.chdir("video")
		subprocess.call(['ffmpeg', '-framerate', '100', '-i', 'frame%05d.png', '-pix_fmt', 'yuv420p', '-vcodec', 'libx264', '../video.mp4'])
		os.chdir('../')
		subprocess.call(['rm', '-r', 'video'])
			
	def render(self, bounds = {'x':(-5, 5), 'y':(-5, 5), 'z':(0, 10)}):
		fig, ax = self.chain.render()
		ax.scatter(self.target[0], self.target[1], self.target[2], c='r', marker='x', label='target (x={:.2f}, y={:.2f},z={:.2f})'.format(self.target[0], self.target[1], self.target[2]))

		ax.set_xlim(bounds['x'])
		ax.set_ylim(bounds['y'])
		ax.set_zlim(bounds['z'])

		plt.legend()

		return fig, ax

def pseudo_inverse(J):
	"""
	Invert non-square matrices. J_inv = (J_T*J)_inv * J_T
	"""
	return np.dot(np.linalg.inv(np.dot(J.T, J)), J.T)
