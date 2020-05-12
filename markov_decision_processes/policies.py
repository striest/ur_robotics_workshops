import abc
import numpy as np; np.set_printoptions(suppress=True, linewidth=100000, precision=2)
import matplotlib.pyplot as plt

from env import Env

class Policy(abc.ABC):
	"""
	Class for designing policies on gridworlds.
	"""
	@abc.abstractmethod
	def forward(self, obs):
		"""
		Return a distribution over the action space, given observation.
		"""
		pass

	def action(self, obs, deterministic = False):
		"""
		Given an observation of the world, take some action.
		"""
		dist = self.forward(obs)
		if deterministic:
			act = np.argmax(dist)
		else:
			act = np.random.choice(dist.shape[0], p=dist)

		return act

class RandomPolicy(Policy):
	"""
	Take a random action uniformly from the action space
	"""
	def __init__(self, env):
		self.n_acts = env.action_space.shape[0]

	def forward(self, obs):
		return np.ones(self.n_acts) / self.n_acts

class ToGoalPolicy(Policy):
	"""
	Greedily move to the goal
	"""
	def __init__(self, env):
		self.n_acts = env.action_space.shape[0]
		self.goal = env.goal
		self.action_map = env.action_map

	def forward(self, obs):
		dist = np.zeros(4)
		to_goal = self.goal - obs		

		if np.sum(to_goal) == 0:
			return np.ones(self.n_acts)/self.n_acts

		val = np.stack([np.sum(self.action_map[a] * to_goal) for a in range(self.n_acts)])
		act = np.argmax(val)
		dist[act] = 1
		return dist

class ArgmaxQPolicy(Policy):
	"""
	Policy that steps in the direction that maximizes Q.
	"""

	def __init__(self, env, Q = None):
		self.n_acts = env.action_space.shape[0]
		if Q is None:
			Q = np.zeros((env.n, env.m, self.n_acts))
		self.Q = Q

	def forward(self, obs):
		dist = np.zeros(self.n_acts)
		Q_s = self.Q[obs[0], obs[1], :]
		act = np.argmax(Q_s)
		dist[act] = 1
		return dist

class EpsilonGreedyPolicy(ArgmaxQPolicy):
	"""
	Argmax Q policy with an epsilon chance of a random action
	"""
	def __init__(self, env, Q = None, e = 0.05):
		super().__init__(env, Q)
		self.e = e

	def forward(self, obs):
		dist = super().forward(obs)
		if np.random.random() < self.e:
			dist = np.ones(dist.shape) * (1/self.n_acts)

		return dist
		
class TabularPolicy(Policy):
	"""
	Generic policy that can take arbitrary actions
	"""
	def __init__(self, env):
		self.n_acts = env.action_space.shape[0]
		self.policy = np.zeros((env.n, env.m, self.n_acts))
		self.policy[:, :, 0] = 1.0

	def forward(self, obs):
		return self.policy[obs[0], obs[1]]

if __name__ == '__main__':
	n = 20
	env = Env(n=n, max_steps = 2 * n)
	policy = ArgmaxQPolicy(env)
	env.obstacles = np.random.choice(2, size = (n, n), p = [0.75, 0.25])
	o = env.reset()

	ValueIteration.value_iteration(env, policy, discount = 0.97, n_itrs = 100)
