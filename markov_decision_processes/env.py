import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches

class Env:
	"""
	Basic gridworld environment for implementing MDP stuff
	"""
	def __init__(self, n = None, m = None, fp = None, max_steps = 20):
		self.goal = np.zeros(2).astype(int)
		self.state = np.zeros(2).astype(int)
		self.random_goal = True
		if n and m and max_steps:
			self.obstacles = np.zeros((n, m)).astype(int)
			self.n = n
			self.m = m

		self.action_map = {
			0:np.array([0, 1]),
			1:np.array([1, 0]),
			2:np.array([0, -1]),
			3:np.array([-1, 0]),

		}

		self.max_steps = max_steps
		self.steps = 0

		if fp:
			self.load_from_fp(fp)

	def load_from_fp(self, fp):
		self.obstacles = []
		for line in open(fp):
			arr = []
			for c in line:
				if c == '.':
					arr.append(0)
				elif c == 'X':	
					arr.append(1)
				elif c == 'G':
					arr.append(2)
			arr = np.array(arr)
			self.obstacles.append(arr)
			print(arr)

		self.obstacles = np.stack(self.obstacles)
		self.obstacles = self.obstacles.transpose()
		self.obstacles = np.flip(self.obstacles, axis=1)

		self.goal = np.argwhere(self.obstacles == 2)

		if self.goal.shape[0] != 0:
			self.goal = self.goal[0]
			self.random_goal = False

		self.obstacles[self.obstacles == 2] = 0
		self.n = self.obstacles.shape[0]
		self.m = self.obstacles.shape[1]


	def reset(self):
		free_space = np.argwhere(self.obstacles == 0)
		idx = np.random.choice(len(free_space))
		self.state = free_space[idx]

		if self.random_goal:
			idx = np.random.choice(len(free_space))
			self.goal = free_space[idx]

		return self.state

	def step(self, action):
		assert action in self.action_map, 'invalid action {}'.format(action)
		s, r, t, i = self.step_from(action, self.state)
		self.state = s
		self.steps += 1

		return self.state, self.reward, self.terminal, i

	def step_from(self, action, state):
		assert action in self.action_map, 'invalid action {}'.format(action)
		act = self.action_map[action]
		s_new = state + act
		s_new[0] = np.clip(s_new[0], 0, self.n-1)
		s_new[1] = np.clip(s_new[1], 0, self.m-1)
		reward = self.reward_from(s_new)
		terminal = self.terminal_from(s_new)

		return s_new, reward, terminal, {'prob':1.0}

	@property
	def action_space(self):
		return np.zeros(len(self.action_map.keys()))

	@property
	def observation_space(self):
		return np.zeros(2)

	@property
	def reward(self):
		return self.reward_from(self.state)

	def reward_from(self, state):	
		if all(state == self.goal):
			return 10 * np.ones(1)
		elif self.obstacles[state[0], state[1]] == 1:
			return -10 * np.ones(1)
		else:
			return np.zeros(1)

	@property
	def terminal(self):
		t_term = self.steps >= self.max_steps
		g_term = all(self.state == self.goal)
		o_term = self.obstacles[self.state[0], self.state[1]] == 1
		return t_term or g_term or o_term

	def terminal_from(self, state):
		"""
		NOTE: Will not keep track of nsteps.
		"""	
		g_term = all(state == self.goal)
		o_term = self.obstacles[state[0], state[1]] == 1
		return g_term or o_term

	def render(self, fig=None, ax=None):
		if fig is None or ax is None:
			fig, ax = self.base_render()

		for i in np.arange(0, self.n):
			for j in np.arange(0, self.m):
				color = 'none'
				pos = np.array([i, j])
				if self.obstacles[i, j] == 1:
					color = 'k'
				if all(self.goal == pos):
					color = 'y'
				if all(self.state == pos):
					color = 'b'

				rect = patches.Rectangle((i, j), 1, 1, linewidth=1, color=color)
				ax.add_patch(rect)
		return fig, ax

	def render_reward(self, rew_map = None, show_values = True, fig = None, ax = None):
		"""
		Creates a render of the reward for each state in the gridworld. Can optionally provide own rew map (for V fuctions, etc.)
		"""
		if fig is None or ax is None:
			fig, ax = self.base_render()

		if rew_map is None:
			rew_map = np.zeros(self.obstacles.shape)
			for i in range(rew_map.shape[0]):
				for j in range(rew_map.shape[1]):
					rew_map[i, j] = self.state_reward(np.array([i, j]))

		min_rew = np.min(rew_map)
		max_rew = np.max(rew_map)
		pos_c = 'g'
		neg_c = 'r'
		print('Q min/max = {}/{}'.format(min_rew, max_rew))
		for i in range(rew_map.shape[0]):
			for j in range(rew_map.shape[1]):
				r = rew_map[i, j]
				if r < 0:
					c = 'r'
					a = r/min_rew
				else:
					c = 'g'
					a = r/max_rew

				rect = patches.Rectangle((i, j), 1, 1, linewidth=1, color=c, alpha=a)
				ax.add_patch(rect)
				if show_values:
					ax.text(i + 0.5, j + 0.5, '{:.2f}'.format(r), horizontalalignment='center', color = 'k' if self.obstacles[i, j] == 0 else 'w')
		return fig, ax

	def render_policy(self, policy, fig = None, ax = None):
		if fig is None or ax is None:
			fig, ax = self.base_render()

		for i in np.arange(0, self.n):
			for j in np.arange(0, self.m):
				state = np.array([i, j])
				act_dist = policy.forward(state)
				o_x = i + 0.5
				o_y = j + 0.5
				for a in range(act_dist.shape[0]):
					if act_dist[a] < 0.01:
						continue
					dx = act_dist[a] * 0.35 * self.action_map[a][0]
					dy = act_dist[a] * 0.35 * self.action_map[a][1]
					ax.arrow(o_x, o_y, dx, dy, color='y', head_width = 0.1)

		return fig, ax

	def base_render(self, cell_size=0.7, fig = None, ax = None):
	
		if fig is None or ax is None:
			w = cell_size * self.n
			h = cell_size * self.m
			maxlen = max(w, h)
			if maxlen > 10:
				w *= 10/maxlen
				h *= 10/maxlen


			fig, ax = plt.subplots(figsize=(w, h))

		for i in np.arange(0, self.n + 1):
			ax.axvline(i, c='k', ls = '-' if i in (0, self.n) else '--')

		for i in np.arange(0, self.m + 1):
			ax.axhline(i, c='k', ls = '-' if i in (0, self.m) else '--')

		return fig, ax

if __name__ == '__main__':
	env = Env(n=10, max_steps=10)
	env.obstacles = np.random.choice(2, size = (10, 10), p = [0.7, 0.3])
	env.reset()
	t = False

	while not t:
		env.render()
		plt.show()
		o, r, t, i = env.step(np.random.choice(4))
		print(o, r, t, i)

	env.render_reward()
	plt.show()
