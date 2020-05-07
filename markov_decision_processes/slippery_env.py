import random
import numpy as np

from env import Env

class SlipperyEnv(Env):
	"""
	Environment with 'slippery' actions. There's a random chance you will move perpendicular to what you expect.
	"""

	def __init__(self, n, m, max_steps, slip_chance = 0.2):
		super().__init__(n, m, max_steps)
		self.slip_chance = slip_chance

	def step_from(self, action, state):
		assert action in self.action_map, 'invalid action {}'.format(action)
		old_act = self.action_map[action]

		if action == 4:
			return super().step_from(action, state)
		

		if random.random() < self.slip_chance:
			maxidx = np.argmax(np.abs(old_act))
			if random.random() > 0.5:
				slip = 1
			else:
				slip = -1

			act = old_act + slip
			act[maxidx] = 0
		else:
			act = old_act


		if all(act == old_act):
			prob = 1 - self.slip_chance
		else:
			prob = self.slip_chance/2


		s_new = state + act
		s_new[0] = np.clip(s_new[0], 0, self.n-1)
		s_new[1] = np.clip(s_new[1], 0, self.m-1)

		rew = self.reward_from(s_new)
		term = self.terminal_from(s_new)

		return s_new, rew, term, {'prob':prob}
