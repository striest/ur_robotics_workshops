import numpy as np
import matplotlib.pyplot as plt

class ValueEstimation:
	"""
	Collection of functions for doing value estimation for gridworld MDPs.
	"""
	def compute_V(env, policy, discount = 1.0, n_itrs = 3, n_samples=10):
		V = np.zeros((env.n, env.m))

		for itr in range(n_itrs):
			print('ITR {}/{}'.format(itr, n_itrs), end='\r')
			for i in range(V.shape[0]):
				for j in range(V.shape[1]):
#					import pdb;pdb.set_trace()
					state = np.array([i, j])
					act_dist = policy.forward(state)
					v_max = -float('inf')
					for a in range(len(act_dist)):
						v_acc = 0.0
						for _ in range(n_samples):
							s_new, r, t, _ = env.step_from(a, state)
							v_acc += r + discount * V[s_new[0], s_new[1]] * ~t
						v_max = max(v_acc, v_max)
					V[i, j] = discount * (v_max / n_samples)
					if env.terminal_from(state):
						V[i, j] = 0.0
					

		return V

	def compute_Q(env, policy, discount, n_itrs = 3, n_samples=10):
		"""
		Q(s, a) = r(s, a, s') + gamma * E[V(s')]
		"""	
		V = ValueEstimation.compute_V(env, policy, discount, n_itrs, n_samples)
		Q = np.zeros((V.shape[0], V.shape[1], env.action_space.shape[0]))

		for i in range(Q.shape[0]):
			for j in range(Q.shape[1]):
				for a in range(Q.shape[2]):
					for _ in range(n_samples):
						state = np.array([i, j])
						s_new, r, t, _ = env.step_from(a, state)
						Q[i, j, a] += r + discount * V[s_new[0], s_new[1]]
					Q[i, j, a] /= n_samples

		return Q, V

class ValueIteration:
	"""
	Perform value iteration
	"""
	def value_iteration(env, policy, discount, v_itrs, n_samples):
		Q, V = ValueEstimation.compute_Q(env, policy, discount, v_itrs, n_samples)
		policy.Q = Q

		fig, ax = env.render()
		fig, ax = env.render_policy(policy, fig, ax)
		fig, ax = env.render_reward(V, True, fig, ax)
		plt.show()
		
