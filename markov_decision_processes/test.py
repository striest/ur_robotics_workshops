import numpy as np

from env import Env
from slippery_env import SlipperyEnv

from policies import ArgmaxQPolicy
from value_iteration import ValueIteration


n = 15
m = 6
env = SlipperyEnv(n, m, max_steps = 2 * n)
#env = Env(n, m, max_steps = 2 * n)

policy = ArgmaxQPolicy(env)

o = env.reset()

env.state = np.array([1, 1])
env.goal = np.array([13, 1])
env.obstacles[3:12, [0, 2, 3]] = 1


"""
env = Env(3, 3, 10)
policy = ArgmaxQPolicy(env)
o = env.reset()
env.state = np.array([0, 0])
env.goal = np.array([2, 2])
"""

env.obstacles[1, 1] = 1

ValueIteration.value_iteration(env, policy, discount = 0.97, v_itrs=50, n_samples=100)
