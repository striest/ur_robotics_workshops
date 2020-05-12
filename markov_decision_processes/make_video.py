import numpy as np
import matplotlib.pyplot as plt
import os
import subprocess
import argparse

from policies import ArgmaxQPolicy
from value_iteration import ValueIteration
from env import Env
from slippery_env import SlipperyEnv

parser = argparse.ArgumentParser(description='Parse videomaker params')
parser.add_argument('--framerate', type=int, required=False, default=10, help='framerate of the output video')
parser.add_argument('--deterministic', type=bool, required=False, default=True, help='determines whether the policy takes deterministic actions')
parser.add_argument('--n_runs', type=int, required=False, default=1, help='number of videos to make')
parser.add_argument('--slip', type=bool, required=False, default=False, help='add a chance of \'slipping\'')
parser.add_argument('--slip_chance', type=float, required=False, default=0.1, help='chance of slipping')
parser.add_argument('--maze_fp', type=str, required=True, help='path to the maze file')
parser.add_argument('--max_steps', type=int, required=False, default=100, help='number of steps to run before terminating')
parser.add_argument('--discount', type=float, required=False, default=0.95, help='the discount factor for the env')
parser.add_argument('--itrs', type=int, required=False, default=100, help='the amount of iterations of value iteration')
parser.add_argument('--samples', type=int, required=False, default=50, help='the number of state tranisitions to sample at each state for value iteration')

args = parser.parse_args()

print(args)

if args.slip:
	env = SlipperyEnv(fp = args.maze_fp, slip_chance = args.slip_chance, max_steps=args.max_steps)
else:
	env = Env(fp=args.maze_fp, max_steps=args.max_steps)

policy = ArgmaxQPolicy(env)


print('running value iteration...')
ValueIteration.value_iteration(env, policy, discount = args.discount, v_itrs=args.itrs, n_samples=args.samples)

for run in range(args.n_runs):
	subprocess.call(['mkdir', 'tmp'])
	o = env.reset()
	frame = 0
	while True:
		print('Frame {}'.format(frame), end='\r')
		env.render()
		plt.savefig(os.path.join('tmp', 'frame{0:05d}.png'.format(frame)))
		plt.close()
		o, r, t, i = env.step(policy.action(o, deterministic=True))
		frame += 1

		if t:
			env.render()
			plt.savefig(os.path.join('tmp', 'frame{0:05d}.png'.format(frame)))
			plt.close()
			break

	os.chdir('tmp')
	os.chdir('../')
	subprocess.call(['ffmpeg', '-framerate', '2', '-i', 'tmp/frame%05d.png', '-pix_fmt', 'yuv420p', '-vf', 'scale=1280:-2', 'video_{}.mp4'.format(run + 1)])
	subprocess.call(['rm', '-r', 'tmp'])
