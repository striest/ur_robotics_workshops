from numpy import sin, cos, pi
import numpy as np
import matplotlib.pyplot as plt

from link import Link, FixedLink
from arm import Arm
from arm_gui import ArmGUI
from general_astar import AStarPlanner, Node

class ArmAStar(AStarPlanner):

	def __init__(self, arm, discretization = 5 * pi / 180, min_dist = 0.15):
		self.arm = arm
		self.goal_pt = None
		self.discretization = discretization
		self.min_dist = min_dist

	def goal_fn(self, state):
		return self.dist_to_goal(state) < self.min_dist

	def update_gui(self, ol, cl, gui):
		gui.openlist = ol
		gui.closedlist = cl
		gui.update()
		plt.pause(1e-10)
		print('ol = {}, cl = {}'.format(len(gui.openlist), len(gui.closedlist)))

	def h(self, node, G):
		return self.dist_to_goal(node.state)
		
	def dist_to_goal(self, state):	
		ee = self.arm.get_end_effector_pose_from(state)[:-1]
		return ((ee[0] - self.goal_pt[0])**2 + (ee[1] - self.goal_pt[1])**2)**0.5

	def cost(self, curr, n):
		return self.discretization
		return np.sum(np.abs(curr.state - n.state))

	def neighbors(self, curr):
		curr_state = curr.state

		#Get all expansions
		expansions = self.expand_2n(curr_state)
		#Filter invalid expansions
		valid_expansions = []

		for node in expansions:
			if self.arm.valid_configuration(node.state):
				valid_expansions.append(node)

		#print([n.state for n in valid_expansions])

		return valid_expansions

	def expand_1n(self, state):
		expansions = []

		for d in range(state.shape[0]):
			new_state = state.copy()
			new_state[d] += self.discretization
			expansions.append(Node(new_state))
			new_state = state.copy()
			new_state[d] -= self.discretization
			expansions.append(Node(new_state))

		return expansions
	
	#There should be a better way to do this than copy-paste the combos of plus/minus	
	def expand_2n(self, state):
		expansions = self.expand_1n(state)

		for d in range(state.shape[0]):
			for d2 in range(d+1, state.shape[0]):
				#4 options
				new_state = state.copy()
				new_state[d] += self.discretization
				new_state[d2] += self.discretization
				expansions.append(Node(new_state))
				new_state = state.copy()
				new_state[d] += self.discretization
				new_state[d2] -= self.discretization
				expansions.append(Node(new_state))
				new_state = state.copy()
				new_state[d] -= self.discretization
				new_state[d2] += self.discretization
				expansions.append(Node(new_state))
				new_state = state.copy()
				new_state[d] -= self.discretization
				new_state[d2] -= self.discretization
				expansions.append(Node(new_state))

		return expansions

if __name__ == '__main__':
	l1 = FixedLink(length = 5, angle = pi/2)
	l2 = FixedLink(length = 0, angle = -pi/2)
	l3 = Link(length = 3, min_angle = -1e4, max_angle=1e4, angle = pi/2)
	l4 = FixedLink(length = 0, angle = -pi/2)
	l5 = Link(length = 3, min_angle = -1e4, max_angle=1e4, angle = pi/2)
	l6 = FixedLink(length = 0, angle = -pi/2)
	l7 = Link(length = 3, min_angle = -1e4, max_angle=1e4, angle = pi/2)
	arm = Arm([l1, l2, l3, l4, l5, l6, l7])
	print(arm)
	astar = ArmAStar(arm)
	gui = ArmGUI(arm, astar)
