from numpy import sin, cos, pi
import numpy as np
import matplotlib.pyplot as plt

from link import Link, FixedLink
from arm import Arm
from arm_gui import ArmGUI
from general_astar import AStarPlanner, Node

class ArmAStar(AStarPlanner):

	def __init__(self, arm, discretization = 5 * pi / 180, min_dist = 0.15):
		"""
		Intitializes an A* planner with an arm
		Args:
			arm: The arm to plan with
			discretization: How far to move joint angles for neighboring nodes.
			min_dist: How close to the goal point a pose needs to be to be consideres a goal.
		"""
		self.arm = arm
		self.goal_pt = None
		self.discretization = discretization
		self.min_dist = min_dist

	def goal_fn(self, state):
		"""
		Goal function is whether the current distance to goal is less than the threshold.
		"""
		return self.dist_to_goal(state) < self.min_dist

	def update_gui(self, ol, cl, gui):
		"""
		Updates the GUI using the GUI's update func.
		"""
		gui.openlist = ol
		gui.closedlist = cl
		gui.update()
		plt.pause(1e-10)
		print('ol = {}, cl = {}'.format(len(gui.openlist), len(gui.closedlist)))

	def h(self, node, G):
		"""
		Heuristic is Euclidean distance to the goal point
		"""
		return self.dist_to_goal(node.state)
		
	def dist_to_goal(self, state):
		"""
		Use arm's built-in end-effector pose func and use Euclidean dist to goal pose.
		"""
		ee = self.arm.get_end_effector_pose_from(state)[:-1]
		return ((ee[0] - self.goal_pt[0])**2 + (ee[1] - self.goal_pt[1])**2)**0.5

	def cost(self, curr, n):
		"""
		All node costs are discretization
		"""
		return self.discretization

	def neighbors(self, curr):
		"""
		Compute neighbor nodes from current state.
		Neighbors are valid expansions from the current state.
		An expansion is obtained by adding or subtracting discretizations from the current state. This function also prunes out invalid configurations as computed by the arm.
		"""
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
		"""
		Compute the 1-neighborhood from a node. (Add/subtract 1 discretization from each dimension of state).
		"""
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
		"""
		Compute the 2-neighborhood from a node (Add/subtract discreizations from every pair of dimensions of state).
		"""
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
