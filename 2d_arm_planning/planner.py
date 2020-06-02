import numpy as np

from arm import Arm

class Planner:
	"""
	class for planning to a goal using a* in config space
	"""
	def __init__(self, arm, threshold = 0.1, discretization = 0.05):
		self.arm = arm
		self.goal = (0, 0)
		self.threshold = threshold
		self.discretization = discretization

	def set_goal(self, pt):
		self.goal = np.array(list(pt))

	def solve(self):
		"""
		Use A* with heuristic as Eudlidean dist in op space.
		"""
		start = Node(self.arm.config_point, 0, self.heuristic(self.arm.config_point), None)
		openlist = [start]
		closedlist = []

		while openlist:
			curr = openlist.pop()
			print(curr)
			print(self.arm.end_effector_from(curr.pt)[:-1])
			print(self.goal)
			print(self.is_goal(curr.pt))

			if self.is_goal(curr.pt):
				break

			#expand
			new_nodes = self.expand_2_neighborhood(curr)
			for n_node in new_nodes:
				if n_node in closedlist:
					continue
				elif n_node in openlist:
					n_idx = openlist.index(n_node)
					n2 = openlist[n_idx]
					openlist[n_idx] = n_node if n_node.f < n2.f else n2

				else:
					openlist.append(n_node)

			closedlist.append(curr)

			openlist.sort(key = lambda x: x.f, reverse=True)

		#reconstruct path
		path = [curr]
		prev = curr.bp
		while prev is not None:
			prev = prev.bp
			curr = curr.bp
			path.append(curr)
		path.reverse()
		locs = [self.arm.end_effector_from(n.pt)[:-1] for n in path]
		np.set_printoptions(precision=2, suppress=True)
		print(np.concatenate([np.stack([n.pt for n in path]), locs], axis=1))
		
		print('openlist size = {}, closedlist size = {}'.format(len(openlist), len(closedlist)))
		return path, closedlist, openlist
		

	def expand_1_neighborhood(self, node):
		pt = node.pt
		neighbors = [pt + np.array([0.0, self.discretization]), pt + np.array([0.0, -self.discretization]), pt + np.array([self.discretization, 0.0]), pt + np.array([-self.discretization, 0.0])]

		return [Node(n, node.g + self.discretization , self.heuristic(n), node) for n in neighbors]
		#* np.sum(self.arm.links[-2:])

	def expand_2_neighborhood(self, node):
		pt = node.pt
		neighbors = [pt + np.array([0.0, self.discretization]), pt + np.array([0.0, -self.discretization]), pt + np.array([self.discretization, 0.0]), pt + np.array([-self.discretization, 0.0]), pt + np.array([self.discretization, self.discretization]), pt + np.array([self.discretization, -self.discretization]), pt + np.array([-self.discretization, self.discretization]), pt + np.array([-self.discretization, -self.discretization])]

		return [Node(n, node.g + self.discretization , self.heuristic(n), node) for n in neighbors]

	def heuristic(self, pt):
		ee = self.arm.end_effector_from(pt)[:-1]
		return sum((ee - self.goal) ** 2) ** 0.5

	def is_goal(self, pt):
		ee = self.arm.end_effector_from(pt)[:-1]
		return sum((ee - self.goal) ** 2) ** 0.5 < self.threshold
		
class Node:
	"""
	Node class for A* that stores 
	"""
	def __init__(self, pt, g, h, bp):
		self.pt = pt
		self.g = g
		self.h = h
		self.bp = bp

	@property
	def f(self):
		return self.g + self.h	

	def __repr__(self):
		return '<pt:{}, f:{:.2f}, g:{:.2f}, h:{:.2f}>'.format(self.pt, self.f, self.g, self.h)

	def __hash__(self):
		return hash(self.pt[0]) + hash(self.pt[1])

	def __eq__(self, n):
		return sum(np.abs(self.pt - n.pt)) < 1e-10

if __name__ == '__main__':
	arm = Arm([1, 2, 3])
	planner = Planner(arm, discretization = 0.05)
	planner.set_goal(np.array([2.0, 3.0]))
	planner.solve()
