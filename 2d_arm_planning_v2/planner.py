import numpy as np

from arm import Arm

class Planner:
	"""
	class for planning to a goal using a* in config space
	"""
	def __init__(self, arm, threshold = 0.1, discretization = 0.05, max_itrs=2000):
		self.arm = arm
		self.goal = (0, 0)
		self.threshold = threshold
		self.discretization = discretization
		self.max_itrs = max_itrs

	def set_goal(self, pt):
		self.goal = np.array(list(pt))

	def solve(self):
		"""
		Use A* with heuristic as Eudlidean dist in op space.
		"""
		start = Node(self.arm.get_joint_space(), 0, self.heuristic(self.arm.get_joint_space()), None)
		openlist = [start]
		closedlist = []

		itr = 0
		while openlist and itr < self.max_itrs:
			curr = openlist.pop()
			print(curr)
			print(self.arm.get_end_effector_pose_from(curr.pt)[:-1])
			print(self.goal)
			print(self.is_goal(curr.pt))

			if self.is_goal(curr.pt):
				break

			#expand
			new_nodes = self.expand_1_neighborhood(curr)
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
			itr += 1
			print('ITRS = {}'.format(itr))

		#reconstruct path
		path = [curr]
		prev = curr.bp
		while prev is not None:
			prev = prev.bp
			curr = curr.bp
			path.append(curr)
		path.reverse()
		if itr != self.max_itrs:
			print('no path found.')
		locs = [self.arm.get_end_effector_pose_from(n.pt)[:-1] for n in path]
		np.set_printoptions(precision=2, suppress=True)
		print(np.concatenate([np.stack([n.pt for n in path]), locs], axis=1))
		
		print('openlist size = {}, closedlist size = {}'.format(len(openlist), len(closedlist)))
		return path, closedlist, openlist
		

	def expand_1_neighborhood(self, node):
		pt = node.pt
		neighbors = []
		for i in range(len(pt)):
			diff = np.zeros(pt.shape)
			diff[i] += self.discretization
			neighbors.append(pt + diff)
			neighbors.append(pt - diff)

		nodes = [Node(n, node.g + self.discretization, 10*self.heuristic(n), node) for n in neighbors]
		valid_nodes = self.filter_expansions(nodes)
		return valid_nodes

	def filter_expansions(self, nodes):
		"""
		Given a list of node expansions, filter out the ones that are not feasible (joint limits, obstacles, etc.)
		"""
		#Filter joint constraints
		out = []
		joint_limits = [(link.min_angle, link.max_angle) for link in self.arm.control_links]
		for node in nodes:
			node_ok = True
			for angle, limits in zip(node.pt, joint_limits):
				if angle < limits[0] or angle > limits[1]:
					node_ok = False
					break
			if node_ok:
				out.append(node)
		return out
				
			

	def heuristic(self, pt):
		ee = self.arm.get_end_effector_pose_from(pt)[:-1]
		return sum((ee - self.goal) ** 2) ** 0.5

	def is_goal(self, pt):
		ee = self.arm.get_end_effector_pose_from(pt)[:-1]
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
