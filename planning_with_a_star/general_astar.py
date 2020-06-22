# General A* Formulation
# Author: Benned Hedegaard
# Last revised 6/12/2020
# Revisions by Sam Triest on 6/12/2020

import abc

class State:
	"""A State representation must be explicitly defined."""
	def __init__(self, data):
		"""
		Initialize a new State object. NEEDS IMPLEMENTATION

		Args:
			data (): 
		"""
		self.data = data

	def __eq__(self, other):
		"""
		Overrides the default implementation. NEEDS IMPLEMENTATION

		Returns if this State's data is equal to another given State.
		
		Args:
			other (State): Another State to compare with this one.

		Returns:
			same (bool): T/F if this State has the same data as the other.
		"""
		if type(self) != type(other):
			return False

		# Check State's data here.
		pass

	def __repr__(self):
		"""
		Overrides the default implementation. OPTIONAL IMPLEMENTATION
		
		Returns a string representation of this State.
		"""
		pass

class Node:
	"""
	General Node data structure. Use any State representation you'd like.
	"""
	def __init__(self, state):
		self.state = state
		self.g = 0
		self.f = 0
		self.prev = None

	def __eq__(self, other):
		"""
		Overrides the default implementation.
		
		Args:
			other (Node): Another Node to compare with this one.

		Returns:
			same (bool): T/F if this Node has the same State as the other.
		"""
		if type(self) != type(other):
			return False
		equal = (self.state == other.state)
		return equal if type(equal) is bool else all(equal) #do this to handle numpy arrays as state

	def __repr__(self):
		return str(self.state)+": g = "+str(self.g)+" f = "+str(self.f)


class AStarPlanner(object, metaclass=abc.ABCMeta):

	@abc.abstractmethod
	def neighbors(self, n):
		"""
		General neighbors function. NEEDS IMPLEMENTATION

		This function simplifies the overall A* process. We combine what would have been two
		functions: actions(n) and result(n, a) into one function. In doing so, we can more
		easily ensure that all new nodes we're generating are within the valid state space.

		Args:
			n (Node): Node we're expanding.

		Returns:
			neighbors (list of Nodes): All valid resulting Nodes from all valid actions of Node n.
			Only sets neighbor.state. Doesn't touch g, f, or prev.
		"""
		pass

	@abc.abstractmethod
	def cost(self, c, n):
		"""
		Computes and returns the cost between the given current Node and next Node.

		Args:
			curr (Node): The Node an action was taken from.
			next (Node): The Node reached by that action.
		"""
		pass

	@abc.abstractmethod
	def h(self, n, G = None):
		"""
		General heuristic function. NEEDS IMPLEMENTATION
	
		Args:
			n (Node): Node to evaluate this heuristic on.
			G (list of States): The set of goal States.
    
		Returns:
			min_cost (float): The minimum cost to any goal State from Node n.
		"""
		pass
  
	def backtrack(self, n, closed_list):
		"""
		General backtracking function. Works for the above Node format.

		Computes/returns the path preceding Node n by recursively backtracking through the Node.prev pointers.

		Args:
			n (Node): The Node we're backtracking from.

		Returns:
			path (list of Nodes): The entire path leading to this Node.
		"""
		curr = n
		path = [n]
		while curr.prev != None:
			path.insert(0, curr.prev)
			curr = curr.prev
		return path

	def push(self, n, l):
		"""
		General unique-enforcing Push function.

		Adds Node n to the given list. If another Node in the list has the same State as n, keeps the better Node based on its f value.

		Args:
			n (Node): The Node we're considering adding to the list.
			list (list of Nodes): List to add the Node to.

		Returns:
			Directly edits the given list and thus returns nothing.
		"""
		add_n = True # Add n unless a better Node is found.

		for i in range(len(l)):
			if l[i] == n: # list[i] has the same State as n...
				if (n.f < l[i].f): # but n is better.
					l.pop(i)
				else: # but n is worse.
					add_n = False
				break

		if add_n:
			l.append(n)

	def update_gui(self, open_list, closed_list, gui):
		"""
		Updates the GUI grid based on the current open and closed lists.
    
		Interfaces with the GUI using information from the current open and closed lists. Entirely optional but can help debugging.

		Implementation is optional. Interface with your chosen GUI object.
	
		Args:
			open_list (list of Nodes): Current open list.
			closed_list (list of Nodes): Current closed list.
			gui (): 
		"""
		pass

	def a_star(self, s, G, gui=None):
		"""General A* implementation. Works with above functions.

		The corresponding line of pseudocode is labeled throughout.
	
		Args:
			s (State): Starting State for the algorithm.
			G (list of States): Goal States for the algorithm.
			gui (): Optional GUI parameter.

		Returns:
			path (list of Nodes): Optimal path for the given planning problem.
			An empty path indicates that the search has failed.
		"""
		open_list = [Node(s)] # Line 1. Converts starting State to Node.
		closed_list = [] # Line 2

		if type(G) is list:
			g_fn = lambda x: x in G
		else:
			g_fn = G

		while len(open_list) != 0: # Line 3
			open_list.sort(key=lambda x: x.f) # Keeps open list sorted by f.
			self.update_gui(open_list, closed_list, gui) # Update the GUI.

			curr = open_list.pop(0) # Line 4

			# Line 5. Scans over all goal states checking for equality.
			if g_fn(curr.state):
				print('found goal')
				return self.backtrack(curr, closed_list) # Line 6

			closed_list.append(curr) # Line 7

			for n in self.neighbors(curr): # Line 8
			
				# Line 9. Scans over closed list to ensure new node is not closed.
				if n not in closed_list:
					n.g = curr.g + self.cost(curr, n) # Line 11
					n.f = n.g + self.h(n, G) # Line 12
					n.prev = curr # Line 13
					self.push(n, open_list) # Line 14

		return [] # Line 15. Empty path indicates failure.
