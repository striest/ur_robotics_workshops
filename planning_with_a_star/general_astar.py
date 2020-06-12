# General A* Formulation
# Author: Benned Hedegaard
# Last revised 6/12/2020

class State:
	"""A State representation must be explicitly defined."""
	def __init__(self, data):
		"""Initialize a new State object. NEEDS IMPLEMENTATION

		Args:
			data (): 
		"""
		self.data = data

	def __eq__(self, other):
		"""Overrides the default implementation. NEEDS IMPLEMENTATION

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
		"""Overrides the default implementation. OPTIONAL IMPLEMENTATION
		
		Returns a string representation of this State.
		"""
		pass

class Node:
	"""General Node data structure. Use any State representation you'd like."""
	def __init__(self, state):
		self.state = state
		self.g = 0
		self.f = 0
		self.prev = None

	def __eq__(self, other):
		"""Overrides the default implementation.
		
		Args:
			other (Node): Another Node to compare with this one.

		Returns:
			same (bool): T/F if this Node has the same State as the other.
		"""
		if type(self) != type(other):
			return False
		return (self.state == other.state)

	def __repr__(self):
		return str(self.state)+": g = "+str(self.g)+" f = "+str(self.f)

def neighbors(n):
	"""Generates all valid neighbors of the given node. NEEDS IMPLEMENTATION

	This function simplifies the overall A* process. We combine what would have
	been two functions: actions(n) and result(n, a) into one function. We can
	immediately check that all newly explored nodes are valid and unoccupied.

	Args:
		n (Node): Node we're expanding.

	Returns:
		neighbors (list of Nodes):
			All valid resulting Nodes from all valid actions of Node n.
			Only sets neighbor.state. Doesn't touch g, f, or prev.
	"""
	pass

def cost(curr, next):
	"""General cost function. NEEDS IMPLEMENTATION

	Computes and returns the cost between the given current Node and next Node.

	Args:
		curr (Node): The Node an action was taken from.
		next (Node): The Node reached by that action.

	Returns:
		cost (float): Cost of the performed action.
	"""
	pass

def h(n, G):
	"""General heuristic function. NEEDS IMPLEMENTATION

	Returns an estimate of the minimum cost from Node n to any goal State in G.
	
	Args:
		n (Node): Node to evaluate this heuristic on.
		G (list of States): The set of goal States. Assumed non-empty.
    
    Returns:
    	min_cost (float): The minimum cost to any goal State from Node n.
	"""
	pass

def backtrack(n):
	"""General backtracking function. Works for the above Node format.

	Computes/returns the path preceding Node n by recursively backtracking
	through the Node.prev pointers.
	
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

def push(n, list):
	"""General unique-enforcing Push function.

	Adds Node n to the given list. If another Node in the list has the
	same State as n, keeps the better Node based on its f value.
	
	Args:
		n (Node): The Node we're considering adding to the list.
		list (list of Nodes): List to add the Node to.

	Returns:
		Directly edits the given list and thus returns nothing.
	"""
	add_n = True # Add n unless a better Node is found.

	for i in range(len(list)):
		if list[i] == n: # list[i] has the same State as n...
			if (n.f < list[i].f): # but n is better.
				list.pop(i)
			else: # but n is worse.
				add_n = False
			break

	if add_n:
		list.append(n)

def update_gui(open_list, closed_list, gui):
	"""Updates the GUI. OPTIONAL IMPLEMENTATION

	Interfaces with the GUI using information from the current
	open and closed lists. Entirely optional but can help debugging.
	
	Args:
		open_list (list of Nodes): Current open list.
		closed_list (list of Nodes): Current closed list.
		gui (): 
	"""
	pass

def a_star(s, G, gui=None):
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

	while len(open_list) != 0: # Line 3
		open_list.sort(key=lambda x: x.f) # Keeps open list sorted by f.
		update_gui(open_list, closed_list, gui) # Update the GUI.

		curr = open_list.pop(0) # Line 4

		# Line 5. Scans over all goal states checking for equality.
		for g in G:
			if curr.state == g:
				return backtrack(curr) # Line 6

		closed_list.append(curr) # Line 7

		for next in neighbors(curr): # Line 8
			
			# Line 9. Scans over closed list to ensure new node is not closed.
			skip = False
			for closed_node in closed_list:
				if next == closed_node:
					skip = True
					break
			if skip:
				continue # Line 10

			next.g = curr.g + cost(curr, next) # Line 11
			next.f = next.g + h(next, G) # Line 12
			next.prev = curr # Line 13
			push(next, open_list) # Line 14

	return [] # Line 15. Empty path indicates failure.
