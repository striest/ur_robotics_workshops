# General A* Formulation - Grid Demo Implementation.
# Author: Benned Hedegaard
# Last revised 6/11/2020

import pygame
import numpy as np
import random
from math import sqrt

### GLOBAL VARIABLES FOR GUI ###
SIZE = 800
CELL_SIZE = 30.0
GRID_R = 10 # Grid cells off from 0-axis in either direction.
OBSTACLE_DENSITY = 0.4

CENTER = (SIZE*0.5, SIZE*0.5)
GRID_OFFSET = CENTER[0] - (GRID_R+0.5)*CELL_SIZE
GRID = np.zeros((GRID_R*2+1,GRID_R*2+1))

# The grid stores/illustrates obstacles and the open/closed lists.
# Certain numbers in the grid correspond to specific colors.

RED = (255,0,0)
GREEN = (0,204,0)
BLUE = (0,102,255)
LIGHT_BLUE = (102,204,255)
BLACK = (0,0,0)
WHITE = (255,255,255)

# Color preferences
DEFAULT_COLOR = WHITE
TEXT_COLOR = BLACK
TEXT_CENTER = (CENTER[0], 30)
GRID_BORDER_COLOR = BLACK

OBSTACLE_COLOR = BLACK #1
START_COLOR = GREEN #2
GOAL_COLOR = RED #3
OPEN_LIST_COLOR = LIGHT_BLUE #4
CLOSED_LIST_COLOR = BLUE #5
PATH_COLOR = GREEN #6

### A* CODE BEGIN ###

class State:
	"""A State representation must be explicitly defined.

	For this program, data should be a [row, column] list.
	"""
	def __init__(self, data):
		"""Initialize a new State object.

		Args:
			data [r,c] list: The grid row and column this State represents.
		"""
		self.data = data

	def __eq__(self, other):
		"""Overrides the default implementation.
		
		Args:
			other (State): Another State to compare with this one.

		Returns:
			same (bool): T/F if this State has the same data as the other.
		"""
		if type(self) != type(other):
			return False
		same = (self.data[0] == other.data[0]) and (self.data[1] == other.data[1])
		return same

	def __repr__(self):
		return "("+str(self.data[0])+", "+str(self.data[1])+")"

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

def actions4(n):
	"""4-connected actions function.

	Args:
		n (Node): The Node from which we're considering actions.

	Returns:
		action_list (list of actions): List of valid actions from Node n.
	"""
	return [[0,1],[-1,0],[1,0],[0,-1]]

def actions8(n):
	"""8-connected actions function.

	Args:
		n (Node): The Node from which we're considering actions.

	Returns:
		action_list (list of actions): List of valid actions from Node n.
	"""
	return [[-1,1],[0,1],[1,1],[-1,0],[1,0],[-1,-1],[0,-1],[1,-1]]

def neighbors(n):
	"""Generates all valid neighbors of the given node.

	This function simplifies the overall A* process. We combine what would have been two
	functions: actions(n) and result(n, a) into one function. In doing so, we can more
	easily ensure that all new nodes we're generating are within the valid state space.

	Args:
		n (Node): Node we're expanding.

	Returns:
		neighbors (list of Nodes): All valid resulting Nodes from all valid actions of Node n.
		Only sets neighbor.state. Doesn't touch g, f, or prev.
	"""
	neighbors = []

	actions = actions4(n)
	for a in actions:
		new_r = n.state.data[0] + a[0]
		new_c = n.state.data[1] + a[1]

		# Skip nodes outside the grid or occupied nodes.
		if new_r < 0 or new_c < 0 or new_r >= GRID.shape[0] or new_c >= GRID.shape[1]:
			continue
		if GRID[new_r,new_c] == 1:
			continue

		new_state = State([new_r, new_c])
		neighbors.append(Node(new_state))

	return neighbors

def euclidean(s1, s2):
	"""Returns Euclidean distance between two States.

	Args:
		s1 (State): First point.
		s2 (State): Second point.

	Returns:
		distance (float): Distance between the States.
	"""
	p1 = s1.data
	p2 = s2.data
	return sqrt(float(p1[0] - p2[0])**2 + float(p1[1] - p2[1])**2)

def cost(curr, next):
	"""Cost function for grid demonstration.

	Args:
		curr (Node): The Node the action was taken from.
		next (Node): The Node reached by the action.

	Returns:
		cost (float): Cost of the performed action.
	"""
	return euclidean(curr.state, next.state)

def h(n, G):
	"""Heuristic function for grid demonstration.
	
	Args:
		n (Node): Node to evaluate this heuristic on.
		G (list of States): The set of goal States.
    
    Returns:
    	min_cost (float): The minimum cost to any goal State from Node n.
	"""
	min_cost = euclidean(n.state, G[0]) # Initializes min safely.

	for g in G: # Loop over all goal states to find which has minimum cost.
		distance = euclidean(n.state, g)
		if min_cost > distance:
			min_cost = distance
	return min_cost

def backtrack(n, closed_list):
	"""General backtracking function. Works for the above Node format.
	
	Args:
		n (Node): The Node we're backtracking from.
		closed_list (list of Nodes): Closed list from A* search.

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
	"""Updates the GUI grid based on the current open and closed lists.
	
	Args:
		open_list (list of Nodes): Current open list.
		closed_list (list of Nodes): Current closed list.
		gui (pygame.Surface): Screen to draw updates to.
	"""
	for o in open_list:
		GRID[o.state.data[0],o.state.data[1]] = 4
	for c in closed_list:
		GRID[c.state.data[0],c.state.data[1]] = 5

	draw_grid(gui)

	text = "Currently running A*..."
	font = pygame.font.SysFont('vira', 32)
	draw_text(text, font, TEXT_CENTER, gui)
	pygame.display.flip() # Update screen to display all changes made.
	clock = pygame.time.Clock()
	clock.tick(30) # Delays so user can process what's going on.

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
				return backtrack(curr, closed_list) # Line 6

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

### A* CODE END ###

### GUI CODE BEGIN ###

def reset_grid():
	"""Resets the GUI's grid to a random field.

	Accesses the GRID and OBSTACLE_DENSITY global variables.
	"""
	for r in range(GRID.shape[0]):
		for c in range(GRID.shape[1]):
			if random.random() < OBSTACLE_DENSITY:
				GRID[r,c] = 1
			else:
				GRID[r,c] = 0

def draw_grid(screen):
	"""Draws the grid to the given screen.

	Args:
		screen (pygame.Surface): The screen to draw on.

	Accesses the DEFAULT_COLOR, GRID, GRID_OFFSET, CELL_SIZE,
	SIZE, and various _COLOR global variables.
	"""
	screen.fill(DEFAULT_COLOR) # Clears screen
	
	for r in range(GRID.shape[0]):
		for c in range(GRID.shape[1]):
			rect = pygame.Rect(GRID_OFFSET+CELL_SIZE*c, GRID_OFFSET+CELL_SIZE*r,CELL_SIZE,CELL_SIZE)
			rect_color = WHITE # By default draw nothing
			if GRID[r,c] == 1:
				rect_color = OBSTACLE_COLOR # Obstacles
			elif GRID[r,c] == 2:
				rect_color = START_COLOR # Start point
			elif GRID[r,c] == 3:
				rect_color = GOAL_COLOR # Goal point
			elif GRID[r,c] == 4:
				rect_color = OPEN_LIST_COLOR # Open list
			elif GRID[r,c] == 5:
				rect_color = CLOSED_LIST_COLOR # Closed list
			elif GRID[r,c] == 6:
				rect_color = PATH_COLOR # Path

			pygame.draw.rect(screen, rect_color, rect)

	pygame.draw.line(screen, GRID_BORDER_COLOR, (GRID_OFFSET,GRID_OFFSET), (GRID_OFFSET,SIZE-GRID_OFFSET))
	pygame.draw.line(screen, GRID_BORDER_COLOR, (GRID_OFFSET,GRID_OFFSET), (SIZE-GRID_OFFSET,GRID_OFFSET))
	pygame.draw.line(screen, GRID_BORDER_COLOR, (SIZE-GRID_OFFSET,SIZE-GRID_OFFSET), (GRID_OFFSET,SIZE-GRID_OFFSET))
	pygame.draw.line(screen, GRID_BORDER_COLOR, (SIZE-GRID_OFFSET,SIZE-GRID_OFFSET), (SIZE-GRID_OFFSET,GRID_OFFSET))

def draw_text(text, font, center, screen):
	"""Draws the given text to the screen centered at center.

	Args:
		text (str): String to be drawn.
		font (pygame.font.Font): Sets font of the text.
		center (r,c) 2-tuple: Sets center pixel of the text on screen.
		screen (pygame.Surface): The screen to draw on.

	Accesses the global TEXT_COLOR variable.
	"""
	text_surface = font.render(text, True, TEXT_COLOR)
	text_rect = text_surface.get_rect()
	text_rect.center = center
	screen.blit(text_surface, text_rect)

def draw_path(path):
	"""Draws the given path to the grid.

	Args:
		path (list of Nodes): The path to be drawn.

	Accesses the GRID global variable.
	"""
	for n in path:
		GRID[n.state.data[0],n.state.data[1]] = 6

def pixel_to_cell(pos):
	"""Converts the given pixel on the screen to cell in the grid.

	Args:
		pos (x,y) 2-tuple: Pixel coordinates on screen.

	Returns:
		output [x,y] list: Corresponding coordinate on grid.

	Accesses the CENTER, GRID_R, CELL_SIZE, and GRID global variables.
	"""
	rel_pixels_x = pos[0]-CENTER[0]
	rel_pixels_y = CENTER[1]-pos[1]
	cell_row = GRID_R - round(rel_pixels_y/CELL_SIZE)
	cell_col = round(rel_pixels_x/CELL_SIZE) + GRID_R
	if cell_row < 0 or cell_row >= GRID.shape[0] or cell_col < 0 or cell_col >= GRID.shape[1]:
		return None
	output = [cell_row, cell_col]
	return output

def wait_for_click():
	"""Waits for the user to click the screen.

	Uses a pygame.time.Clock() object.
	"""
	clock = pygame.time.Clock()
	while True:
		clock.tick(20) # Loop at 20hz
		for event in pygame.event.get(): # User did something!
			if event.type == pygame.QUIT: # If user clicked close...
				done = True # Flag done so we exit this loop
			if event.type == pygame.MOUSEBUTTONDOWN:
				return

### GUI CODE END ###

def main():
	pygame.init()
	screen = pygame.display.set_mode((SIZE, SIZE))
	pygame.display.set_caption("A* Demonstration")
	clock = pygame.time.Clock()
	reset_grid()

	text = "Please choose a starting point."
	FONT = pygame.font.SysFont('vira', 32)

	# Control flow variables
	stage = 0 # Tracks how far along the process the program is.
	done = False # Tracks if the user has closed the window.

	"""
	STAGES

	Stage 0 - User selects starting point for the planning problem.
	Stage 1 - User selects goal point(s) for the planning problem.
	Stage 2 - Exits while loop to run A*
	"""

	# A* algorithm variables
	start_state = None
	goals = []

	while not done: # Main program loop
		### PyGame updates to the screen ###
		clock.tick(60) # Caps this loop to 60hz.
		draw_grid(screen)
		draw_text(text, FONT, TEXT_CENTER, screen)

		mouse_pos = pygame.mouse.get_pos()

		for event in pygame.event.get(): # User did something!
			if event.type == pygame.QUIT: # If user clicked close...
				done = True # Flag done so we exit this loop
			if stage == 0: # User is selecting the starting point.
				if event.type == pygame.MOUSEBUTTONDOWN:
					cell_index = pixel_to_cell(mouse_pos)
					if cell_index == None:
						text = "Invalid starting location. Please click any cell inside the grid."
					else:
						GRID[cell_index[0], cell_index[1]] = 2
						start_state = State(cell_index)
						text = "Now choose any number of goal points. Then click outside the grid."
						stage = 1
			elif stage == 1: # User is selecting goal points.
				if event.type == pygame.MOUSEBUTTONDOWN:
					cell_index = pixel_to_cell(mouse_pos)
					if cell_index == None and len(goals) == 0:
						text = "We need at least one goal. Please click any cell inside the grid."
					elif cell_index == None:
						text = "We'll now run A* on this setup."
						stage = 2
					else: # Add a valid goal
						GRID[cell_index[0], cell_index[1]] = 3
						text = "Choose additional goal points if desired, else click outside the grid."
						goals.append(State(cell_index))

		pygame.draw.circle(screen, BLUE, mouse_pos, 3)

		pygame.display.flip() # Update screen to display all changes made.

		if stage < 2:
			continue

		# Once we're here, we're ready to run A*
		break # Exit the while loop.

	path = a_star(start_state, goals, screen)

	if len(path) == 0: # A* failed.
		text = "No nodes are left to expand. Search failed."
	else:
		text = "We've found a valid path! (click to exit)"
		draw_path(path)

	# Update GUI and do nothing more.
	draw_grid(screen)
	draw_text(text, FONT, TEXT_CENTER, screen)
	pygame.display.flip() # Update screen to display all changes made.
	pygame.event.get() # Clears the queue of events.
	wait_for_click()

	pygame.quit() # Be IDLE friendly :)
	print("Window closed.")

if __name__ == "__main__":
	main()
