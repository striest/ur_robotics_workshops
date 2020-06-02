import matplotlib.pyplot as plt
from matplotlib.widgets import Button
import numpy as np
from math import pi
import time

from arm import Arm
from planner import Planner

class GUI:
	"""
	Class for rendering operational and config-space planning for 2D robot arms
	Arm assumed to be 2D so we can plot its configuration space.
	"""
	def __init__(self, arm, planner):
		self.arm = arm
		self.planner = planner
		self.h = False

		self.config_pt = (0, 0)
		self.goal_pt = (0, 0)
		self.path = None
		self.openlist = None
		self.closedlist = None

		self.move_config_pt = False
		self.move_goal_pt = False

		fig, ax = plt.subplots(1, 2, figsize = (9, 4))
		self.config_ax = ax[0]
		self.op_ax = ax[1]
		self.fig = fig
		plt.subplots_adjust(bottom=0.2)

		self.config_ax.set_xlim(-0.1, pi + 0.1)
		self.config_ax.set_ylim(-0.1, pi + 0.1)
		self.op_ax.set_xlim(-5.1, 5.1)
		self.op_ax.set_ylim(-0.1, 10.1)

		self.config_ax.grid()

		self.config_obj = self.draw_config_point(self.config_pt)
		self.goal_obj = self.draw_op_point(self.goal_pt)
		self.arm_obj = self.draw_arm()

		axbutton = plt.axes([0.7, 0.05, 0.1, 0.075])
		self.plan_button = Button(axbutton, 'Plan')
		self.plan_button.on_clicked(self.handle_plan)

		self.cid1 = fig.canvas.mpl_connect('button_press_event', self.select_point)
		self.cid2 = fig.canvas.mpl_connect('button_release_event', self.deselect_point)
		self.cid3 = fig.canvas.mpl_connect('motion_notify_event', self.move_point)
		self.update()

		plt.show()

	def draw_config_point(self, pt):
		return self.config_ax.scatter([pt[0]], [pt[1]], marker='x', c='r')

	def draw_op_point(self, pt):
		return self.op_ax.scatter([pt[0]], [pt[1]], marker='x', c='g')

	def draw_arm(self):
		locs = self.arm.joint_locations
		ee = self.arm.end_effector
		self.op_ax.plot(locs[:, 0], locs[:, 1], c='k')
		self.op_ax.scatter(locs[1:-1, 0], locs[1:-1, 1], marker='o', c='k')
		self.op_ax.scatter(ee[0], ee[1], marker='x', c='r')

	def draw_path(self):
		if self.path:
			path_conf_pts = np.stack([n.pt for n in self.path]) % (2*pi)
			self.config_ax.scatter(path_conf_pts[:, 0], path_conf_pts[:, 1], marker='.', s=4, c='b')
			op_pts = np.stack([self.arm.end_effector_from(n.pt) for n in self.path])
			self.op_ax.plot(op_pts[:, 0], op_pts[:, 1], color='b')

	def draw_lists(self):
		if self.openlist:
			pts = np.stack([n.pt%(2*pi) for n in self.openlist])
			self.config_ax.scatter(pts[:, 0], pts[:, 1], alpha=0.25, c='k', marker='.', s=4)
		if self.closedlist:
			pts = np.stack([n.pt%(2*pi) for n in self.closedlist])
			self.config_ax.scatter(pts[:, 0], pts[:, 1], alpha=0.75, c='k', marker='.', s=4)
			
		
	def select_point(self, event):
		if event.inaxes == self.config_ax:
			self.move_config_pt = True
		elif event.inaxes == self.op_ax:
			self.move_goal_pt = True
		self.move_point(event)

	def deselect_point(self, event):
		self.move_config_pt = False
		self.move_goal_pt = False

	def move_point(self, event):
		if event.inaxes == self.config_ax and self.move_config_pt:
			self.config_pt = (event.xdata, event.ydata)
		elif event.inaxes == self.op_ax and self.move_goal_pt:
			self.goal_pt = (event.xdata, event.ydata)
		self.update()

	def handle_plan(self, event):
		print('Planning...')
		path, ol, cl = self.planner.solve()
		self.path = path
		self.openlist = ol
		self.closedlist = cl
		for n in path:
			pt = n.pt % (2*pi)
			print('pt=', pt, 'npt=', n.pt)
			self.config_pt = (pt[0], pt[1])
			self.update()
			self.fig.canvas.draw()
			plt.pause(0.05)

	def draw_h(self):
		pts = []
		alphas = []
		for x in np.arange(0, 2*pi, 0.1):
			for y in np.arange(0, 2*pi, 0.1):
				pt = np.array([x, y])
				a = self.planner.heuristic(np.array([x, y]))
				pts.append(pt)
				alphas.append(a)
		amax = max(alphas)
		for p,a in zip(pts, alphas):
			print('h({}) = {}'.format(p, a))
			self.config_ax.scatter([p[0]], [p[1]], alpha = a/amax, c='b')


	def update(self):
		self.config_ax.clear()
		self.op_ax.clear()
		self.config_ax.grid()

		self.arm.update_joints(list(self.config_pt))
		self.planner.set_goal(self.goal_pt)

		self.arm_obj = self.draw_arm()
		self.goal_obj = self.draw_op_point(self.goal_pt)
		self.config_obj = self.draw_config_point(self.config_pt)
		self.draw_path()
		self.draw_lists()
		if self.h:
			self.draw_h()

		self.config_ax.set_title('Config Space ($\Theta_1$ = {:.2f}, $\Theta_2$ = {:.2f})'.format(*self.config_pt))
		self.op_ax.set_title('Operational Space: Goal:(x = {:.2f}, y = {:.2f})'.format(*self.goal_pt))
		self.config_ax.set_xlim(-0.1, 2*pi + 0.1)
		self.config_ax.set_ylim(-0.1, 2*pi + 0.1)
		self.op_ax.set_xlim(-10.1, 10.1)
		self.op_ax.set_ylim(-10.1, 10.1)
		self.config_ax.set_xlabel('$\Theta_1$')
		self.config_ax.set_ylabel('$\Theta_2$')
		self.op_ax.set_xlabel('X')
		self.op_ax.set_ylabel('Y')

		self.fig.canvas.draw()


if __name__ == '__main__':
	arm = Arm([1, 4, 5])
	planner = Planner(arm, discretization=0.05, threshold=0.15)
	gui = GUI(arm, planner)
