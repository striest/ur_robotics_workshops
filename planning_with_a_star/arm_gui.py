import matplotlib.pyplot as plt
from matplotlib.widgets import Button
import numpy as np
from math import pi
import time

from link import Link, FixedLink
from arm import Arm

class ArmGUI:
	"""
	Class for rendering operational and config-space planning for 2D robot arms
	Arm assumed to be 2D so we can plot its configuration space.
	"""
	def __init__(self, arm, planner):
		self.arm = arm
		self.h = False

		self.planner = planner

		self.config_pt = np.zeros(len(self.arm.control_links))
		self.goal_pt = (0, 0)
		self.path = None
		self.openlist = None
		self.closedlist = None

		self.move_goal_pt = False

		fig, ax = plt.subplots(1, 2, figsize = (9, 4))
		self.config_ax = ax[0]
		self.op_ax = ax[1]
		self.fig = fig
		plt.subplots_adjust(bottom=0.2)

		self.config_ax.grid()

		axbutton = plt.axes([0.7, 0.05, 0.1, 0.075])
		self.plan_button = Button(axbutton, 'Plan')
		self.plan_button.on_clicked(self.handle_plan)

		self.config_obj = self.draw_config_point(self.config_pt)
		self.goal_obj = self.draw_op_point(self.goal_pt)
		self.arm_obj = self.draw_arm()

		self.cid1 = fig.canvas.mpl_connect('button_press_event', self.select_point)
		self.cid2 = fig.canvas.mpl_connect('button_release_event', self.deselect_point)
		self.cid3 = fig.canvas.mpl_connect('motion_notify_event', self.move_point)
		self.update()

		plt.show()

	def draw_config_point(self, pt):
		xs = pt % (2*pi)
		ys = np.arange(len(pt)) + 1

		for x, y in zip(xs, ys):
			self.config_ax.text(x + 0.1, y + 0.1, 'Joint {}'.format(y))

		return self.config_ax.scatter(xs, ys, marker='x', c='r')

	def draw_op_point(self, pt):
		return self.op_ax.scatter([pt[0]], [pt[1]], marker='x', c='g')

	def draw_arm(self):
		locs = self.arm.get_joint_poses()[:, :-1]
		ee = self.arm.get_end_effector_pose()[:-1]
		self.op_ax.plot(locs[:, 0], locs[:, 1], c='k')
		self.op_ax.scatter(locs[1:-1, 0], locs[1:-1, 1], marker='o', c='k')
		self.op_ax.scatter(ee[0], ee[1], marker='x', c='r')

	def draw_path(self):
		if self.path:
			op_pts = np.stack([self.arm.get_end_effector_pose_from(n.state) for n in self.path])
			self.op_ax.plot(op_pts[:, 0], op_pts[:, 1], color='b')

	def draw_lists(self):
		if self.openlist:
			pts = np.stack([self.arm.get_end_effector_pose_from(n.state) for n in self.openlist])
			self.op_ax.scatter(pts[:, 0], pts[:, 1], alpha=0.25, c='k', marker='.', s=4)
		if self.closedlist:
			pts = np.stack([self.arm.get_end_effector_pose_from(n.state) for n in self.closedlist])
			self.op_ax.scatter(pts[:, 0], pts[:, 1], alpha=0.5, c='k', marker='.', s=4)

	def handle_plan(self, event):
		self.move_goal_pt = False
		self.path = []
		self.planner.goal_pt = self.goal_pt
		print(self.planner.goal_pt)
		print(self.planner.goal_fn(np.zeros(len(self.arm.control_links))))
		path = self.planner.a_star(s = self.config_pt, G = self.planner.goal_fn, gui = self)
		self.path = path
		pathlen = len(path)
		for i, node in enumerate(path):
			print('path node {}/{}'.format(i, pathlen))
			print(node)
			self.config_pt = node.state
			self.arm.set_joint_space(node.state, suppress = False)
			print(self.arm.get_end_effector_pose())
			self.update()
			plt.pause(1e-3)

		self.move_goal_pt = True
				
	def select_point(self, event):
		if event.inaxes == self.op_ax:
			self.move_goal_pt = True
		self.move_point(event)

	def deselect_point(self, event):
		self.move_goal_pt = False

	def move_point(self, event):
		if event.inaxes == self.op_ax and self.move_goal_pt:
			self.goal_pt = (event.xdata, event.ydata)
		self.update()

	def update(self):
		self.config_ax.clear()
		self.op_ax.clear()
		self.config_ax.grid()

		self.arm.set_joint_space(list(self.config_pt))

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
		self.config_ax.set_ylim(0, len(self.arm.control_links) + 1)
		self.op_ax.set_xlim(-self.arm_len() - 0.1, self.arm_len() + 0.1)
		self.op_ax.set_ylim(-5.1, 2*self.arm_len() - 4.9)
		self.config_ax.set_xlabel('$\Theta$')
		self.config_ax.set_ylabel('Joint Number')
		self.op_ax.set_xlabel('X')
		self.op_ax.set_ylabel('Y')

		self.fig.canvas.draw()
		self.fig.canvas.flush_events()

	def arm_len(self):
		return sum([link.length for link in self.arm.links])


if __name__ == '__main__':
	l1 = FixedLink(length = 5, angle = pi/2)
	l2 = FixedLink(length = 0, angle = -pi/2)
	l3 = Link(length = 6, max_angle=pi)
	l4 = FixedLink(length = 0, angle = -pi/2)
	l5 = Link(length = 4, max_angle=pi)
	l6 = FixedLink(length = 0, angle = -pi/2)
	l7 = Link(length = 4, max_angle=pi)
	arm = Arm([l1, l2, l3, l4, l5, l6, l7])
	print(arm)
	gui = ArmGUI(arm)




