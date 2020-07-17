from math import pi

from stepper_motors.stepper import Stepper

from arm import Arm
from link import Link, FixedLink
from arm_astar import ArmAStar
from arm_gui import ArmGUI

if __name__ == '__main__':
    lf1 = FixedLink(length = 0, angle = pi/2)
    l1 = Link(length = 11.8, angle = 0, min_angle = -1e4, max_angle = 1e4)
    lf2 = FixedLink(length = .12, angle = pi/2)
    lf3 = FixedLink(length = 0, angle = -pi/2)
    l2 = Link(length = 11.8, angle = 0, min_angle = -1e4, max_angle = 1e4)

    s2 = Stepper(2, 3, 4, 17, delay = 1e-2)
    s1 = Stepper(27, 22, 10, 9, delay = 1e-2)
    s1.reverse()

    arm = Arm([lf1, l1, lf2, lf3, l2], [s1, s2])

    print(arm)
    astar = ArmAStar(arm, discretization = 3*pi/180, min_dist = 0.5)
    gui = ArmGUI(arm, astar)
