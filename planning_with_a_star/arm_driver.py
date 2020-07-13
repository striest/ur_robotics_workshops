from math import pi

from stepper_motors.stepper import Stepper

from arm import Arm
from link import Link, FixedLink
from arm_astar import ArmAStar
from arm_gui import ArmGUI

if __name__ == '__main__':
    l0 = FixedLink(length = 0, angle = pi/2)
    l1 = Link(length = 5.5, angle = 0, min_angle = -1e4, max_angle = 1e4)
    l2 = Link(length = 5.9, angle = 0, min_angle = -1e4, max_angle = 1e4)
    l3 = Link(length = 4.0, angle = 0, min_angle = -1e4, max_angle = 1e4)
    l4 = Link(length = 2.0, angle = 0, min_angle = -1e4, max_angle = 1e4)
    l5 = Link(length = 2.0, angle = 0, min_angle = -1e4, max_angle = 1e4)
    l6 = Link(length = 2.0, angle = 0, min_angle = -1e4, max_angle = 1e4)
    l7 = FixedLink(length = 2.0, angle = pi/2)

#    s2 = Stepper(24, 25, 8, 7, delay = 5e-3)
#    s1 = Stepper(2, 3, 4, 17, delay = 5e-3)
#    s3 = Stepper(27, 22, 10, 9, delay = 5e-3)
#    s1.reverse()
#    s3.reverse()

#    arm = Arm([l0, l1, l2, l3], [s1, s2, s3])
    arm = Arm([l0, l1, l2, l3, l4, l5, l7, l6])

    print(arm)
    astar = ArmAStar(arm, discretization = 3*pi/180, min_dist = 0.2)
    gui = ArmGUI(arm, astar)
