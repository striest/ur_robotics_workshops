from math import pi

from stepper_motors.stepper import Stepper

class StepperArray:
    """
    Its an array of steppers. Commands to step and set angles are now given as arrays
    """

    def __init__(self, steppers):
        self.steppers = steppers


    def rotate_to(self, angles, degrees = False):
        """
        Rotates to the angle specified (chooses the direction of minimum rotation)
        """
        assert len(angles) == len(self.steppers), 'Expected {} angles for {} steppers bot got {}'.format(len(self.steppers), len(self.steppers), len(angles))
        targets = [angle * pi / 180 for angle in angles] if degrees else angles
        for s, t in zip(self.steppers, targets):
            s.rotate_to(t)

    def rotate_by(self, angles, degrees = False):
        """
        Rotate the stepper by this angle (radians unless specified)
        Positive angles rotate clockwise, negative angles rotate counterclockwise
        """
        assert len(angles) == len(self.steppers), 'Expected {} angles for {} steppers bot got {}'.format(len(self.steppers), len(self.steppers), len(angles))
        targets = [angle * pi / 180 for angle in angles] if degrees else angles
        for s, t in zip(self.steppers, targets):
            s.rotate_by(t)

    def zero(self):
        """
        Resets the position of the stepper to 0
        """
        for s in self.steppers:
            s.zero()

    def step_c(self):
        for s in self.steppers:
            s.step_c()

    def step_cc(self):
        for s in self.steppers:
            s.step_cc()

    def close(self):
        for s in self.steppers:
            s.close()

    def check(self):
        for s in self.steppers:
            s.check()
