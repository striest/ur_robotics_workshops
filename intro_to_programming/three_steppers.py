# Author: Samuel Triest
# Last revised 6/22/2020

from time import sleep
from math import pi

from stepper_motors.stepper import Stepper
from stepper_motors.stepper_array import StepperArray

def main():
    """Part 1: Working with one servo"""
    
    s1 = Stepper(1, 2, 3, 4) # Creates an Stepper object named s1. CHECK THE ACTUAL PINS (the lights on the motor driver should flash in sequence).
    input() # Waits for the user to input something.
    
    print(type(s1)) # What should this print?
    print(s1.angle) # Read the docs. What should this print?
    
    
    
    """Part 2: Controlling a servo"""
    
    s1.rotate_to(pi/2)
    sleep(3)
    s1.rotate_to(-pi/2)
    sleep(3)
    s1.angle = rotate_to(pi/2)
    input()
    
    print(s1.angle) # What will this print now?
    
    
    
    """Part 3: More servos"""
    
    # Just like before, we can create more AngularServo objects
    # connected to other pins.
    s2 = Stepper(5, 6, 7, 8)
    s3 = Stepper(9, 10, 11, 12)
    
    text = input("Please input an angle to set all three servos to.")
    angle = int(text)
    
    s1.rotate_to(angle)
    s2.rotate_to(angle)
    s3.rotate_to(angle)
    
    
    
    """Part 4: Make them dance"""
    
    # Let's say I want all three servos to swap from -90 degrees to 
    # 90 degrees, every 2 seconds, four times.
    # Using the above examples, can you construct this behavior?
    
    ### YOUR CODE HERE ###
    
    
    
    """Part 5: Renaming variables"""
    
    # We'll reassign the servos to different variables. Try to
    # track which of the servos will be the odd one out.
    
    # Q1: Direct assignment
    input()
    s1.rotate_to(-pi/2)
    s2.rotate_to(-pi/2)
    s3.rotate_to(pi/2)
    
    # Q2: Variable renaming
    input()
    a = s1
    b = s2
    c = s3
    
    a.angle = 90
    
    # Q3: More variable renaming
    input()
    a = b
    b = c
    c = s1
    
    s1.rotate_to(pi/2)
    s2.rotate_to(-pi/2)
    s3.rotate_to(pi/2)
    
    # Q4: Variables upon variables
    i = b
    j = i
    k = a
    i = k
    k = c
    
    s1.rotate_to(-pi/2)
    s2.rotate_to(pi/2)
    s3.rotate_to(pi/2)
    
    
    
    """Part 5: Working with lists"""
    
    stepper_list = [s1, s2, s3]
    
    stepper_list[0].rotate_to(pi/2)
    stepper_list[1].rotate_to(pi/2)
    stepper_list[2].rotate_to(pi/2)
    input()
    
    #Removed this part because we set the stepper via a function
 
    # Can you fill in the following function? It would make our lives
    # much easier to set all three servos at once.
    
    def set_three_servos(servo1, servo2, servo3, angles):
        """Sets the angles of three servos at once.
        
        Args:
            servo1 (gpiozero.AngularServo): First servo.
            servo2 (gpiozero.AngularServo): Second servo.
            servo3 (gpiozero.AngularServo): Third servo.
            angles (list of 3 integers): Angles to set servos to.
        """
        pass # Replace this with your code.
    
    input()
    set_three_servos(s1, s2, s3, [-pi/2, -pi/2, -pi/2])
    
    # Using your new function, can you recreate the dancing behavior 
    # from Part 4? Notice how much easier the job becomes.
    
    ### YOUR CODE HERE ###
    
if __name__ == "__main__":
    main()
