# Author: Benned Hedegaard
# Last revised 6/16/2020

from gpiozero import AngularServo
from time import sleep

def main():
    """Part 1: Working with one servo"""
    
    s1 = AngularServo(5) # Creates an AngularServo named s1.
    input() # Waits for the user to input something.
    
    print(type(s1)) # What should this print?
    print(s1.angle) # Read the docs. What should this print?
    
    
    
    """Part 2: Controlling a servo"""
    
    s1.angle = 90
    sleep(3)
    s1.angle = -90
    sleep(3)
    s1.angle = 90
    input()
    
    print(s1.angle) # What will this print now?
    
    
    
    """Part 3: More servos"""
    
    # Just like before, we can create more AngularServo objects
    # connected to other pins.
    s2 = AngularServo(2)
    s3 = AngularServo(3)
    
    text = input("Please input an angle to set all three servos to.")
    angle = int(text)
    
    s1.angle = angle
    s2.angle = angle
    s3.angle = angle
    
    
    
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
    s1.angle = -90
    s2.angle = -90
    s3.angle = 90
    
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
    
    a.angle = 90
    b.angle = -90
    c.angle = 90
    
    # Q4: Variables upon variables
    i = b
    j = i
    k = a
    i = k
    k = c
    
    i.angle = -90
    j.angle = 90
    k.angle = 90
    
    
    
    """Part 5: Working with lists"""
    
    servo_list = [s1, s2, s3]
    
    servo_list[0].angle = 90 # Note that lists start with index 0.
    servo_list[1].angle = 90
    servo_list[2].angle = 90
    input()
    
    """Part 6: Function writing"""
    
    def set_servo(servo, a):
        """Sets the angle of the given AngularServo to a.

        Args:
            servo (gpiozero.AngularServo): Servo to set.
            a (integer): Angle to set the servo to.
        """
        servo.angle = a
        
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
    set_three_servos(s1, s2, s3, [-90, -90, -90])
    
    # Using your new function, can you recreate the dancing behavior 
    # from Part 4? Notice how much easier the job becomes.
    
    ### YOUR CODE HERE ###
    
if __name__ == "__main__":
    main()
