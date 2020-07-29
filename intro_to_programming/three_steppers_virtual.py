# Author: Samuel Triest and Benned Hedegaard
# Last revised 7/7/2020

from time import sleep
from math import pi

from stepper_motors.virtual_stepper import VirtualStepper
from stepper_motors.stepper_array import StepperArray

def main():
    """Part 1: Working with one stepper motor"""
    print("\nHello! This entire file will guide you through some stepper motor examples.")
    input("When a code example is printed, press enter to run that code. (press enter to advance)")
    print("\n\n\nPart 1: Working with one stepper motor")
    
    input("s1 = VirtualStepper() # Creates an Stepper object named s1. This will open in a new tab.") # Waits for the user to input something.
    s1 = VirtualStepper() # Creates an Stepper object named s1. CHECK THE ACTUAL PINS (the lights on the motor driver should flash in sequence).
    input("print(type(s1)) # What should this print?")
    print(type(s1)) # What should this print?
    input("print(s1.angle) # Read the docs. What should this print?")
    print(s1.angle) # Read the docs. What should this print?
    

    """Part 2: Controlling the stepper"""
    print("\n\n\nPart 2: Controlling one stepper")
    input("s1.rotate_to(pi/2)")
    s1.rotate_to(pi/2)

    print("# Now flip back and forth once a second a few times.")
    input("s1.rotate_to(-pi/2)\nsleep(1)\ns1.rotate_to(pi/2)\nsleep(1)\ns1.rotate_to(-pi/2)\nsleep(1)\ns1.rotate_to(pi/2)")
    
    s1.rotate_to(-pi/2)
    sleep(1)
    s1.rotate_to(pi/2)
    sleep(1)
    s1.rotate_to(-pi/2)
    sleep(1)
    s1.rotate_to(pi/2)

    input("print(s1.angle) # What will this print now?")
    print(s1.angle) # What will this print now?
    
    
    """Part 3: More steppers"""
    print("\n\n\nPart 3: More steppers")
    
    # Just like before, we can create more Stepper objects connected to other pins.
    input("Just like before, we can create more Stepper objects connected to other pins.\ns2 = VirtualStepper()\ns3 = Stepper()")
    s2 = VirtualStepper()
    s3 = VirtualStepper()
    
    text = input("Please input an angle to set all three steppers to:")
    angle = int(text)
    
    input("s1.rotate_to(angle)\ns2.rotate_to(angle)\ns3.rotate_to(angle)")
    s1.rotate_to(angle)
    s2.rotate_to(angle)
    s3.rotate_to(angle)
    
    
    """Part 4: Make them dance"""
    print("\n\n\nPart 4: Make them dance")

    print("For the next part of this program, you'll need to edit the code! Open three_steppers.py and go to Part 4.")
    
    # Let's say I want all three servos to swap from -90 degrees to 
    # 90 degrees, every 2 seconds, four times.
    # Using the above examples, can you construct this behavior?
    
    ### YOUR CODE HERE ###

    return # Once you've implemented your version, REMOVE THIS LINE to allow the code to move forward.
    

    """Part 5: Renaming variables"""
    print("\n\n\nPart 5: Renaming variables")
    
    # We'll reassign the servos to different variables. Try to
    # track which of the servos will be the odd one out.
    
    # Q1: Direct assignment
    print("Q1: Direct assignment")
    input("s1.rotate_to(-pi/2)\ns2.rotate_to(-pi/2)\ns3.rotate_to(pi/2)")
    s1.rotate_to(-pi/2)
    s2.rotate_to(-pi/2)
    s3.rotate_to(pi/2)
    
    # Q2: Variable renaming
    print("Q2: Variable renaming")
    input("a = s1\nb = s2\nc = s3\na.angle = 90")
    a = s1
    b = s2
    c = s3
    
    a.angle = 90
    
    # Q3: More variable renaming
    print("Q3: More variable renaming")
    input("a = b\nb = c\nc = s1\n\ns1.rotate_to(pi/2)\ns2.rotate_to(-pi/2)\ns3.rotate_to(pi/2)")
    a = b
    b = c
    c = s1
    
    s1.rotate_to(pi/2)
    s2.rotate_to(-pi/2)
    s3.rotate_to(pi/2)
    
    # Q4: Variables upon variables
    print("Variables upon variables")
    input("i = b\nj = i\nk = a\ni = k\nk = c\n\ns1.rotate_to(-pi/2)\ns2.rotate_to(pi/2)\ns3.rotate_to(pi/2)")
    i = b
    j = i
    k = a
    i = k
    k = c
    
    s1.rotate_to(-pi/2)
    s2.rotate_to(pi/2)
    s3.rotate_to(pi/2)
    
    
    """Part 6: Working with lists"""
    print("\n\n\nPart 6: Working with lists")
    
    input("stepper_list = [s1, s2, s3] # Create a list with the three servos.")
    stepper_list = [s1, s2, s3] # Create a list with the three servos.
    
    input("stepper_list[0].rotate_to(pi/2)\nstepper_list[1].rotate_to(pi/2)\nstepper_list[2].rotate_to(pi/2)")
    stepper_list[0].rotate_to(pi/2)
    stepper_list[1].rotate_to(pi/2)
    stepper_list[2].rotate_to(pi/2)

    input("We have some more code for you to write. \nLook for Part 6 and fill in the function.")

    # Can you fill in the following function? It would make our lives much easier to set all three stepper motor angles at once.
    
    def set_steppers(steppers, angles):
        """Set the angles of multiple steppers at once.

        Note: The list could have zero, or more steppers in it, not just three. How can we handle this? (recall for loops?)
        
        Args:
            steppers (stepper_motors.Stepper): Steppers whose angles will be set.
            angles (list of integers): Angles to set the steppers to.
        """

        ### YOUR CODE HERE ###
        pass # Remove this once you've implemented your code.

    return # Also remove this line once you've implemented your code.
    
    input("set_steppers([s1, s2, s3], [-pi/2, -pi/2, -pi/2])")
    set_steppers([s1, s2, s3], [-pi/2, -pi/2, -pi/2])

    input("set_steppers([s1, s3], [pi/2, -pi/4])")
    set_steppers([s1, s3], [pi/2, -pi/4])

    input("One last coding exercise. Using your new function, can you recreate the dancing behavior from Part 4?\nNotice how much easier the job becomes.")
    
    ### YOUR CODE HERE ###

    print("And that's it. Congrats, we hope you learned something from these exercises.\nWe'll use this stepper motor interface during the next workshop to control our arm.")
    
if __name__ == "__main__":
    main()
