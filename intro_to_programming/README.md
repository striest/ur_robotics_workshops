
Make sure to clone the stepper_motors repository into this folder before running the workshop examples.
That code can be found at: https://github.com/striest/stepper_motors

-Benned

# Intro to Robotics Workshop 4: Intro to Programming
This is the source code for the fourth workshop in the Intro to Robotics Workshop Series for the Robotics Club at the University of Rochester.
## Note:
The code in this workshop relies on a python package for stepper motors which can be found [here](https://github.com/striest/stepper_motors). Clone this repository onto your local machine and enter the source directory in the terminal of your choice. Then run: `pip3 install stepper_motors`. This will install the package onto your machine. You can verify this by running `python3 -c 'import stepper_motors'` in your terminal. This should run without giving a `ModuleNotFoundError`. From there, you can continue the workshop by running `python3 three_steppers.py` if you are in Raspberry Pi and have steppers wired, else run `python3 three_steppers_virtual.py` to complete the workshop with virtual steppers.
## What's in this directory:
### three_steppers.py
Source code for the workshop which will actuate physical stepper motors.
### three_steppers_virtual.py
Source code for the workshop which will actuate virtual stepper motors.
## What you have to implement:
First, run either `three_steppers.py` or `three_steppers_virtual.py`, depending on if you have access to physical stepper motors and follow the prompts. You then need to implement the code for parts 4-6 (lines 70, 147, 160) in either `three_steppers.py` or `three_steppers_virtual.py`, depending on what you've been running.
## How to tell if it's working:
Re-run the script you ran after implementing the code for parts 4-6. The final results should look something like [this](https://youtu.be/eAWpkSIO3vE) TODO: make video for ws 4, not 5. 
