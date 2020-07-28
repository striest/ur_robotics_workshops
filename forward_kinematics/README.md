# Intro to Robotics Workshop 5: Forward Kinematics
This is the source code for the fifth workshop in the Intro to Robotics Workshop Series for the Robotics Club at the University of Rochester.
## What's in this directory:
### link.py
Source code for a two-dimensional link in an arm with an adjustable angle, as well as a version with a fixed joint.
### arm.py
Source code for a two-dimensional arm, notably without code for forward kinematics.
### gui.py
Source code for visualizing and changing the joint angles of a 2DOF 2d arm. This file can be run as a script to check if forward kinematics were inplemented correctly.
### answer.txt
Forward kinematics code that can be copy-pasted into line 58 of arm.py to do forward kinematics. Only look at this if you're stuck, as writing this code is essentially the entire workshop.
## What you have to implement:
Forward kinematics is done in the function `get_joint_poses` defined in line 44 of arm.py. As the goal of this workshop is to teach forward kinematics, the actual code has been omitted for you to implement yourself (remember to remove the `pass` keyword from the function). This function should return a (n+1)x3 numpy array where each row is the pose (x, y, theta) of the origin of the n-th link of the arm. The (n+1)-th row is the pose of the end-effector. Line 60 creates the appropriate numpy array, so the code you write should focus on computing the pose of the endpoints of each link and appending them to `poses`.
## How to tell if it's working:
You can run `python3 gui.py` to pull up an interactive window that lets you move around the arm's pose in joint space and see how it affects the arm in config space. The final results should look something like this (TODO make demo video). 
