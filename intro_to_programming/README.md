
# Intro to Robotics Workshop 4: Intro to Programming
This folder contains the source code for the fourth workshop in the Intro to Robotics Workshop Series for the Robotics Club at the University of Rochester. Full setup instructions are given for both Windows and OSX machines.

## Setup Instructions (Windows)
These instructions include all programs, installs, commands, and steps to set up your computer for these workshops. If any steps don't work, try Googling the error you were given (how CS majors solve many issues) or email bhedegaa@u.rochester.edu to help us improve these materials for future use.

1. We need some environment to read and write code in. Any text editor could work, but one extra-helpful option we suggest for Windows is [VS Code](https://code.visualstudio.com/). Download the .exe and follow the installation process. If you'd prefer to use a different text editor, please be aware that you'll need to separately open a terminal window to run the commands you see below.
2. We need to install Git in order to use it in the command line. Visit [here](https://git-scm.com/download/win) and follow the installation process. During this process, you can select VS Code as your default editor for Git. Make sure to **allow Git from the commandline**. All other default settings should be fine.
3. Test if the Git install worked by opening VS Code and selecting `Terminal > New terminal` from the menu. Run the command `git`. You should see a bunch of options print out, which means that Git has successfully installed to your command line.
4. Install the VS Code Python extension by clicking the symbol with four boxes on the left of the screen and searching "Python" in the search bar. This extension will enable VS Code to highlight and parse Python's syntax.
5. We also need to give your overall computer the ability to interpret and run Python code. Visiting [python.org](https://www.python.org) and installing Python  3.8 for your operating system is a straightforward process. During the installation, make sure to select the "Add to PATH" option so that Python will be included whenever your computer opens a new terminal. Restart VS Code after this finishes.
6. To test if the Python interpreter properly installed, run `python` in a new terminal window. This will open the Python interpreter in the command line. You should see some information pop up and then a line starting with `>>>`. You can input Python code here, for example `1+1` or `print('Hello world')`. Entering `exit()` will quit this interpreter and bring you back to a normal command line.
7. You can now follow through the steps in the WS 4 slides. Note that the `cd`, `pwd`, and `mkdir` commands should work for you, while the `ls` command is instead `dir` on Windows. Practicing these commands in the VS Code terminal should generally work as the slides suggest, including the `git` steps which you should be able to execute as written in the slides.
8. Once you've finished the slides, open up the `ur_robotics_workshops` folder in a new VS Code window using `File > Open Folder`.
9. Then right click the `intro_to_programming` folder on the left side of the screen. Selecting "Open in integrated terminal" will open the terminal such that `intro_to_programming` is the current working directory. Running `dir` will show you what's in this folder.
10. Run `git clone https://github.com/striest/stepper_motors` to clone the code we'll need to interface with the virtual stepper motors for this and following workshops. Then run `pip install ./stepper_motors` to make this code 'visible' to the Python interpreter on your machine.
11. Run `pip install matplotlib` to install a Python graphing library that we'll use for various illustrations during these workshops. Also run `pip install pygame` to install another graphics package we'll use for a demo in the A* workshop. Thankfully, these installation steps cover all we'll need for the rest of the workshops. Whew.
12. We can now run the workshop code! Run `python program-name` to run your desired Python file, in this case `three_steppers_virtual.py`. This code will simulate stepper motors, as your computer doesn't have GPIO pins. Follow along with the prompts given to complete the practice Python exercises.

## Setup Instructions (Mac)
These instructions include all programs, installs, commands, and steps to set up your computer for these workshops. If any steps don't work, try Googling the error you were given (how CS majors solve many issues) or email bhedegaa@u.rochester.edu to help us improve these materials for future use.

1. We need some environment to read and write code in. Any text editor could work, but we suggest [Sublime Text](https://www.sublimetext.com/) for Mac due to its simplicity. After all, OSX already has the Terminal app for our command-line needs. Download the .dmg and follow the installation process.
2. We need to install Git in order to use it in the command line. Open the Terminal app and run the command `git`. You should see a prompt asking if you want to install developer tools, which basically handles this installation for us. Select OK and follow that installation process.
3. Test if the Git install worked by again inputting `git`. You should see a bunch of options print out, which means that Git has successfully installed to your command line.
5. We also need to give your overall computer the ability to interpret and run Python code. Visiting [python.org](https://www.python.org) and installing Python  3.8 for OSX is a straightforward process. Restart your Terminal window after this finishes.
6. To test if the Python interpreter properly installed, run `python3` in a new terminal window. This will open the Python interpreter in the command line. You should see some information pop up and then a line starting with `>>>`. You can input Python code here, for example `1+1` or `print('Hello world')`. Entering `exit()` will quit this interpreter and bring you back to a normal command line.
7. You can now follow through the steps in the workshop slides. Because Mac has a Unix command line, all commands in the slides will work for you. The above installation steps should enable you to execute the `git` steps in the slides.
8. Once you've finished the slides, navigate to the `ur_robotics_workshops` folder in Terminal and then into the `intro_to_programming` folder. Running `ls` will show you what's in this folder.
10. Run `git clone https://github.com/striest/stepper_motors` to clone the code we'll need to interface with the virtual stepper motors for this and following workshops.
11. Run `curl https://bootstrap.pypa.io/get-pip.py -o get-pip.py` and then `python get-pip.py` to install pip, a package for manager for Python. Then run `pip3 install stepper_motors` to make the stepper motor code 'visible' to the Python interpreter on your machine.
11. Run `pip install matplotlib` to install a Python graphing library that we'll use for various illustrations during these workshops.
12. We also need to install Pygame, another library we'll use for a demo in the A* workshop. Run `python3 -m pip install -U pygame==2.0.0.dev6 --user`. Thankfully, these installation steps cover all we'll need for the rest of the workshops. Whew.
12. We can now run the workshop code! Run `python3 program-name` to run the desired Python file, in this case `three_steppers_virtual.py`. This code will simulate stepper motors, as your computer doesn't have GPIO pins. Follow along with the prompts given to complete the practice Python exercises.

## What's in this directory
### three\_steppers.py
Source code for the workshop which will actuate physical stepper motors. Use this if you're on a Raspberry Pi.
### three\_steppers\_virtual.py
Source code for the workshop which will actuate virtual stepper motors. Use this on all non-Raspberry-Pi computers.

## What to implement
First, run either `three_steppers.py` or `three_steppers_virtual.py`, depending on your platform. Follow along with the prompts. You will eventually need to implement the code for parts 4-6 (lines 70, 147, 160) in either `three_steppers.py` or `three_steppers_virtual.py`, depending on what you've been running.

## How to tell if it's working
Re-run the script you ran after implementing the code for parts 4-6. The final results should look something like [this](https://youtu.be/eAWpkSIO3vE). TODO: This video is from WS 5, not 4.
