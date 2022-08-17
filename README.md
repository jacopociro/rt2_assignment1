# REPOSITORY DESCRPTION
 This repository is divided in 6 branches: main, action, ros2, jupyter, doxygen and sphinx.
 Branches main, action and ros2 are the branches for the first assignment divided as such:
 - main: branch with coppelia simulation
 - action: branch with the ROSActoin implementation
 - ros2: branch with Ros2, Ros1 and Ros12_Bridge implementation
 Branches jupyter, doxygen and sphinx are for the second assignment, divided as such:
 - jupyter: branch whit notebook implementaition
 - doxygen: branch with doxygen documentation
 - sphinx: branch with sphinx documentation

# DESCRIPTION OF CONTENT OF THE PACKAGE
The package is composed of:
- Four nodes: 
    - go_to_point.py
    - user_interface.py
    - position_service.cpp
    - state_machine.cpp
- Three custom services:
    - Command.srv
    - Position.srv
    - RandomPosition.srv
- One launch file:
    - sim.launch
- One VREP scene:
    - Coppelia.ttt

## NODES
### go_to_point.py
Implements a service (Position.srv) that, when called, requests a position , runs the algorithm to control the robot and return a boolean true value as response. The algorithm reads from /odom the current position and orientation and, knowing the goal, calculates the linear and angular speed to send on the /cmd_vel topic. All of this is done with a state machine that has three states.
### user_interface.py
Implements the call for a service (Command.srv) for a user input and then, when it is received, launches the server in go_to_point.py. When the user hits the input for stopping the robot, the robot will reach the last goal received and then will stop.
### position_service.cpp
Implements a service (RandomPosition.srv) that, given a range, returns a random position for the robot to reach.
### state_machine.cpp
This is the node that calls both the services in go_to_point.py and position_service.cpp and the service that is called in user_interface.py. This node waits for user_interface.py to tell it to call the other two services and start the robot.

## VREP SIMULATION
This package is uses a  VREP simulation instead of gazebo. The file Coppelia.ttt describes the scene and the script linked to the robot implements the movement of the robot based on the nodes that run on the rosmaster.

# HOW TO RUN THE PACKAGE
Open up two terminals. In the first one you will need to run the roscore and the nodes using the launch file:
```
roslaunch rt2_assignment1 sim.launch
```
After this is done, and it is very important to run the roscore before opening Coppelia, you will have to move into your coppelia folder (or wherver your coppelia launcher is) and run:
```
cd CoppeliaSim_Edu_V4_2_0_Ubuntu20_04 (skip this if you are already in the right folder)
./coppeliaSim.sh
```
Now you need to open the correct Coppelia scene, called Coppelia.ttt, wait for it to load and then input 1 on the terminal waiting for you. 
Press 0 to stop the robot.
# ROBOT BEHAVIOUR
The robot spawns in (0.0) and stays still until the user input tells it to move. It will then start moving to a random position and when it reaches this position will start going to a new random position until the user tells it to stop.
# SYSTEM ARCHITECTURE
