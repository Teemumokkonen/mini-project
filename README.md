# mini-project
Fundamentals of mobile robotics mini project

## What this project contains

This projoct handles route calculation, path following and obstacle detection for turtlebot3 robot.
 
In this project is used premade turtlebot3 robot for ubuntu. 

### To run this project

You will need to have ros noetic installed and turtlebot3 package installed.


### To run needed files for this project

You must first run the turtlebot3 simulation by running following commands

Export the turtlebot3 model for the simulation:

export TURTLEBOT3_MODEL=burger 

Turn on the gazebo simulation environtment with right world for this project:

roslaunch turtlebot3_gazebo turtlebot3_world.launch

Turn on the rviz with simulation to give the robot instructions for the final pose:

roslaunch turtlebot3_gazebo turtlebot3_gazebo_rviz.launch

note(these files can be downloaded to any workspace to work)

### To run the route planning code 

extract the zip file to some ros workspace source directory inside a route_plan folder.

After this the file folder should look like this

route_plan/
├── CMakeLists.txt
├── package.xml
├── README.md
├── setup.py
└── src
    ├── main.py
    └── utils
        ├── __init__.py
        ├── path.py
        ├── robot.py
        └── simulation.py

after this you should build your workspace with:

catkin_make

and source the environment:

source devel/setup.bash

and to run the code you should use command 

rosrun route_plan main.py



