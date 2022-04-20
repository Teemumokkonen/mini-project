# mini-project
Fundamentals of mobile robotics mini project

## What this project contains

This projoct handles route calculation, path following and obstacle detection for turtlebot3 robot.
 
In this project is used premade turtlebot3 robot for ubuntu. 

### To run this project

You will need to have ros noetic installed and turtlebot3 package installed.

### To run this project

You must first run the turtlebot3 simulation by running following commands

Export the turtlebot3 model for the simulation:

export TURTLEBOT3_MODEL=burger 

Turn on the gazebo simulation environtment with right world for this project:

roslaunch turtlebot3_gazebo turtlebot3_world.launch

Turn on the rviz with simulation to give the robot instructions for the final pose:

roslaunch turtlebot3_gazebo turtlebot3_gazebo_rviz.launch
