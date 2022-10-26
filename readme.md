# Lab 5
## Ariac Environemnt Setup
This lab uses the 2019 ARIAC simulation environemnt developed by the Open Source Robotics Foundation. To find out more about the ARIAC environement, read the documentation [here](https://bitbucket.org/osrf/ariac/wiki/2019/Home).


To run the simulation, you will need this course's ARIAC package which can be found [here](https://github.com/cwru-eecs-373/ecse_373_ariac). After installing the package, run `roslaunch ecse_373_ariac ecse_373_ariac.launch python:=false &` to launch the simulation.

This package is designed to interface with the ARIAC environment to control the simulation and locate parts which have been ordered. After installing and building the package, run `rosrun ecse373_f22_team6_lab5 ariac_interface` to start the node. You may need to unpause the Gazebo simulation. The node will print the location of the ordered part to the console.
