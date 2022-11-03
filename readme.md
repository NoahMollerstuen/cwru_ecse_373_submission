# Lab 5  
Team 6  
Seongmin Jung, Noah Mollerstuen, Rafay Chaudhry  
Professor Lee  
ECSE 373  
Last modified: 3 November 2022  

---

## Ariac Environemnt Setup
This lab uses the 2019 ARIAC simulation environemnt developed by the Open Source Robotics Foundation. To find out more about the ARIAC environement, read the documentation [here](https://bitbucket.org/osrf/ariac/wiki/2019/Home).

### Run the simulation

To run the simulation, you will need this course's ARIAC package which can be found [here](https://github.com/cwru-eecs-373/ecse_373_ariac). However, there is a bug at the empy module for Python3. The first solution is runing

    roslaunch ecse_373_ariac ecse_373_ariac.launch python:=false &

to launch the simulation. The second solution is fixing the issue in the Python module directly.

    # Path the em.py file of the Python3 empy module.
    sudo patch /usr/lib/python3/dist-packages/em.py < `rospack find ecse_373_ariac`/patches/empy.patch

and then run the `roslaunch` command without python argument.  
If you still have an error, try the command below:

    sudo apt install update-alternatives
    update-alternatives --install /usr/bin/python python /usr/bin/python2.7 1
    update-alternatives --install /usr/bin/python python /usr/bin/python3 2

`ecse373_f22_team6_lab5` package is designed to interface with the ARIAC environment to control the simulation and locate parts which have been ordered. After installing and building the package, run `rosrun ecse373_f22_team6_lab5 ariac_interface` to start the node. You may need to unpause the Gazebo simulation. The node will print the location of the ordered part to the console. Plus, it will print the original and transformed version of `Pose`.
