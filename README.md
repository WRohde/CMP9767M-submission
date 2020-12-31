# CMP9767M-submission
Code for the CMP9767M assignment 1.

## chosen specialisation.

## 100 word summary of approach to the project

The navigation for this project currently uses topological navigation and the DWAPlanner.


## how to install and run the code.

* install LCAS ROS
* git clone the repo to the src folder of a catkin workspace. Run catkin_make from the root of the catkin_workspace.
* navigate to the launch folder and launch the stack with roslaunch thorvald.launch
* run rosrun topological_utils load_yaml_map.py $(rospack find thorvald_2dnav)/maps/topological_map.yaml -f


