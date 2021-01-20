# CMP9767M-submission
Code for the CMP9767M assignment 1.

## 100 word summary of approach to the project

For this project the specialisation is in the deployment area. Specifically the generation of a topological map which fits the rows of crops and can be used to guide the robot from node to node for weeding. Currently the topological map is built on top of a pre-defined static map and is provided fake crop
detections.

The navigation for this project uses fake localisation, topological navigation and the DWAPlanner.
The vision component uses colour segmentation and clustering to find weed targets.
The behaviour of the robot is controlled using a smach state machine.

## Getting Started

* install LCAS ROS
* curl https://raw.githubusercontent.com/LCAS/rosdistro/master/lcas-rosdistro-setup.sh | bash -
* git clone the repo to the src folder of a catkin workspace. Run catkin_make from the root of the catkin_workspace.

## launching the code
* run source devel/setup.bash
* navigate to cmp9767m_submission/launch/ 
* launch the stack with roslaunch thorvald.launch 
* you can specify a topological map with the tmap:= argument. 
* if you would like to run the topological map generation script set generate_topo_map:=true Note: 

## useful topological_utils commands
* run rosrun topological_utils load_yaml_map.py $(rospack find thorvald_2dnav)/maps/topological_map.yaml -f


