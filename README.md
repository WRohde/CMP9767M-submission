# CMP9767M-submission

For this project the specialisation is in the deployment area. Specifically the generation of a topological map which fits the rows of crops and can be used to guide the robot from node to node for weeding. Currently the topological map is built on top of a pre-defined static map and is provided fake crop
detections.

The navigation for this project uses fake localisation, topological navigation and the DWAPlanner.
The vision component uses colour segmentation and clustering to find weed targets.
The behaviour of the robot is controlled using a smach state machine.

## Getting Started

* install LCAS ROS
* curl https://raw.githubusercontent.com/LCAS/rosdistro/master/lcas-rosdistro-setup.sh | bash -
* git clone the repo to the src folder of a catkin workspace. Run catkin_make from the root of the catkin_workspace.

## ros launch for topo map generation
* run source devel/setup.bash
* navigate to cmp9767m_submission/launch/ 
> roslaunch thorvald.launch generate_topo_map:=true

## ros launch for weed spraying
* run source devel/setup.bash
* navigate to cmp9767m_submission/launch/ 
> roslaunch thorvald.launch
* tmap argument you can specify a topological map with the tmap:= argument. It defaults to topological_map.  
    * a user-defined topological map
      > roslaunch thorvald.launch tmap:=topological_map
    * an example generated topological map
      > roslaunch thorvald.launch tmap:=generated

## useful topological_utils commands
* run rosrun topological_utils load_yaml_map.py $(rospack find thorvald_2dnav)/maps/topological_map.yaml -f


