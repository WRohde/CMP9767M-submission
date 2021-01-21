# CMP9767M-submission

For this project the specialisation is in the deployment area. Specifically the generation of a topological map which fits the rows of crops and can be used to guide the robot from node to node for weeding. Currently the topological map is built on top of a pre-defined static map and is provided fake crop
detections.

The navigation for this project uses fake localisation, topological navigation and the DWAPlanner.
The vision component uses colour segmentation and clustering to find weed targets.
The behaviour of the robot is controlled using a smach state machine.

## Getting Started

* install LCAS ROS
> curl https://raw.githubusercontent.com/LCAS/rosdistro/master/lcas-rosdistro-setup.sh | bash -\
> sudo apt-get install \\ \
>  ros-melodic-gmapping \\ \
>  ros-melodic-topological-utils \\ \
>  ros-melodic-robot-pose-publisher
>  ros-melodic-robot-localization \\ \
>  ros-melodic-topological-navigation \\ \
>  ros-melodic-amcl \\ \
>  ros-melodic-fake-localization \\ \
>  ros-melodic-smach
* git clone the repo to the src folder of a catkin workspace. Run catkin_make from the root of the catkin_workspace.

## ros launch for topo map generation
* run source devel/setup.bash
* navigate to cmp9767m_submission/launch/ 
* run the launch command below. Replace the tmap arg with your desired pointset name. 
> roslaunch thorvald.launch generate_topo_map:=true tmap:=custom_generated_map
* after the print out shows generate_topo_map is complete you can run the following command to see the new map in rviz: 
> rosrun topological_utils topological_map_update.py 
* ctrl-c out of ros launch

## ros launch for weed spraying
* run source devel/setup.bash
* navigate to cmp9767m_submission/launch/ 
* ensure you have maps in the database by following either "Adding example maps to database" or "ros launch for topo map generation"
* run the launch command below. Replace the tmap arg with your desired pointset name.  It defaults to topological_map.  
> roslaunch thorvald.launch tmap:=generated
  * a user-defined topological map
  > roslaunch thorvald.launch tmap:=topological_map
  * an example generated topological map
  > roslaunch thorvald.launch tmap:=generated

## Adding example maps to database
* run source devel/setup.bash
* navigate to cmp9767m_submission/launch/ 
> roslaunch thorvald.launch
* you will see an error in the logs as the expected topological map can't be found. You can run the following to load the maps currently in the repo.
> rosrun topological_utils load_yaml_map.py $(rospack find thorvald_2dnav)/maps/topological_map.yaml 
> rosrun topological_utils load_yaml_map.py $(rospack find thorvald_2dnav)/maps/empty_map.yaml 
> rosrun topological_utils load_yaml_map.py $(rospack find thorvald_2dnav)/maps/generated_example.yaml 
* the mapserver stores only one map with each pointset name, if you wish to load a map with a different pointset name use the --pointset arg e.g.
> rosrun topological_utils load_yaml_map.py $(rospack find thorvald_2dnav)/maps/empty_map.yaml --pointset custom_empty_map

## useful topological_utils commands for debugging
* To load yaml map 
> rosrun topological_utils load_yaml_map.py filepath/map.yaml 
* Update topo map if you can't see it in rviz when you think you should:
> rosrun topological_utils topological_map_update.py 
* list maps in database
> rosrun topological_utils list_maps
* remove a map from the database
> rosrun topological_utils rm_map_from_db.py your_pointest_name
* saving a map from the database to yaml
> rosrun topological_utils map_to_yaml.py your_pointset_name filepath/mod.yaml



