#!/usr/bin/python

#python libraries
import sys
import numpy as np
import threading

#ROS libraries
import rospy,actionlib
import smach

#ROS messages
from geometry_msgs.msg import PoseStamped, PoseArray
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from topological_navigation.msg import GotoNodeAction, GotoNodeGoal
from strands_navigation_msgs.msg import TopologicalMap
from std_msgs.msg import String

#ros services
from std_srvs.srv import Empty
from strands_navigation_msgs.srv import GetTags, GetTaggedNodes

#utils
from ActionClientClass import ActionClientClass

#--------------------
# subscriber callbacks
#--------------------

weed_targets = PoseArray()
def weed_targets_callback(data):
    """ updates the weed_targets PoseArray """
    global weed_targets
    weed_targets = data

node_list =[]
def topological_map_callback(data):
    """ updates the node_list with the nodes in the topological map."""
    global node_list
    node_list = data.nodes

current_node = ""
def current_node_callback(data):
    """updates global variable current_node to match topological map current_node message """
    global current_node
    current_node = data.data

#--------------------
# service calls
#--------------------

def callSprayService():
    rospy.wait_for_service('/thorvald_001/spray')
    try:
        callSpray = rospy.ServiceProxy('/thorvald_001/spray', Empty)
        return callSpray()
    except rospy.ServiceException as e:
        print('Spray Service call failed: ',e)

def detect_weeds_service():
    """
    calls image_processing.detect_weeds method from vision_node.py. 
    A PoseArray of weed_targets will be published in the /map frame to /robot_name/weed_pose_array  
    """
    rospy.wait_for_service('/thorvald_001/detect_weeds')
    try:
        callDetectWeeds = rospy.ServiceProxy('/thorvald_001/detect_weeds', Empty)
        return callDetectWeeds()
    except rospy.ServiceException as e:
        print('detect weeds Service call failed:',e)

def getTags():
    """
    returns the list of tags applied to nodes in the topological map.
    """
    rospy.wait_for_service('/topological_map_manager/get_tags')
    try:
        callGetTagsService = rospy.ServiceProxy('/topological_map_manager/get_tags', GetTags)
        return callGetTagsService().tags
    except rospy.ServiceException as e:
        print('getTags Service call failed:', e)

def getTaggedNodes(tag):
    """returns a list of nodes tagged with tag"""
    rospy.wait_for_service('/topological_map_manager/get_tagged_nodes')
    try:
        callGetTaggedNodesService = rospy.ServiceProxy('/topological_map_manager/get_tagged_nodes', GetTaggedNodes)
        return callGetTaggedNodesService(tag).nodes
    except rospy.ServiceException as e:
        print('getTaggedNodes service call failed: ',e)

def moveSprayerToWeed():
    """ moves sprayer to the published weedpose topic with translations"""
    rospy.wait_for_service('/thorvald_001/moveSprayerToWeed')
    try:
        callMoveSprayerToWeed= rospy.ServiceProxy('/thorvald_001/moveSprayerToWeed', Empty)
        return callMoveSprayerToWeed()
    except rospy.ServiceException as e:
        print('moveSprayerToWeed Service call failed:',e)

#--------------------
# Other functions
#--------------------

def getNextNodeWithTag(tag):
    """
    Checks the edges of node to see which destination nodes in the row haven't been visited. Returns the 
    first unvisited node from the edges , or a random unvisited node.
    """
    #look up goal_node in node_list
    for node in node_list:
        if node.name == goal_node:
            #check if any edges end at an unvisited node in the row.
            for edge in node.edges:
                if edge.node in current_row_unvisited_nodes:
                    #return first unvisited node in row
                    return edge.node
                else:
                    return np.random.choice(current_row_unvisited_nodes)

def getStartNodeForRow(rowtag):
    """returns a node tagged with 'start' and rowtag """
    return list(set(tagged_nodes_dict[rowtag]).intersection(tagged_nodes_dict['start']))[0]

#--------------------
# State machine states
#--------------------

#global variables for the state machine
tagged_nodes_dict = {}
goal_node = None
row_tags = None
current_row = None
current_row_unvisited_nodes = None

# define state Launch
class launch(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['SETNEXTGOALNODE'])

    def execute(self, userdata):
        rospy.loginfo('executing state LAUNCH')

        # get topological map tags. These are used to choose which node to move to.
        # the map must have tag "start", and a tag for each row in the format "row_n".
        tags = getTags()

        #initialising tagged_nodes_dict
        global tagged_nodes_dict
        for tag in tags:
            tagged_nodes_dict[tag] = getTaggedNodes(tag)
        
        #row_tags is used to ensure each row is visited once.
        global row_tags
        row_tags = tags
        #remove tags that shouldn't be included in row_tags
        try: row_tags.remove('start') 
        except: pass
        try: row_tags.remove('end') 
        except: pass
        #origin is used for the single node in the empty_map.yaml it should be ignored.
        try: row_tags.remove('origin') 
        except: pass

        #remove hard rows for computer vision specialisation
        try:
            row_tags.remove('row_4')
            row_tags.remove('row_5')
            row_tags.remove('row_2')
            row_tags.remove('row_1')
            row_tags.remove('row_0')
        except: pass

        #choose a row to harvest from row_tags
        global current_row
        global current_row_unvisited_nodes
        current_row = row_tags.pop(0)
        current_row_unvisited_nodes = getTaggedNodes(current_row)
        print("current_row target is",current_row)

        #set first goal_node for navigation
        global goal_node
        goal_node = getStartNodeForRow(current_row)
        print('first goal_node is:',goal_node)
        goal = GotoNodeGoal()
        goal.target = goal_node

        try:
            topological_navigation_client.send_goal(goal)
            # remove goal_node from unvisited nodes
            current_row_unvisited_nodes.remove(goal_node)
        except rospy.ROSException as e:
            print("failure sending goal to topologication navigation:",e)

        return('SETNEXTGOALNODE')

# define state SETNEXTGOALNODE
class set_next_goal_node(smach.State):
    """ In this state the next goal node in the topological map is set for the robot."""

    def __init__(self):
        smach.State.__init__(self, outcomes=['DETECTWEEDS','ENDSTATE'])
    
    def execute(self, userdata):
        rospy.loginfo('executing state SETNEXTGOALNODE')

        global goal_node
        global current_row
        global current_row_unvisited_nodes
        global row_tags

        #if the topological_navigation goalStatus is at a terminal state send a new goal.
        if topological_navigation_client.goal_status_check(): 
                
            #if all the nodes in the current row have been visited choose a new row.
            if len(current_row_unvisited_nodes) == 0:
                
                #get the next row from row_tags, or transition to end state if row_tags is empty
                try:
                    current_row = row_tags.pop(0)
                except:
                    return('ENDSTATE')

                #set goal_node to the node with both current_row and 'start' tags.
                goal_node = getStartNodeForRow(current_row)
                
            #otherwise set goal_node to the next node in the row.
            else:
                goal_node = getNextNodeWithTag(current_row)
                
            print('current_node is:',current_node,'next goal_node is:',goal_node,'targeting row:',current_row)
            
            #send topological navigation goal
            goal = GotoNodeGoal()
            goal.target = goal_node
            try:
                topological_navigation_client.send_goal(goal)
                # remove goal_node from unvisited nodes
                current_row_unvisited_nodes.remove(goal_node)
            except rospy.ROSException as e:
                print("failure sending goal to topologication navigation:",e)

        #return next state
        return('DETECTWEEDS')

# define state DETECTWEEDS
class detect_weeds_state(smach.State):
    """ calls the detect_weeds_service """
    def __init__(self):
        smach.State.__init__(self, outcomes=['SETWEEDGOAL'])

    def execute(self, userdata):
        rospy.loginfo('Executing state DETECTWEEDS')
        try:
            detect_weeds_service()
        except rospy.ServiceException as e:
            print('detect_weeds_service call failed:',e)
        return('SETWEEDGOAL')

# define state SETWEEDGOAL
class set_weed_goal(smach.State):
    """ Assigns one weed pose detected in DETECTWEEDS as a goal and moves the sprayer to it. """
    def __init__(self):
        smach.State.__init__(self, outcomes=['SPRAY','SETNEXTGOALNODE'])

    def execute(self, userdata):
        rospy.loginfo('Executing state SETWEEDGOAL')
        #get next weed_target 
        global weed_targets
        if len(weed_targets.poses) > 0:
            weed_pose = PoseStamped()
            weed_pose.header.frame_id = weed_targets.header.frame_id
            weed_pose.pose = weed_targets.poses.pop()

            # publish pose
            weed_pose_pub.publish(weed_pose)

            # move to published weed_pose
            try:
                moveSprayerToWeed()
            except rospy.ServiceException as e:
                print('moveSprayerToWeed call failed:',e)

            #move to spray state unless weed_targets is empty
            return('SPRAY')
        else:
            #once weed_targets is empty move to the next node
            return('SETNEXTGOALNODE')

# define state SPRAY        
class spray(smach.State): 
    """calls the spray service"""
    def __init__(self):
        smach.State.__init__(self, outcomes=['SETWEEDGOAL'])
    
    def execute(self,userdata):
        rospy.loginfo('Executing state SPRAY')
        callSprayService()
        return('SETWEEDGOAL')


if __name__ == '__main__':
    if(len(sys.argv)>1):
        robot_name = sys.argv[1]
        print(robot_name)
    else:
        robot_name = 'thorvald_001'

    rospy.init_node('{}_state_machine'.format(robot_name),anonymous=True)

    #subscribers
    topological_map_sub = rospy.Subscriber('/topological_map',TopologicalMap,topological_map_callback)
    weed_targets_sub = rospy.Subscriber('/{}/weed_pose_array'.format(robot_name),PoseArray,weed_targets_callback)
    current_node_sub = rospy.Subscriber('/{}/current_node'.format(robot_name),String,current_node_callback)

    #publishers
    weed_pose_pub = rospy.Publisher('/{}/weed_pose'.format(robot_name),PoseStamped,queue_size=0)

    #action clients
    move_base_action_client = ActionClientClass('/{}/move_base'.format(robot_name),MoveBaseAction)
    topological_navigation_client = ActionClientClass('/thorvald_001/topological_navigation',GotoNodeAction)

    # Create a SMACH state machine thorvald_sm
    thorvald_sm = smach.StateMachine(outcomes=['ENDSTATE'])

    # Open the state machine container
    with thorvald_sm:
        #add state to the state machine container 
        smach.StateMachine.add('LAUNCH',launch(),transitions={'SETNEXTGOALNODE':'SETNEXTGOALNODE'})
        smach.StateMachine.add('SETNEXTGOALNODE',set_next_goal_node(),transitions={'DETECTWEEDS':'DETECTWEEDS','ENDSTATE':'ENDSTATE'})
        smach.StateMachine.add('DETECTWEEDS',detect_weeds_state(),transitions={'SETWEEDGOAL':'SETWEEDGOAL'})
        smach.StateMachine.add('SETWEEDGOAL',set_weed_goal(),transitions={'SPRAY':'SPRAY','SETNEXTGOALNODE':'SETNEXTGOALNODE'})
        smach.StateMachine.add('SPRAY',spray(),transitions={'SETWEEDGOAL':'SETWEEDGOAL'})

    # Execute SMACH thorvald_sm
    outcome = thorvald_sm.execute() 