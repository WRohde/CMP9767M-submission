#!/usr/bin/python

#python libraries
import sys
import numpy as np

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
    except rospy.ServiceException:
        print 'Spray Service call failed: %s' % e

def detect_weeds_service():
    """
    calls image_processing.detect_weeds method from vision_node.py. 
    Publishes a PoseArray of weed_targets in the /map frame to /robot_name/weed_pose_array  
    """
    rospy.wait_for_service('/thorvald_001/detect_weeds')
    try:
        callDetectWeeds = rospy.ServiceProxy('/thorvald_001/detect_weeds', Empty)
        return callDetectWeeds()
    except rospy.ServiceException:
        print 'Service call failed: %s' % e

def getTags():
    """
    returns the list of tags applied to nodes in the topological map.
    """
    rospy.wait_for_service('/topological_map_manager/get_tags')
    try:
        callGetTagsService = rospy.ServiceProxy('/topological_map_manager/get_tags', GetTags)
        return callGetTagsService().tags
    except rospy.ServiceException:
        print('Service call failed: %s' % e)

def getTaggedNodes(tag):
    """returns a list of nodes tagged with tag"""
    rospy.wait_for_service('/topological_map_manager/get_tagged_nodes')
    try:
        callGetTaggedNodesService = rospy.ServiceProxy('/topological_map_manager/get_tagged_nodes', GetTaggedNodes)
        return callGetTaggedNodesService(tag).nodes
    except rospy.ServiceException:
        print('Service call failed: %s' % e)

def getNextNodeWithTag(tag):
    """
    Checks the edges of node to see which destination nodes have the tag. returns the first
    node found with tag, or None if none of the edges of currentnode have that tag.
    """
    #look up goal_node in node_list
    for node in node_list:
        if node.name == goal_node:
            #check if any edges end at a node with tag
            for edge in node.edges:
                if edge.node in tagged_nodes_dict[tag]:
                    #return first tagged node
                    return edge.node
    #returns None if no nodes with tag were in the edges of goal_node                     
    return None

#--------------------
# State machine states
#--------------------

#global variables for the state machine
tagged_nodes_dict = {}
goal_node = None
row_tags = None
current_row = None

# define state Launch
class launch(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['SETNEXTGOALNODE'])

    def execute(self, userdata):
        rospy.loginfo('executing state LAUNCH')

        # get topological map tags.
        # the map has tags "start","end", and a tag for each row in the format "row_n".
        tags = getTags()

        #initialising tagged_nodes_dict
        global tagged_nodes_dict
        for tag in tags:
            tagged_nodes_dict[tag] = getTaggedNodes(tag)
        
        #row_tags is used to ensure each row is visited once.
        global row_tags
        row_tags = tags
        row_tags.remove('start')
        row_tags.remove('end')
        #remove hard rows for computer vision specialisation
        row_tags.remove('row_4')
        row_tags.remove('row_5')
        #TODO sort row_tags so that row_tags.pop() gives rows in desired order with user-defined topological map
        row_tags.sort(reverse=True)

        #choose a row to harvest from row_tags
        global current_row
        current_row = row_tags.pop(0)
        print("current_row target is",current_row)

        #set first node to navigate to
        global goal_node
        goal_node = list(set(tagged_nodes_dict[current_row]).intersection(tagged_nodes_dict['start']))[0]
        print('first goal_node is:',goal_node)
        goal = GotoNodeGoal()
        goal.target = goal_node
        try:
            topological_navigation_client.send_goal(goal)
        except:
            pass #TODO exception or something.

        return('SETNEXTGOALNODE')

# define state SETNEXTGOALNODE
class set_next_goal_node(smach.State):
    """
    In this state the next goal node in the topological map is set for the robot .
    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['DETECTWEEDS','ENDSTATE'])
    
    def execute(self, userdata):
        rospy.loginfo('executing state SETNEXTGOALNODE')

        global goal_node
        global current_row
        global row_tags

        #if the topological_navigation goalStatus is at a terminal state send a new goal.
        if topological_navigation_client.goal_status_check(): 
                
            #if at the end of a row choose a new row and set goal_node to the node with 'start' tag.
            if current_node in tagged_nodes_dict['end']:
                
                #get the next row from row_tags, or transition to end state if row_tags is empty
                try:
                    current_row = row_tags.pop(0)
                except:
                    return('ENDSTATE')

                #set goal_node to the node with both current_row and 'start' tags TODO turn into a function
                goal_node = list(set(tagged_nodes_dict[current_row]).intersection(tagged_nodes_dict['start']))[0]
                
            #otherwise set goal_node to the next node in the row.
            else:
                goal_node = getNextNodeWithTag(current_row)

            print('current_node is:',current_node,'next goal_node is:',goal_node,'targeting row:',current_row)
            goal = GotoNodeGoal()
            goal.target = goal_node
            try:
                topological_navigation_client.send_goal(goal)
            except:
                pass #TODO this should probably raise an exception.
        
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
        except:
            pass #TODO this should probably raise an exception.
        return('SETWEEDGOAL')

# define state SETWEEDGOAL
class set_weed_goal(smach.State):
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

            #start moving to weed_pose
            weed_goal = MoveBaseGoal()
            weed_goal.target_pose = weed_pose
            move_base_action_client.send_goal(weed_goal)

            #move to spray state unless weed_targets is empty
            return('SPRAY')
        else:
            #once weed_targets is empty move to the next node
            return('SETNEXTGOALNODE')

# define state SPRAY        
class spray(smach.State): 
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
    state_pub = rospy.Publisher('/{}/state'.format(robot_name),String,queue_size=0)

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
    