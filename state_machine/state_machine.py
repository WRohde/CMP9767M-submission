#!/usr/bin/python

#python libraries
import sys
import numpy as np

#ROS libraries
import rospy,actionlib

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

class StateMachine:
    """
    state machine class from the example here: https://www.python-course.eu/finite_state_machine.php
    Modified to better fit this ROS application
    """
    def __init__(self):
        self.handlers = {}
        self.startState = None
        self.endStates = []

    def add_state(self, name, handler, end_state=0):
        name = name.upper()
        self.handlers[name] = handler
        if end_state:
            self.endStates.append(name)

    def set_start(self, name):
        self.startState = name.upper()

    def run(self, cargo):
        try:
            handler = self.handlers[self.startState]
        except:
            raise InitializationError('must call .set_start() before .run()')
        #if not self.endStates: #TODO decide whether it is important to have an end_state e.g. is it good practice to not ctrl-c out?
        #    raise  InitializationError('at least one state must be an end_state')

        r = rospy.Rate(5) #5 hz
        while not rospy.is_shutdown():
            (newState, cargo) = handler(cargo)
            state_pub.publish(newState)
            if newState.upper() in self.endStates:
                print('reached ', newState)
                break 
            else:
                handler = self.handlers[newState.upper()]
            r.sleep()

weed_targets = PoseArray()
def weed_targets_callback(data):
    """
    updates the weed_targets PoseArray 
    """
    global weed_targets
    weed_targets = data

node_list =[]
def topological_map_callback(data):
    """
    updates the node_list with the nodes in the topological map.
    """
    global node_list
    node_list = data.nodes

current_node = ""
def current_node_callback(data):
    global current_node
    current_node = data.data

# service calls

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
    rospy.wait_for_service('/topological_map_manager/get_tagged_nodes')
    try:
        callGetTaggedNodesService = rospy.ServiceProxy('/topological_map_manager/get_tagged_nodes', GetTaggedNodes)
        return callGetTaggedNodesService(tag).nodes
    except rospy.ServiceException:
        print('Service call failed: %s' % e)

def getNextNodeWithTag(tag,currentnode):
    """
    Checks the edges of currentnode to see which destination nodes have the tag. returns the first
    node found with tag, or None if none of the edges of currentnode have that tag.
    """
    tagged_nodes = getTaggedNodes(tag)

    #look up currentnode in node_list
    for node in node_list:
        if node.name is currentnode:
            #check if any edges end at a node with tag
            for edge in node.edges:
                if edge.node in tagged_nodes:
                    #return first tagged node
                    return edge.node
    return None


#State machine states

tagged_nodes_dict = {}
row_tags = None
current_row = None
def launch(_):

    #the topological map has tags "start","end", and a tag for each row in the format "row_n"
    #where n is the row number.
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

    #choose a row to harvest from
    global current_row
    current_row = row_tags.pop()
    print("current_row target is",current_row)

    #set first goal.
    goal_node = list(set(tagged_nodes_dict[current_row]).intersection(tagged_nodes_dict['start']))[0]
    goal = GotoNodeGoal()
    goal.target = goal_node
    try:
        topological_navigation_client.send_goal(goal)
    except:
        pass

    newState = 'SETNEXTGOALNODE'
    print('transistion to state:',newState)
    return(newState,_)

def set_next_goal_node(_):
    """
    In this state the next goal node in the topological map is set for the robot .
    """
    #if the topological_navigation goalStatus is at a terminal state send a new goal.
    if topological_navigation_client.goal_status_check(): 
        #if at the end of a row choose a new row and navigate to the start of it.
        if current_node in tagged_nodes_dict['end']:
            global current_row
            global row_tags
            current_row = row_tags.pop()
            #TODO this line is a massive hack
            goal_node = list(set(tagged_nodes_dict[current_row]).intersection(tagged_nodes_dict['start']))[0]
            print('goal_node')
        
        #otherwise move to the next node in the row.
        else:
            goal_node = getNextNodeWithTag(current_row,current_node)

        goal = GotoNodeGoal()
        goal.target = goal_node
        try:
            topological_navigation_client.send_goal(goal)
        except:
            pass
    
    #new state selection. 
    newState = 'WAITFORNEXTGOALNODE'
    print('transistion to state:',newState)
    return(newState,_)

def wait_for_next_goal_node(_):
    """
    In this state the robot travels to the next goal node in the topological map.
    """
    #if the topological_navigation goalStatus is at a terminal state transition to DETECTWEEDS
    if topological_navigation_client.goal_status_check(): 
        #TODO check if node is on a crop row. if true check for weeds if false move to next node.
        if True:
            newState = 'DETECTWEEDS'
        else:
            newState = 'SETNEXTGOALNODE'
        print('transistion to state:',newState)
    else:
        newState = 'WAITFORNEXTGOALNODE'
    return(newState,_)

def detect_weeds_state(_):
    """
    calls the detect_weeds_service
    """
    try:
        detect_weeds_service()
    except:
        pass
    newState = 'SETWEEDGOAL'
    print('transistion to state:',newState)
    return(newState,_)

def set_weed_goal(_):
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

        #stay in spray state unless weed_targets is empty
        newState = 'WAITFORWEED'
    else:
        newState = 'SETNEXTGOALNODE'
    print('transistion to state:',newState)
    return(newState,_)

def wait_for_weed(_):
    #if move_base has arrived at the weed transition to spray.
    if move_base_action_client.goal_status_check():
        newState = 'SPRAY'
        print('transistion to state:',newState)
    else:
        newState = 'WAITFORWEED'
    return(newState,_)

def spray(_): 
    callSprayService()
    newState = 'SETWEEDGOAL'
    print('transistion to state:',newState)
    return(newState,_)

#setting up the state machine  
thorvald_StateMachine = StateMachine() 
thorvald_StateMachine.add_state('LAUNCH',launch)
thorvald_StateMachine.add_state('SETNEXTGOALNODE',set_next_goal_node)
thorvald_StateMachine.add_state('WAITFORNEXTGOALNODE',wait_for_next_goal_node)
thorvald_StateMachine.add_state('DETECTWEEDS',detect_weeds_state)
thorvald_StateMachine.add_state('SETWEEDGOAL',set_weed_goal)
thorvald_StateMachine.add_state('WAITFORWEED',wait_for_weed)
thorvald_StateMachine.add_state('SPRAY',spray)
thorvald_StateMachine.set_start('LAUNCH')

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

    #start state machine
    thorvald_StateMachine.run('')
    