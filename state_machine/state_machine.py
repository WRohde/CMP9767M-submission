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
    print(weed_targets)

node_list =[]
def topological_map_callback(data):
    """
    updates the node_list with the names of nodes in the topological map
    """
    global node_list
    nodes = []
    for node in data.nodes:
        nodes.append(node.name)
    node_list = nodes

def callSprayService():
    rospy.wait_for_service('/thorvald_001/spray')
    try:
        callSpray = rospy.ServiceProxy('/thorvald_001/spray', Empty)
        return callSpray()
    except rospy.ServiceException:
        print 'Service call failed: %s' % e

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

#State machine states
def launch(_):
    #topological navigation fails if first node isn't 
    goal = GotoNodeGoal()
    goal.target = 'row_2_start'
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
        goal = GotoNodeGoal()
        goal.target = np.random.choice(node_list) #TODO more sensible node order.
        try:
            topological_navigation_client.send_goal(goal)
        except:
            pass
    
    #new state selection. 
    newState = 'WAITFORNEXTNODE'
    print('transistion to state:',newState)
    return(newState,_)

def wait_for_next_node(_):
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
    else:
        newState = 'WAITFORNEXTNODE'
    print('transistion to state:',newState)
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

def wait_for_weed():
    #if move_base has arrived at the weed transition to spray.
    if move_base_action_client.goal_status_check():
        newState = 'SPRAY'
    else:
        newState = 'WAITFORWEED'

    print('transistion to state:',newState)
    return(newState,_)

def spray(_): 
    callSprayService()
    newState = 'SETWEEDGOAL'
    print('transistion to state:',newState)
    return(newState,weed_pose)

#setting up the state machine  
thorvald_StateMachine = StateMachine() 
thorvald_StateMachine.add_state('LAUNCH',launch)
thorvald_StateMachine.add_state('SETNEXTGOALNODE',set_next_goal_node)
thorvald_StateMachine.add_state('WAITFORNEXTNODE',wait_for_next_node)
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
    weed_targets_sub = rospy.Subscriber('{}/weed_pose_array'.format(robot_name),PoseArray,weed_targets_callback)

    #publishers
    state_pub = rospy.Publisher('/{}/state'.format(robot_name),String,queue_size=0)

    #action clients
    move_base_action_client = ActionClientClass('/{}/move_base'.format(robot_name),MoveBaseAction)
    topological_navigation_client = ActionClientClass('/thorvald_001/topological_navigation',GotoNodeAction)

    #start state machine
    thorvald_StateMachine.run('')
    