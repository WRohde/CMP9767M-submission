#!/usr/bin/python

import rospy
import sys
import numpy as np
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import String
from std_srvs.srv import Empty

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
    
        while not rospy.is_shutdown():
            (newState, cargo) = handler(cargo)
            state_pub.publish(newState)
            if newState.upper() in self.endStates:
                print('reached ', newState)
                break 
            else:
                handler = self.handlers[newState.upper()]
            rospy.sleep(0.1)
            
#the target_position_list is a list of waypoints for the robot to navigate to.
#TODO change this to topological navigation of rows. Nice to have: autonomously generate topological nodes.
target_position_list = [[8,-8,0],[8,8,0],[5,-8,0],[5,8,0],[0,-8,0],[0,8,0],[-5,-8,0],[-5,8,0],[-8,8,0],[-8,-8,0]]
current_target = []

#greendetection callback.
green_detection = False
def green_detection_callback(data):
    global green_detection 
    if(data.data =='TRUE'):
        green_detection = True
    else:
        green_detection = False 

def callSprayService():
    rospy.wait_for_service('/thorvald_001/spray')
    try:
        callSpray = rospy.ServiceProxy('/thorvald_001/spray', Empty)
        return callSpray()
    except rospy.ServiceException, e:
        print 'Service call failed: %s' % e

#State machine states
def launch(_):
    newState = 'ROAM'
    return(newState,_)

def roam(_):
    """
    In this state the robot moves to positions from a list until it finds green in which case the system
    will transition to the GREENCLASSIFIER state.
    """
    global current_target
    global target_position_list
    
    target_position = PoseStamped()
    target_position.header.frame_id = '/{}/base_link'.format(robot_name)
    try:
        current_target = target_position_list.pop(0)
        target_position.pose.position = Point(current_target[0],current_target[1],current_target[2])
        goal_pub.publish(target_position)
    except:
        pass
    
    #new state selection. 
    if(green_detection == True):
        newState = 'GREENCLASSIFIER'
    else:
        newState = 'ROAM'
    return(newState,_)

def green_classifier(_):
    """
    TODO classification of detected green image.(Potentially combine this with the green detection.)
    TODO add logic to decide whether to spray.
    """
    if(True):
        newState = 'SPRAY'
    return(newState,_)


def spray(_):
    callSprayService()
    green_detection = False
    newState = 'ROAM'
    return(newState,_)

#setting up the state machine  
thorvald_StateMachine = StateMachine() 
thorvald_StateMachine.add_state('LAUNCH',launch)
thorvald_StateMachine.set_start('LAUNCH')
thorvald_StateMachine.add_state('ROAM',roam)
thorvald_StateMachine.add_state('GREENCLASSIFIER',green_classifier)
thorvald_StateMachine.add_state('SPRAY',spray)

if __name__ == '__main__':
    if(len(sys.argv)>1):
        robot_name = sys.argv[1]
        print(robot_name)
    else:
        robot_name = 'thorvald_001'

    rospy.init_node('{}_state_machine'.format(robot_name),anonymous=True)

    #subscribers
    green_detection_sub = rospy.Subscriber('/{}/green_detected'.format(robot_name),String,green_detection_callback)
    
    #publishers 
    goal_pub = rospy.Publisher('move_base_simple/goal',PoseStamped,queue_size=0) #TODO change to using actionlib for this
    state_pub = rospy.Publisher('/{}/state'.format(robot_name),String,queue_size=0)

    thorvald_StateMachine.run('')
    