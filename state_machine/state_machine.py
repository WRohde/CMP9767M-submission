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
        #if not self.endStates:
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
#TODO generate the list autonomously
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

#move_status callback
move_status = ""
def move_status_callback(data):
    global move_status
    move_status = data.data

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
    will transition to the GREENCLASSIFIER state, or detects a nearby collision object (in which case the
    system will transition to the PLAN state. 
    """
    global current_target
    global target_position_list
    
    target_position = PoseStamped()
    target_position.header.frame_id = '/{}/odom'.format(robot_name)
    if (move_status == 'WAITINGFORTARGET' or (move_status == 'ATGOAL'and len(target_position_list)>0)):
        current_target = target_position_list.pop(0)
        target_position.pose.position = Point(current_target[0],current_target[1],current_target[2])
        target_position_pub.publish(target_position)
    elif move_status == 'MOVING':
        target_position.pose.position = Point(current_target[0],current_target[1],current_target[2])
        target_position_pub.publish(target_position)
    
    #new state selection. 
    if(move_status == 'COLLISION'):
        newState = 'PLAN'
    elif(green_detection == True):
        newState = 'GREENCLASSIFIER'
    else:
        newState = 'ROAM'
    return(newState,_)

def plan(_):
    """
    #TODO plan route for robot considering collision objects.
    """
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
thorvald_StateMachine.add_state('PLAN',plan)
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
    move_status_sub = rospy.Subscriber("/{}/move_status".format(robot_name),String,move_status_callback)
    
    #target position publisher. The moving_thorvald node subscribes to these messages and the robot will navigate to the position given.
    target_position_pub = rospy.Publisher('/{}/target_position'.format(robot_name),PoseStamped,queue_size=0)
    state_pub = rospy.Publisher('/{}/state'.format(robot_name),String,queue_size=0)

    thorvald_StateMachine.run('')
    