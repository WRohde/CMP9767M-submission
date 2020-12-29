#!/usr/bin/python
import rospy
import actionlib


class ActionClientClass:
    def __init__(name,Action,timeout=rospy.Duration):
        """
        This class implements a SimpleActionClient, and provides a non-blocking method for monitoring progress
        with a goal.

        name should be a string
        Action should be the appropriate action message type for the action server
        Timeout should be a rospy duration 
        """
        self.Action = Action
        self.name = name
        self.timeout = timeout
        
        self.client = actionlib.SimpleActionClient(name, self.Action)
        self.client.wait_for_server(self.timeout) 

    def send_goal(goal):
        self.client.send_goal(goal)

    def goal_results_check():
        """
        returns true if the result is available.
        """
        self.action_state = self.client.get_state()
        pass

    def get_result():
        return self.client.get_result()

def move_base_SimpleActionClient(goal_pose_stamped, timeout=rospy.Duration(60)):
    """
    A simple action client for move_base navigation.
    #TODO consider making an ActionClientClass. 
    #TODO add error handling if server is not online.
    #TODO change to monitoring client.get_state instead of using client.wait_for_result to prevent blocking.
    """
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server(timeout) 

    #prepare goal and send to the action server
    goal = MoveBaseGoal()
    goal.target_pose = goal_pose_stamped
    client.send_goal(goal)

    client.wait_for_result(timeout) 
    return client.get_result()