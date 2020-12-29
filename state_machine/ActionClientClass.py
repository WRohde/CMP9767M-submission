#!/usr/bin/python
import rospy
import actionlib

class ActionClientClass:
    goalIDs = {'0':'PENDING','1':'ACTIVE','2':'SUCCEEDED','3':'SUCCEEDED','4':'ABORTED','5':'REJECTED','6':'PREEMPTING','7':'RECALLING','8':'RECALLED','9':'LOST'}
    
    def __init__(self,name,Action,timeout=rospy.Duration()):
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
        self.goalStatus = self.goalIDs['0']
        
        self.client = actionlib.SimpleActionClient(name, self.Action)
        self.client.wait_for_server(self.timeout) 

    def send_goal(self,goal):
        self.client.send_goal(goal,done_cb=self.goal_done_callback)

    def goal_results_check(self):
        """
        returns true if the result is available.
        """
        self.action_state = self.client.get_state()
        pass

    def get_result(self):
        return self.client.get_result()

    def goal_done_callback(self,goalStatus,goalResult):
        """
        callback triggered when goal action transitions to done.
        goalStatus is a integer corresponding with the states describe for the actionlib_msgs/GoalStatus.msg 
        message type, of these states only 2,3,4,5,8 are terminal states and 
        should be expected to occur here. goalResult is the result on reaching the done state. 
        """
        self.goalStatus = self.goalIDs[str(goalStatus)]
        return goalResult

