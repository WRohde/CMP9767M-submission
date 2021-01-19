#!/usr/bin/python

#python libraries
import sys
import numpy as np

#ROS libraries
import rospy
import tf

#ROS messages
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String

#ROS services
from std_srvs.srv import Empty

class move:
    weed_pose = None
    
    def __init__(self,robot_name,max_forward_speed=2,position_threshold=0.1):
        self.robot_name = robot_name
        self.max_forward_speed = max_forward_speed
        self.position_threshold = position_threshold

        rospy.init_node('translation_move_node', anonymous=True)

        #subscribers
        self.weed_pose_sub = rospy.Subscriber('/{}/weed_pose'.format(robot_name),PoseStamped, self.weed_pose_callback)
        self.transforms = tf.TransformListener()
        
        #publishers
        self.pub = rospy.Publisher("/{}/twist_mux/cmd_vel".format(robot_name),Twist,queue_size=0)

    def move_sprayer_to_position(self,req):
        """ This function will attempt to translate the robot to bring the sprayer to within self.position_threshold of weed_pose. """        
        cmd_vel_message = Twist()
        at_goal_flag_x,at_goal_flag_y = False,False

        # translate the robot so that it's sprayer is within position_threshold of the target
        while (at_goal_flag_x != True and at_goal_flag_y != True):
            
            # check if weed_pose has been updated from weed_pose_callback
            if self.weed_pose is None:
                continue

             # transform target to robot's sprayer.                                                                                        
            try:
                self.transforms.waitForTransform('/{}/base_link'.format(robot_name), self.weed_pose.header.frame_id,rospy.Time(),rospy.Duration(0.5))
                target_pose_WRT_spray = self.transforms.transformPose('/{}/sprayer'.format(robot_name), self.weed_pose)
                error_WRT_spray = np.array([target_pose_WRT_spray.pose.position.x,\
                        target_pose_WRT_spray.pose.position.y,\
                        target_pose_WRT_spray.pose.position.z])
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                return -1 #TODO find a better way of doing this.

            #move to target in x and stop within threshold distance of it.
            if abs(error_WRT_spray[0]) > self.position_threshold:                                                                                                                                                                                                         
                cmd_vel_message.linear.x = np.sign(error_WRT_spray[0]) * min(abs(error_WRT_spray[0]), self.max_forward_speed)
            else: at_goal_flag_x = True
            #move to target in y and stop within threshold distance of it.
            if abs(error_WRT_spray[1]) > self.position_threshold:
                cmd_vel_message.linear.y = np.sign(error_WRT_spray[1]) * min(abs(error_WRT_spray[1]), self.max_forward_speed)
            else: at_goal_flag_y = True
            self.pub.publish(cmd_vel_message)
            
        return 0
    
    def weed_pose_callback(self,data):
        """sets self.weed_pose to match the weed_pose topic"""
        self.weed_pose = data

if __name__ == '__main__':
    if(len(sys.argv)>1):
        robot_name = sys.argv[1]
    else:
        robot_name = "thorvald_001"

    move = move(robot_name)
    s = rospy.Service('/{}/moveSprayerToWeed'.format(robot_name), Empty, move.move_sprayer_to_position)
    rospy.spin()


