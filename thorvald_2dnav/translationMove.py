#!/usr/bin/python

import sys
import rospy
import numpy as np
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import Twist, Pose, PoseStamped, Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
import tf

#ros services
from std_srvs.srv import Empty

class move:
    weed_pose = None
    
    def __init__(self,robot_name,min_distance=2,max_forward_speed=2,publish_closest_collision=True):
        """
        robot_name should be in the format "thorvald_001", "thorvald_002", etc so that topics are published correctly
        min_distance is the minimum distance between the robot and an obstacle before it turns to avoid.
        """

        self.robot_name = robot_name
        self.min_distance = min_distance
        self.max_forward_speed = max_forward_speed

        #subscribers
        self.scan_sub = rospy.Subscriber("/{}/scan".format(robot_name), LaserScan, self.laserscan_subscriber_callback)
        self.weed_pose_sub = rospy.Subscriber('/{}/weed_pose'.format(robot_name),PoseStamped, self.weed_pose_callback)
        self.transforms = tf.TransformListener()
        
        #publishers
        self.pub = rospy.Publisher("/{}/twist_mux/cmd_vel".format(robot_name),Twist,queue_size=0)
        
        #closest_collision message is a point indicating location of closest collision for Rviz
        self.closest_collision_publisher = rospy.Publisher("/{}/closest_collision".format(robot_name), PoseStamped,queue_size=0)

    def move_sprayer_to_position(self,rq):
        """ This function will attempt to move the sprayer to within a threshold of a posestamped position and stop there. """        
        position_threshold = 0.1
        angular_threshold = np.pi/8
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
            if abs(error_WRT_spray[0]) > position_threshold:                                                                                                                                                                                                         
                cmd_vel_message.linear.x = np.sign(error_WRT_spray[0]) * min(abs(error_WRT_spray[0]), self.max_forward_speed)
            else: at_goal_flag_x = True
            #move to target in y and stop within threshold distance of it.
            if abs(error_WRT_spray[1]) > position_threshold:
                cmd_vel_message.linear.y = np.sign(error_WRT_spray[1]) * min(abs(error_WRT_spray[1]), self.max_forward_speed)
            else: at_goal_flag_y = True
            self.pub.publish(cmd_vel_message)
            
        return 0
    
    def weed_pose_callback(self,data):
        """sets self.weed_pose to match the weed_pose topic"""
        self.weed_pose = data


    def laserscan_subscriber_callback(self,data):
        """
        processes the laserscan data and checks for nearby collisions

        laserscan message type http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/LaserScan.html
        For the Thorvald Robot the laserscanner is positioned so that it sweeps from -pi/2 to pi/2
        """

        min_scan_distance = min(data.ranges)

        #find the polar coord for the minimum distance
        min_scan_distance = min(data.ranges)
        min_scan_distance_angle = data.angle_increment * data.ranges.index(min_scan_distance) - data.angle_min

        #convert to a pose WRT /robot_name/hokuyo frame
        pose = PoseStamped()
        pose.header.frame_id = data.header.frame_id
        pose.pose.position.x = min_scan_distance * np.cos(min_scan_distance_angle)
        pose.pose.position.y = min_scan_distance * np.sin(min_scan_distance_angle)
        #convert angle to quaternion
        r = Rotation.from_euler('z', min_scan_distance_angle)
        min_scan_distance_quaternion = r.as_quat()
        pose.pose.orientation.x = min_scan_distance_quaternion[0]
        pose.pose.orientation.y = min_scan_distance_quaternion[1]
        pose.pose.orientation.z = min_scan_distance_quaternion[2]
        pose.pose.orientation.w = min_scan_distance_quaternion[3]

        #publish pose
        self.closest_collision_publisher.publish(pose)  

if __name__ == '__main__':
    if(len(sys.argv)>1):
        robot_name = sys.argv[1]
    else:
        robot_name = "thorvald_001"

    rospy.init_node('translation_move_node', anonymous=True)
    move = move(robot_name)
    s = rospy.Service('/{}/moveSprayerToWeed'.format(robot_name), Empty, move.move_sprayer_to_position)
    rospy.spin()


