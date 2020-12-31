#!/usr/bin/python
import sys
import rospy
from strands_navigation_msgs.srv import AddNode
from geometry_msgs.msg import PoseStamped,Pose

def addToTopoMap(name,pose):
    rospy.wait_for_service('/topological_map_manager/add_topological_node')
    try:
        callService = rospy.ServiceProxy('/topological_map_manager/add_topological_node', AddNode)
        return callService(name,pose,False)
    except rospy.ServiceException:
        print 'Service call failed: %s' % e

current_pose = Pose()
waiting_for_pose = True
def pose_callback(data):
    global current_pose
    global waiting_for_pose
    waiting_for_pose = False
    current_pose = data

if __name__ == '__main__':
    node_name = sys.argv[1]
    
    rospy.init_node('add_to_topo_map',anonymous=True)
    #subscribers
    pose_sub = rospy.Subscriber('/thorvald_001/robot_pose',Pose,pose_callback)

    #give some time for a pose to be published
    while waiting_for_pose == True:
        rospy.sleep(0.1)
    print(current_pose)
    addToTopoMap(node_name,current_pose)

