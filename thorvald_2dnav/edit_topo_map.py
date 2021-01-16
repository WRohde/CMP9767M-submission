#!/usr/bin/python

#python libraries
import sys
import numpy as np

#ros lbraries
import rospy

#ros message and service types
from strands_navigation_msgs.srv import AddNode, AddEdge, AddTag, GetTags, GetEdgesBetweenNodes, GetTaggedNodes
from geometry_msgs.msg import PoseStamped,Pose
from strands_navigation_msgs.msg import TopologicalMap

topological_map = None
def topological_map_callback(data):
    """
    updates topologica_map global variable.
    """
    global topological_map
    topological_map = data

def addToTopoMap(name,pose):
    rospy.wait_for_service('/topological_map_manager/add_topological_node')
    try:
        callAddTopoMapService = rospy.ServiceProxy('/topological_map_manager/add_topological_node', AddNode)
        return callAddTopoMapService(name,pose,False)
    except rospy.ServiceException:
        print('Service call failed: %s' % e)

def createTopoMapEdge(node1,node2,edge_id):
    rospy.wait_for_service('/topological_map_manager/add_edges_between_nodes')
    try:
        callCreateTopoEdgeService = rospy.ServiceProxy('/topological_map_manager/add_edges_between_nodes', AddEdge)
        return callCreateTopoEdgeService(node1,node2,'move_base',edge_id)
    except rospy.ServiceException:
        print('Service call failed: %s' % e)

def removeTopoMapEdge(node1,node2,edge_ids):
    """
    removes edges between node1 and node2 with edge_id in the string list edge_ids
    """
    rospy.wait_for_service('/topological_map_manager/remove_edge')
    try:
        callRemoveTopoEdgeService = rospy.ServiceProxy('/topological_map_manager/remove_edge', AddEdge)
        for edge_id in edge_ids:
            callRemoveTopoEdgeService(node1,node2,'move_base',edge_id)
    except rospy.ServiceException:
        print('Service call failed: %s' % e)

def getEdgesBetweenNodes(node_a,node_b,returnAsList):
    """
    if returnAsList is true returns a single list of edge_ids for both directions between nodes. 
    Otherwise returns getEdgesBetweenNodesResponse object with string arrays ids_a_to_b and ids_b_to_a
    https://github.com/strands-project/strands_navigation/blob/indigo-devel/strands_navigation_msgs/srv/GetEdgesBetweenNodes.srv
    """
    rospy.wait_for_service('/topological_map_manager/get_edges_between_nodes')
    try:
        callGetEdgesBetweenNodesService = rospy.ServiceProxy('/topological_map_manager/get_edges_between_nodes', GetEdgesBetweenNodes)
        if returnAsList:
            getEdgesBetweenNodesResponse = callGetEdgesBetweenNodesService(node_a,node_b)
            edge_id_list = getEdgesBetweenNodesResponse.ids_a_to_b + getEdgesBetweenNodesResponse.ids_b_to_a
            return edge_id_list
        else:
            return callGetEdgesBetweenNodesService(node_a,node_b)
    except rospy.ServiceException:
        print('Service call failed: %s' % e)

def addTagToNode(tag,nodes):
    """
    nodes should be a string list of node names.
    """
    rospy.wait_for_service('/topological_map_manager/add_tag_to_node')
    try:
        callAddTagToNodeService = rospy.ServiceProxy('/topological_map_manager/add_tag_to_node', AddTag)
        return callAddTagToNodeService(tag,nodes)
    except rospy.ServiceException:
        print('Service call failed: %s' % e)

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

def getPoseFromNode(node):
    """
    returns the pose of a node.
    """
    #look up node pose from topological_map variable. Assumes nodes have unique names.
    for topologicalNode in topological_map.nodes:
        if topologicalNode.name == node:
            return topologicalNode.pose 

def new_nodes_between_nodes(start_node,end_node,numnodes):
    """
    new nodes are evenly spaced between start_node and end_node, and connected with edges. A direct edge between
    start_node and end_node will be removed if one exists.
    numnodes is inclusive of start_node and end_node e.g. if numnodes=3 is passed only one new node will be created.
    """
    start = getPoseFromNode(start_node)
    end = getPoseFromNode(end_node)
    
    start_tuple = start.position.x,start.position.y,start.position.z

    end_tuple = end.position.x,end.position.y,end.position.z

    #new nodes are calculated linearly spaced between start_node and end_node using np.linspace()
    node_names = []
    for i,coord in enumerate(np.linspace(start_tuple,end_tuple,num=numnodes)):
        current_pose = Pose()
        current_pose.position.x = coord[0]
        current_pose.position.y = coord[1]
        current_pose.position.z = coord[2]
        current_pose.orientation.x = start.orientation.x
        current_pose.orientation.y = start.orientation.y
        current_pose.orientation.z = start.orientation.z
        current_pose.orientation.w = start.orientation.w
        
        #these nodes are start_node and end_node so they are skipped 
        if i == 0 or i == numnodes:
            continue
            
        #generate_node_names
        name = start_node + '_to_' + end_node + "_" + str(i-1)
        node_names.append(name)
        addToTopoMap(name,current_pose)
            
    #link new nodes
    createTopoMapEdge(start_node,node_names[0],start_node + "_to_" + node_names[0])
    for i,node_name in enumerate(node_names):
        if i+1 < len(node_names):
            createTopoMapEdge(node_name,node_names[i+1],node_name + "_to_" + node_names[i+1])
        else:
            createTopoMapEdge(node_name,end_node,node_name + "_to_" + end_node)

    #remove edges between start_node and end_node
    removeTopoMapEdge(start_node,end_node,getEdgesBetweenNodes(start_node,end_node,returnAsList=True))

current_pose = Pose()
def pose_callback(data):
    global current_pose
    current_pose = data

def add_current_pose(node_name):
    """
    adds the current pose of the robot to the topological map as node_name.
    """
    addToTopoMap(node_name,current_pose)

def main():
     #init_node
    rospy.init_node('edit_topo_map',anonymous=True)
    
    #subscribers
    pose_sub = rospy.Subscriber('/thorvald_001/robot_pose',Pose,pose_callback)
    topological_map_sub = rospy.Subscriber('/topological_map',TopologicalMap,topological_map_callback)

    #rospy.spin()


if __name__ == '__main__':
    main()

