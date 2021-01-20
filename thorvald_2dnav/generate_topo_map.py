#!/usr/bin/python

#python libraries
import sys
import numpy as np
import threading

#ROS libraries
import rospy

#ROS messages
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose,PoseArray

#ros services
from nav_msgs.srv import GetMap

#other libraries
from sklearn.cluster import SpectralClustering, KMeans
import skimage.draw 
from scipy.sparse.csgraph import minimum_spanning_tree
from scipy.sparse import csr_matrix
from scipy.spatial import distance

#utils
from edit_topo_map import addToTopoMap,createTopoMapEdge,addTagToNode

#--------------------
# Service calls
#--------------------

def getMapFromMapServer():
    rospy.wait_for_service('/static_map')
    try:
        callGetMapFromServer = rospy.ServiceProxy('/static_map', GetMap)
        return callGetMapFromServer()
    except rospy.ServiceException:
        print('Service call failed: %s' % e)

#--------------------
# functions for graph generation
#--------------------

def SpectralClusterMap(worldmap_array,numclusters):
    """ 
    Runs spectral clustering on the free space in worldmap_array. returns a list of numclusters 
    cluster centres.
    """
    
    #sample free space in map
    scale_down_num = 20
    free_tiles = []
    for row in range(worldmap_array.shape[0]):
        for col in range(worldmap_array.shape[1]):
            if worldmap_array[row][col] == 0:
                #sample once per metre
                if row % 10 == 0 and col % 10 == 0: 
                    free_tiles.append([row,col])

    #https://scikit-learn.org/stable/modules/generated/sklearn.cluster.SpectralClustering.html
    clustering = SpectralClustering(n_clusters=numclusters, assign_labels="discretize").fit(free_tiles)

    assert len(free_tiles) == len(clustering.labels_)

    # free_tiles are added to labels_dict  
    labels_dict ={}

    for label in range(numclusters):
        labels_dict[str(label)] = []
    for i,label in enumerate(clustering.labels_):
        labels_dict[str(label)].append(free_tiles[i])

    #calculate cluster centres
    cluster_centres = []
    for label,labeled_free_tiles in labels_dict.items():
        cluster_centres.append(np.array(labeled_free_tiles).mean(axis=0).astype('int'))

    return cluster_centres

def cluster_crops(crop_coords,n_clusters=48):
    """applies kmeans clustering to crops"""
    kmeans = KMeans(n_clusters=n_clusters, random_state=0).fit(crop_coords)

    return kmeans.cluster_centers_.astype(int)

def collision_check(coord1,coord2,worldmap_array):
    """ checks for a collision in the straight line edge between coord1 and coord2 """
    rr,cc = skimage.draw.line(coord1[0],coord1[1],coord2[0],coord2[1])
    if all(worldmap_array[rr,cc] != 100):
        return False
    else:
        return True

def generate_graph(cluster_centres,worldmap_array,aspect_ratio=[1,1]):
    """
    takes args cluster_centres and worldmap_array. Returns nodes as a dict and edges are list of paired node 
    keys. Nodes are in occupancy grid coordinates.
    Minimum spanning tree is used to optimise the edges of the graph
    https://docs.scipy.org/doc/scipy/reference/generated/scipy.sparse.csgraph.minimum_spanning_tree.html
    """

    # weights_array is an i,j matrix where the value of an element is the euclidean distance between
    # cluster_centres[i]*aspect_ratio and cluster_centres[j]*aspect_ratio. 
    # This is in the format used by scipy for a graph.  
    weights_array = distance.cdist(cluster_centres*aspect_ratio, cluster_centres*aspect_ratio, 'euclidean')

    #check for collisions in weights_array:
    for i,row in enumerate(weights_array):
        for j,col in enumerate(row):
            if (collision_check(cluster_centres[i],cluster_centres[j],worldmap_array)):
                weights_array[i][j] = 0

    # with collisions removed the minimum_spanning_tree algorithm can be used to optimise the graph.
    min_span_tree_array = minimum_spanning_tree(csr_matrix(weights_array)).toarray().astype(int)

    #assign nodes and edges
    nodes = {}
    node_names = []
    name="node_"
    for i,node in enumerate(cluster_centres):
        nodes[name + str(i)] = node
        node_names.append(name + str(i))

    edges = []
    for i,row in enumerate(min_span_tree_array):
        row_node = node_names[i]
        for j,col in enumerate(row):
            col_node = node_names[j]
            if col > 0:
                if not (collision_check(cluster_centres[i],cluster_centres[j],worldmap_array)):
                    edges.append([row_node,col_node])
    
    return nodes,edges

#--------------------
# other functions
#--------------------

def fake_crops(num_per_row=20):
    """ returns a fake sample of coordinates in the crop row regions for testing """

    row_0 = np.random.uniform((-5,-2.7),(5,-3.3),size=(num_per_row,2))
    row_1 = np.random.uniform((-5,-1.7),(5,-2.3),size=(num_per_row,2))
    row_2 = np.random.uniform((-5,-0.3),(5,0.3),size=(num_per_row,2))
    row_3 = np.random.uniform((-5,0.7),(5,1.3),size=(num_per_row,2))
    row_4 = np.random.uniform((-5,2.7),(5,3.3),size=(num_per_row,2))
    row_5 = np.random.uniform((-5,3.7),(5,4.3),size=(num_per_row,2))
    
    return np.vstack((row_0,row_1,row_2,row_3,row_4,row_5))

def convert_coord_to_map(real_coords,map_info):
    """ converts np.array of real_coords to occupancy grid map coords"""
    offset = [-map_info.origin.position.x,-map_info.origin.position.y]
    scale_factor =  1/map_info.resolution

    map_coord = scale_factor * (real_coords + offset)
    return map_coord.astype('int')

def nodeMapCoordsToPose(nodes,map_info):
    """ converts a dict of nodes with vals in the 2d occupancy grid to a dict of nodes with poses. """
    output_dict = {}
    for name,coords in nodes.items():
        node_pose = Pose()
        node_pose.position.x = coords[0]*map_info.resolution + map_info.origin.position.x
        node_pose.position.y = coords[1]*map_info.resolution + map_info.origin.position.y
        node_pose.orientation.x = map_info.origin.orientation.x
        node_pose.orientation.y = map_info.origin.orientation.y
        node_pose.orientation.z = map_info.origin.orientation.z
        node_pose.orientation.w = map_info.origin.orientation.w
        output_dict[name] = node_pose
    return output_dict

if __name__ == '__main__':
    # init_node
    rospy.init_node('generate_topo_map',anonymous=True)
    print('node initialised')

    # get worldmap from map_server
    print('requesting map')
    worldmap = getMapFromMapServer()
    print('received map')
    worldmap_array = np.array(worldmap.map.data).reshape(worldmap.map.info.height,worldmap.map.info.width)

    # generate nodes across the whole map avoiding obstacles (this method does not specifically target crops)
    # cluster_centres = SpectralClusterMap(worldmap_array,10)
    # pixel_nodes,edges = generate_graph(cluster_centres,worldmap_array)
    # nodes = nodeMapCoordsToPose(pixel_nodes,worldmap.map.info)

    print('generating nodes')
    # convert coordinates of detected crops to the occupancy grid coordinate system (refered to as map here)
    crop_coords = convert_coord_to_map(fake_crops(100),worldmap.map.info)
    
    # cluster crop detections using kmeans
    clustered_crops = cluster_crops(crop_coords,60)
    
    # generate nodes and edges for a graph using minimum spanning tree algorithm 
    crop_map_nodes,crop_edges = generate_graph(clustered_crops,worldmap_array=worldmap_array,aspect_ratio=[1,5])
    
    # convert the nodes back to the proper coordinate system and turn 2d coordinates into pose.
    crop_nodes = nodeMapCoordsToPose(crop_map_nodes,worldmap.map.info)
    print('nodes generated')
    
    print('adding nodes to topological map')
    for node,pose in crop_nodes.items():
        addToTopoMap(node,pose)

    print('adding edges to topological map')
    for edge in crop_edges:
        createTopoMapEdge(edge[0],edge[1], edge[0] + '_to_' + edge[1], bidirectional=True)

    print('adding tags to nodes')
    addTagToNode('crops',crop_nodes.keys())
    #arbitrarily set a node to start node
    addTagToNode('start',crop_nodes.keys()[0])

    print('done')
