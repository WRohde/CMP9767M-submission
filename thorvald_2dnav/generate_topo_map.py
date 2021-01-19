#!/usr/bin/python


#python libraries
import sys
import numpy as np
import threading

#ROS libraries
import rospy

#ROS messages
from nav_msgs.msg import OccupancyGrid

#ros services
from nav_msgs.srv import GetMap

#scipy libraries
from sklearn.cluster import SpectralClustering
import skimage.draw 
from scipy.sparse.csgraph import minimum_spanning_tree
from scipy.sparse import csr_matrix
from scipy.spatial import distance

def getMapFromMapServer():
    rospy.wait_for_service('/static_map')
    try:
        callGetMapFromServer = rospy.ServiceProxy('/static_map', GetMap)
        return callGetMapFromServer()
    except rospy.ServiceException:
        print('Service call failed: %s' % e)


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

def collision_check(coord1,coord2,worldmap_array,draw=False):
    """ checks for a collision in the straight line edge between coord1 and coord2 """
    rr,cc = skimage.draw.line(coord1[0],coord1[1],coord2[0],coord2[1])
    if all(worldmap_array[rr,cc] != 100):
        if draw:
            worldmap_array[rr,cc] = 30
        return False
    else:
        return True

def generate_graph(cluster_centres,worldmap_array):
    #Minimum spanning tree is used to optimise the edges of the graph
    #https://docs.scipy.org/doc/scipy/reference/generated/scipy.sparse.csgraph.minimum_spanning_tree.html

    # euclidean_array is an i,j matrix where the value of an element is the euclidean distance between
    # cluster_centres[i] and cluster_centres[j]. This is in the format used by scipy for a graph 
    euclidean_array = distance.cdist(cluster_centres, cluster_centres, 'euclidean')

    #check for collisions in euclidean_array:
    for i,row in enumerate(euclidean_array):
        for j,col in enumerate(row):
            if (collision_check(cluster_centres[i],cluster_centres[j],worldmap_array)):
                euclidean_array[i][j] = 0

    # with collisions removed the minimum_spanning_tree algorithm can be used to optimise the graph.
    min_span_tree_array = minimum_spanning_tree(csr_matrix(euclidean_array)).toarray().astype(int)

    #assign nodes and edges
    nodes = {}
    name="world_node_"
    for i,node in enumerate(cluster_centres):
        nodes[name + str(i)] = node

    edges = []
    for i,row in enumerate(min_span_tree_array):
        row_node = nodes.keys()[i]
        for j,col in enumerate(row):
            col_node = nodes.keys()[j]
            if col > 0:
                if not (collision_check(cluster_centres[i],cluster_centres[j],worldmap_array)):
                    edges.append([row_node,col_node])
    
    return nodes,edges

#get worldmap as np array
worldmap = getMapFromMapServer()
worldmap_array = np.array(worldmap.map.data).reshape(worldmap.map.info.height,worldmap.map.info.width)

cluster_centres = SpectralClusterMap(worldmap_array,10)
nodes,edges = generate_graph(cluster_centres,worldmap_array)