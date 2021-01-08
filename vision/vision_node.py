#!/usr/bin/python

#python libraries
import sys
import numpy as np
import cv2

#ROS libraries
import rospy, image_geometry, tf

#ROS messages
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Pose, PoseStamped, PoseArray

def mask_crop(hsv_image):
    crop_high = (160,200,200)
    crop_low = (40,100,1)
    crop_mask = cv2.inRange(hsv_image, crop_low, crop_high)
    return crop_mask

def mask_weeds(hsv_image):
    weeds_high = (160,100,200)
    weeds_low = (55,1,1)
    weed_mask = cv2.inRange(hsv_image, weeds_low, weeds_high)
    return weed_mask

def remove_small_connected_components(labels_im,stats,centroids,min_num_pixels=60):
    """
    removes labels which have an area less than min_num_pixels. Returns labels_im, stats 
    and centroids for the image with labels corrected after the removal of small clusters so that labels
    in the image still correspond to the index of stats or centroids.
    """
    new_label_index = 0
    removed_indices = []
    for label in range(len(stats)):
        #remove labels if they have an area less than min_num_pixels 
        if stats[label][4] < min_num_pixels: 
            labels_im[labels_im==label] = 0 #set to background.
            removed_indices.append(label)    
        else:
        #for labels with area greater than min_num_pixels the label is updated so that they will correspond to 
        #the new index of stats and centroids  
            labels_im[labels_im==label] = new_label_index
            new_label_index += 1
            
    #stats and centroids arrays are updated to remove references for filtered labels.
    new_stats = np.delete(stats,removed_indices,axis=0)
    new_centroids = np.delete(centroids,removed_indices,axis=0)         
    return labels_im, new_stats, new_centroids

def kmeans_centroids(centroids,num_clusters,labels_im=None):
    """
    Applying kmeans clustering to the centroids of connected components, returns the centroids of the k clusters.
    If labels_im is given, it will be updated so that the labels and indices reflect the new clusters
    TODO tune parameters, num clusters etc for kmeans.
    """    
    #the centroid at index 0 is for the background and is discarded
    compactness,cluster_labels,cluster_centroids = cv2.kmeans(centroids[1:].astype('float32'),K=num_clusters,bestLabels=None,
                                                            criteria=(cv2.TERM_CRITERIA_EPS, 10, 0.1),
                                                            attempts=100,flags=cv2.KMEANS_RANDOM_CENTERS)
    if labels_im is not None:
        for index,cluster_label in enumerate(cluster_labels):
            labels_im[labels_im==index+1] = cluster_label + 1 #index 0 is used for background 
        return cluster_centroids,labels_im
    else:
        return cluster_centroids

def connected_components_from_image(image,filter_small_connected_components=True,min_num_pixels=60):
    """
    returns the connected_components for a given hsv image. If filter_small_connected_components is true 
    connected components with less than min_num_pixels will be removed. 
    """
    grayscale_image = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
    threshold_image = cv2.threshold(grayscale_image,1,255, cv2.THRESH_BINARY)[1] 
    num_labels, labels_im, stats, centroids = cv2.connectedComponentsWithStats(threshold_image)
    
    if filter_small_connected_components:
        labels_im, stats, centroids = remove_small_connected_components(labels_im,stats,centroids,min_num_pixels)
        num_labels = len(stats)
    
    return num_labels,labels_im,stats,centroids

class image_converter:
    camera_model = None
    cv_image = None
    
    def __init__(self):
        #publishers
        self.image_pub = rospy.Publisher("{}/opencv_image".format(robot_name),Image,queue_size=0)
        self.weed_pose_array_pub = rospy.Publisher("{}/weed_pose_array".format(robot_name),PoseArray,queue_size=0)

        #subscribers
        self.image_sub = rospy.Subscriber("/{}/kinect2_camera/hd/image_color_rect".format(robot_name),Image,self.image_callback)
        self.camera_info_sub = rospy.Subscriber('/{}/kinect2_camera/hd/camera_info'.format(robot_name),CameraInfo, self.camera_info_callback)
        
        self.tf_listener = tf.TransformListener()
        self.bridge = CvBridge()

    def camera_info_callback(self, data):
        self.camera_model = image_geometry.PinholeCameraModel()
        self.camera_model.fromCameraInfo(data)
        self.camera_info_sub.unregister() #Only subscribe once

    def image_callback(self,data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data,"bgr8")
            cv_image = self.cv_image
            print("received image")
        except CvBridgeError as e:
            print(e)

        #TODO split image_callback from image_processing when changing to service.

        if self.camera_model is None:
            return #TODO add exception or similar to aid debugging this
        if self.cv_image is None:
            return #TODO add exception or similar to aid debugging this
        
        print("processing image")
        weed_pixel_coords, crop_pixel_coords = self.process_image_colour_kmeans(cv_image)
        print("image processed")

        #transform pixel coordinates to camera frame
        distance_to_floor = 0.7 #TODO get correct value from tf
        weed_pose_array = PoseArray()
        weed_pose_array.header.frame_id = '/map'
        for i,weed_pixel_coord in enumerate(weed_pixel_coords):
            weed_unit_vector = np.array(self.camera_model.projectPixelTo3dRay(weed_pixel_coord))
            
            #define a PoseStamped message of weed_pose in the camera frame
            weed_pose = PoseStamped()
            weed_pose.header.frame_id = self.camera_model.tfFrame()
            weed_pose.pose.position.x,weed_pose.pose.position.y,weed_pose.pose.position.z = weed_unit_vector * distance_to_floor
            
            #transform weed_pose to the map frame
            try: 
                self.tf_listener.waitForTransform('/map', weed_pose.header.frame_id,rospy.Time(),rospy.Duration(0.5))
                target_pose_WRT_map = self.tf_listener.transformPose('/map', weed_pose)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue 
            
            #append transformed pose to weed_pose_array
            weed_pose_array.poses.append(target_pose_WRT_map.pose)
        
        #publish pose_array
        self.weed_pose_array_pub.publish(weed_pose_array)

        #preparing and publishing image with added weed targets. 
        #image should be rgb for display in rviz.
        output_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        #plot a circle at each detected weed.
        if weed_pixel_coords is not None:
            for coord in weed_pixel_coords:
                output_image = cv2.circle(output_image, tuple(coord), radius=10, color=(255, 0, 255), thickness=3)
        #publish output_image 
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(output_image))
        except CvBridgeError as e:
            print(e)

    def process_image_colour_kmeans(self, image, num_weed_targets=10,num_crop_targets=10):
        """
        processes a BGR image of crops and weeds, using colour to segment and classify crops, and kmeans 
        clustering to identify targets.
        """
        
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        #crop_result and weed_result are masked to show only crops and only weeds respectively. 
        #the colour thresholds work well for crops in rows 0 and 1, reasonably well for rows 2 and 3,
        #and poorly for crops in rows 4 and 5.
        crop_result = cv2.bitwise_and(image,image,mask=mask_crop(hsv_image))
        weed_result = cv2.bitwise_and(image,image,mask=mask_weeds(hsv_image))
        
        #get the connected_components in the masked_images and filter small components
        crop_num_labels, crop_labels_im, crop_stats, crop_centroids = connected_components_from_image(crop_result,filter_small_connected_components=True)
        weed_num_labels, weed_labels_im, weed_stats, weed_centroids = connected_components_from_image(weed_result,filter_small_connected_components=True)
        
        #run kmeans clustering on the centroids of crop and weed components.If there are not enough centroids 
        #passed to kmeans the exception will set coords to None
        try:
            crop_pixel_coords = kmeans_centroids(crop_centroids,num_clusters=min(num_crop_targets,len(crop_centroids)-1))
        except:
            crop_pixel_coords = None
        try:
            weed_pixel_coords = kmeans_centroids(weed_centroids,num_clusters=min(num_weed_targets,len(weed_centroids)-1))
        except:
            weed_pixel_coords = None
        
        return weed_pixel_coords, crop_pixel_coords


if __name__ == '__main__':
    if(len(sys.argv)>1):
        robot_name = sys.argv[1]
        print(robot_name)
    else:
        robot_name = "thorvald_001"

    rospy.init_node('vision_node', anonymous=True)
    ic = image_converter()
    #s = rospy.Service('crop_and_weed_targets', AddTwoInts, handle_add_two_ints)
    rospy.spin()
