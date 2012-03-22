#!/usr/bin/env python
from __future__ import print_function
import roslib
roslib.load_manifest('mct_image_stitching')
import rospy
import cv
import cv2
import sys
import threading
import functools
import math
from cv_bridge.cv_bridge import CvBridge 

import mct_introspection
from mct_utilities import file_tools
from mct_transform_2d import transform_2d

# Messages
from sensor_msgs.msg import CameraInfo 
from sensor_msgs.msg import Image 

class ImageStitcher(object):

    def __init__(self, region):
        rospy.init_node('image_stitcher')
        self.ready = False

        self.lock = threading.Lock()
        self.bridge = CvBridge()

        # Get camera list from region
        regions_dict = file_tools.read_tracking_2d_regions()
        self.region = region
        self.camera_list = regions_dict[region]
        self.create_camera_to_image_dict()
        self.create_camera_to_info_dict()
        self.tf2d = transform_2d.Transform2d()

        # Get stitched image size from bounding box
        bbox = self.tf2d.get_stitching_plane_bounding_box(region)
        self.image_width = int(math.ceil(bbox['max_x']))
        self.image_height = int(math.ceil(bbox['max_y']))

        self.seq_to_images = {}

        # Pools for incomming data.  
        self.stamp_to_seq_pool= {}
        self.image_waiting_pool = {} 
        for camera in self.camera_list:
            self.stamp_to_seq_pool[camera] = {}
            self.image_waiting_pool[camera] = {}

        # Subscript to camera info topics
        self.info_sub ={} 
        for camera, topic  in  self.camera_to_info.iteritems():
            info_handler = functools.partial(self.info_handler, camera)
            self.info_sub[camera] = rospy.Subscriber(topic, CameraInfo, info_handler)

        # Subscript to camera info topics
        self.image_sub ={} 
        for camera, topic  in  self.camera_to_image.iteritems():
            image_handler = functools.partial(self.image_handler, camera)
            self.image_sub[camera] = rospy.Subscriber(topic, Image, image_handler)

        # Stitched image publisher 
        self.image_pub = rospy.Publisher('image_stitched', Image)

        self.ready = True


    def create_camera_to_image_dict(self):
        """
        Create camera to image topic dictionary
        """
        self.camera_to_image = {}
        rect_topics = mct_introspection.find_camera_image_topics(transport='image_rect')
        for topic in rect_topics:
            topic_split = topic.split('/')
            for camera in self.camera_list:
                if camera in topic_split:
                    self.camera_to_image[camera] = topic

    def create_camera_to_info_dict(self):
        """
        Create camera to image topic dictionary
        """
        self.camera_to_info = {}
        info_topics = mct_introspection.find_camera_info_topics()
        for topic in info_topics:
            topic_split = topic.split('/')
            for camera in self.camera_list:
                if camera in topic_split:
                    self.camera_to_info[camera] = topic

    def info_handler(self,camera,data):
        """
        Handler for incoming camera info messages. In this callback we place image 
        sequence number into stamp_to_seq_pool dictionay by camera name and timestamp 
        """
        if self.ready:
            with self.lock:
                stamp = data.header.stamp.secs, data.header.stamp.nsecs
                self.stamp_to_seq_pool[camera][stamp] = data.header.seq

    def image_handler(self, camera, data):
        """
        Handler for incoming camera images. In this callback we place the image data
        into the image_waiting_pool by camera name and timestamp. Note, we don't want
        to use the seq information in data.header.seq as this seems to initialized in
        some random way - probably has to do with python not fully supporting ROS's 
        image_transport. The seq's from the camera_info topics are correct.
        """
        if self.ready:
            with self.lock:
                # Place image into waiting pool by camera and time stamp
                stamp = data.header.stamp.secs, data.header.stamp.nsecs
                self.image_waiting_pool[camera][stamp] = data

    def process_waiting_images(self):
        """ 
        Processes waiting images. Associates images in the waiting pool with their acquisition
        sequence number and places them in the seq_to_images buffer. Checks to see if the 
        sequences to images buffer contains all required frames and if so produces and publishes a
        stitched image.
        """
        with self.lock:
            # Associate images in the waiting pool with their seq numbers 
            for camera in self.camera_list:
                for stamp, image_data in self.image_waiting_pool[camera].items():
                    try:
                        seq = self.stamp_to_seq_pool[camera][stamp]
                        del self.stamp_to_seq_pool[camera][stamp]
                        del self.image_waiting_pool[camera][stamp]
                        image_data.header.seq = seq 
                        try:
                            self.seq_to_images[seq][camera] = image_data
                        except KeyError:
                            self.seq_to_images[seq] = {camera: image_data}
                    except KeyError:
                        pass
        
        # Check to see if we have all images for a given sequence number
        for seq, image_dict in self.seq_to_images.items():

            # If we have all images for the sequece - stitch into merged view
            if len(image_dict) == len(self.camera_list):

                for i, camera in enumerate(self.camera_list):

                    ros_image = image_dict[camera] 
                    cv_image = self.bridge.imgmsg_to_cv(ros_image,desired_encoding="passthrough")
                    ipl_image = cv.GetImage(cv_image)

                    if i == 0:

                        temp_image = cv.CreateImage(
                                (self.image_width, self.image_height), 
                                ipl_image.depth,
                                ipl_image.channels
                                )

                        stitched_image = cv.CreateImage(
                                (self.image_width, self.image_height), 
                                ipl_image.depth,
                                ipl_image.channels
                                )

                        cv.Zero(stitched_image)

                    # Get transform from camera to stiching plane
                    tf_matrix = self.tf2d.get_camera_to_stitching_plane_tf(camera)
                    tf_matrix_cv = cv.fromarray(tf_matrix)

                    # Wap into temporary image 
                    cv.WarpPerspective(ipl_image, temp_image, tf_matrix_cv)
                    cv.Or(temp_image,stitched_image,stitched_image)


                    # Blend to create stitched image
                
                print('syncd seq: {0}'.format(seq))

                del self.seq_to_images[seq]

                # Convert stitched image to ros image and publish
                stitched_ros_image = self.bridge.cv_to_imgmsg(stitched_image,encoding="passthrough")
                self.image_pub.publish(stitched_ros_image)


    def run(self):
        while not rospy.is_shutdown(): 
            self.process_waiting_images() 
            rospy.sleep(0.01)
                



# -----------------------------------------------------------------------------
if __name__ == '__main__':

    region = sys.argv[1]
    node = ImageStitcher(region)
    node.run()
