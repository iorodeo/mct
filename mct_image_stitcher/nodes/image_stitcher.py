#!/usr/bin/env python
from __future__ import print_function
import roslib
roslib.load_manifest('mct_image_stitcher')
import rospy
import cv
import cv2
import sys
import threading
import functools
import math
import numpy
from cv_bridge.cv_bridge import CvBridge 

import mct_introspection
from mct_utilities import file_tools
from mct_transform_2d import transform_2d

# Messages
from mct_msg_and_srv.msg import StampInfo
from mct_msg_and_srv.msg import ProcessingInfo
from sensor_msgs.msg import CameraInfo 
from sensor_msgs.msg import Image 
from std_msgs.msg import UInt32
from std_msgs.msg import Float32

MAX_UINT32 = 2**32 - 1

class ImageStitcher(object):
    """
    Subscribes to all rectified image topics in a 2d tracking region and uses the calibration
    data to stitch the images together. Publishes image_stitched and image_stitched/seq.
    """

    def __init__(self, region, max_seq_age=1000, max_stamp_age=5.0):

        #self.warp_flags = cv.CV_INTER_LINEAR | cv.CV_WARP_INVERSE_MAP
        self.warp_flags = cv.CV_INTER_LINEAR 
        #self.warp_flags = cv.CV_INTER_AREA
        #self.warp_flags = cv.CV_INTER_NN
        self.affine_approx = False 

        rospy.init_node('image_stitcher')
        self.max_seq_age = max_seq_age
        self.max_stamp_age = max_stamp_age
        self.ready = False
        self.is_first_write = True
        self.stitched_image = None
        self.seq_to_images = {}  # Seq to image list buffer

        # Information on the latest sequence received and stitched
        self.seq_newest = None
        self.stamp_newest = None

        self.lock = threading.Lock()
        self.bridge = CvBridge()

        # Get image and camera information 
        regions_dict = file_tools.read_tracking_2d_regions()
        self.region = region
        self.camera_list = regions_dict[region]
        self.create_camera_to_image_dict()
        self.create_camera_to_info_dict()

        # Get transforms from cameras to stitched image and stitched image size
        self.tf2d = transform_2d.Transform2d()
        self.create_tf_data()
        self.get_stitched_image_size()
            
        # Create pool dictionaries for incomming data.  
        self.stamp_to_seq_pool= {}
        self.image_waiting_pool = {} 
        for camera in self.camera_list:
            self.stamp_to_seq_pool[camera] = {}
            self.image_waiting_pool[camera] = {}

        # Subscribe to camera info topics
        self.info_sub ={} 
        for camera, topic  in  self.camera_to_info.iteritems():
            info_handler = functools.partial(self.info_handler, camera)
            self.info_sub[camera] = rospy.Subscriber(topic, CameraInfo, info_handler)

        # Subscribe to rectified image topics
        self.image_sub ={} 
        for camera, topic  in  self.camera_to_image.iteritems():
            image_handler = functools.partial(self.image_handler, camera)
            self.image_sub[camera] = rospy.Subscriber(topic, Image, image_handler)

        # Stitched image publisher and seq publisher
        self.image_pub = rospy.Publisher('image_stitched', Image)
        self.seq_pub = rospy.Publisher('image_stitched/seq', UInt32)
        self.stamp_pub = rospy.Publisher('image_stitched/stamp_info', StampInfo)
        self.processing_dt_pub = rospy.Publisher('image_stitched/processing_dt', ProcessingInfo)
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

    def create_tf_data(self):
        """
        Pre-computes transform matrices and roi for creating the stitched images.
        """
        self.tf_data = {}
        for camera in self.camera_list:

            # Get modified transform which maps to ROI in stitched image 
            bbox = self.tf2d.get_stitching_plane_bounding_box(region, camera_list=[camera])
            tf_matrix = self.tf2d.get_camera_to_stitching_plane_tf(camera)
            roi_x = int(math.floor(bbox['min_x']))
            roi_y = int(math.floor(bbox['min_y']))
            roi_w = int(math.floor(bbox['max_x']) - math.floor(bbox['min_x']+5))
            roi_h = int(math.floor(bbox['max_y']) - math.floor(bbox['min_y']+5))
            shift_matrix = numpy.array([
                [1.0, 0.0, -bbox['min_x']],
                [0.0, 1.0, -bbox['min_y']], 
                [0.0, 0.0, 1.0],
                ])
            tf_matrix = numpy.dot(shift_matrix, tf_matrix)

            if self.affine_approx:
                # DEBUG ###########################################################################################
                # This is wrong need to get the real affine approximation
                # #################################################################################################
                self.tf_data[camera] = {'matrix': cv.fromarray(tf_matrix[:2,:]), 'roi': (roi_x, roi_y, roi_w, roi_h)}
            else:
                self.tf_data[camera] = {'matrix': cv.fromarray(tf_matrix), 'roi': (roi_x, roi_y, roi_w, roi_h)}

    def get_stitched_image_size(self):
        """
        Determines the size of the stitched image.
        """
        # Get stitched image size from bounding box
        bbox = self.tf2d.get_stitching_plane_bounding_box(region)
        self.image_width = int(math.ceil(bbox['max_x']))
        self.image_height = int(math.ceil(bbox['max_y']))
        self.image_size = (self.image_width, self.image_height)

    def info_handler(self,camera,data):
        """
        Handler for incoming camera info messages. In this callback we place image 
        sequence number into stamp_to_seq_pool dictionay by camera name and timestamp 
        """
        if self.ready:
            with self.lock:
                stamp = data.header.stamp.secs, data.header.stamp.nsecs
                self.stamp_to_seq_pool[camera][stamp] = data.header.seq

                # Update newest sequence
                if self.seq_newest is None:
                    self.seq_newest = data.header.seq
                else:
                    self.seq_newest = max([self.seq_newest, data.header.seq])

                # Update newest time stamp
                if self.stamp_newest is None:
                    self.stamp_newest = stamp
                else:
                    self.stamp_newest = max([self.stamp_newest, stamp])

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
        sequence number and places them in the seq_to_images buffer.         
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
                        deleted = True
                        try:
                            self.seq_to_images[seq][camera] = image_data
                        except KeyError:
                            self.seq_to_images[seq] = {camera: image_data}
                    except KeyError:
                        deleted = False

                    # Clear out stale data from image waiting pool
                    if not deleted:
                        stamp_age = self.get_stamp_age(stamp)
                        if stamp_age > self.max_stamp_age:
                            del self.image_waiting_pool[camera][stamp]

                # Clear out any stale data from stamp-to-seq pool
                for stamp, seq in self.stamp_to_seq_pool[camera].items():
                    seq_age = self.get_seq_age(seq)
                    if seq_age > self.max_seq_age: 
                        del self.stamp_to_seq_pool[camera][stamp]


    def get_stamp_age(self, stamp):
        """
        Computes the age of a time stamp.
        """
        stamp_secs = stamp_tuple_to_secs(stamp)
        stamp_newest_secs = stamp_tuple_to_secs(self.stamp_newest)
        return stamp_newest_secs - stamp_secs


    def get_seq_age(self, seq):
        """
        Compute sequece age - handling 32bit uint rollover of seq counter.  Note, at 
        30Hz this should roll over for something like 4yrs, but you never know.
        """
        age0 = abs(self.seq_newest - seq)
        age1 = MAX_UINT32 - seq + self.seq_newest + 1
        return min([age0, age1])


    def publish_stitched_image(self):
        """
        Checks to see if the sequences to images buffer contains all required frames and if so 
        produces and publishes a stitched image.
        """

        # Check to see if we have all images for a given sequence number
        for seq, image_dict in sorted(self.seq_to_images.items()): 

            # If we have all images for the sequece - stitch into merged view
            if len(image_dict) == len(self.camera_list):

                # Get start time to measure processing dt
                t0 = rospy.Time.now() 
                for i, camera in enumerate(self.camera_list):

                    # Convert ros image to opencv image
                    ros_image = image_dict[camera] 
                    cv_image = self.bridge.imgmsg_to_cv(
                            ros_image,
                            desired_encoding="passthrough"
                            )
                    ipl_image = cv.GetImage(cv_image)

                    # Create stitced image if it doesn't exist yet
                    if self.stitched_image is None:
                        self.stitched_image = cv.CreateImage(
                                self.image_size,
                                ipl_image.depth,
                                ipl_image.channels
                                )

                    # Set image warping flags - if first fill rest of image with zeros
                    if self.is_first_write:
                        warp_flags = self.warp_flags | cv.CV_WARP_FILL_OUTLIERS
                        self.is_first_write = False
                    else:
                        warp_flags = self.warp_flags

                    # Warp into stitched image 
                    cv.SetImageROI(self.stitched_image, self.tf_data[camera]['roi'])

                    # Warp stitched image
                    if self.affine_approx:
                        cv.WarpAffine(ipl_image,self.stitched_image,self.tf_data[camera]['matrix'],flags=warp_flags)
                    else:
                        cv.WarpPerspective(
                                ipl_image, 
                                self.stitched_image, 
                                self.tf_data[camera]['matrix'],
                                flags=warp_flags,
                                )
                    cv.ResetImageROI(self.stitched_image)

                    # Get max and min time stamps for stamp_info
                    stamp = ros_image.header.stamp
                    if i == 0:
                        stamp_max = stamp 
                        stamp_min = stamp
                    else:
                        stamp_max = stamp if stamp.to_sec() > stamp_max.to_sec() else stamp_max 
                        stamp_mim = stamp if stamp.to_sec() < stamp_min.to_sec() else stamp_min 

                # Convert stitched image to ros image and publish
                stitched_ros_image = self.bridge.cv_to_imgmsg(
                        self.stitched_image,
                        encoding="passthrough"
                        )
                stitched_ros_image.header.seq = seq
                self.image_pub.publish(stitched_ros_image)

                # Publish seq information and time stamp spread
                self.seq_pub.publish(seq)
                stamp_diff = stamp_max.to_sec() - stamp_min.to_sec()
                self.stamp_pub.publish(stamp_max, stamp_min, stamp_diff)
                
                t1 = rospy.Time.now()
                dt = t1.to_sec() - t0.to_sec()
                self.processing_dt_pub.publish(dt,1.0/dt)

                # Remove data from buffer
                del self.seq_to_images[seq]
                print('syncd seq: {0}'.format(seq))

            # Throw away any stale data in seq to images buffer
            seq_age = self.get_seq_age(seq)
            if seq_age > self.max_seq_age:
                del self.seq_to_images[seq]



    def run(self):

        while not rospy.is_shutdown(): 
            if self.seq_newest is None:
                continue

            self.process_waiting_images() 
            self.publish_stitched_image()


# -----------------------------------------------------------------------------
def stamp_tuple_to_secs(stamp):
    return stamp[0] + stamp[1]*1.0e-9


# -----------------------------------------------------------------------------
if __name__ == '__main__':

    region = sys.argv[1]
    node = ImageStitcher(region)
    node.run()
