#!/usr/bin/env python
from __future__ import print_function
import roslib
roslib.load_manifest('mct_tracking_2d')
import rospy
import sys
import threading
import math

import numpy
import scipy.optimize

import cv
from cv_bridge.cv_bridge import CvBridge 
from mct_blob_finder import BlobFinder
from mct_utilities import file_tools

# Messages
from std_msgs.msg import Float32
from std_msgs.msg import Time
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo 
from mct_msg_and_srv.msg import ThreePointTrackerRaw
from mct_msg_and_srv.msg import ImagePt

# Services
from mct_msg_and_srv.srv import BlobFinderSetParam
from mct_msg_and_srv.srv import BlobFinderSetParamResponse
from mct_msg_and_srv.srv import BlobFinderGetParam
from mct_msg_and_srv.srv import BlobFinderGetParamResponse

class ThreePointTracker(object):

    def __init__(self,topic=None):
        self.topic = topic
        self.lock = threading.Lock() 
        self.bridge = CvBridge()
        self.camera = get_camera_from_topic(self.topic)
        self.camera_info_topic = get_camera_info_from_image_topic(self.topic)

        # Get parameters from the parameter server
        params_ns = 'three_point_tracker_params'
        self.blobFinder = BlobFinder()
        self.tracking_pts_roi_size = rospy.get_param(
                '{0}/roi_size'.format(params_ns), 
                (200,200),
                )
        self.blobFinder.threshold = rospy.get_param( 
                '{0}/threshold'.format(params_ns), 
                100,
                )
        self.blobFinder.filter_by_area = rospy.get_param(
                '{0}/filter_by_area'.format(params_ns), 
                False,
                )
        self.blobFinder.min_area = rospy.get_param(
                '{0}/min_area'.format(params_ns),
                0,
                )
        self.blobFinder.max_area = rospy.get_param( 
                '{0}/max_area'.format(params_ns), 
                200,
                )
        self.tracking_pts_spacing = rospy.get_param(
                '{0}/pts_spacing'.format(params_ns), 
                (0.0, 0.04774, 0.07019),
                )
        self.tracking_pts_colors =[(0,0,255), (0,255,0), (0,255,255)]

        # Determine target point spacing 
        d0 = self.tracking_pts_spacing[1] - self.tracking_pts_spacing[0]
        d1 = self.tracking_pts_spacing[2] - self.tracking_pts_spacing[1]
        if d0 > d1:
            self.large_step_first = True
        else:
            self.large_step_first = False

        # Data dictionareis for synchronizing tracking data with image seq number
        self.stamp_to_data = {}
        self.stamp_to_seq = {}
        self.seq_to_stamp_and_data= {}

        self.ready = False
        self.tracking_pts_roi = None
        rospy.init_node('blob_finder')

        # Subscribe to image and camera info topics
        self.image_sub = rospy.Subscriber(
                self.topic, 
                Image, 
                self.image_callback
                )
        self.info_sub = rospy.Subscriber(
                self.camera_info_topic, 
                CameraInfo, 
                self.camera_info_callback
                )

        # Create tracking point and tracking points image publications
        self.tracking_pts_pub = rospy.Publisher('tracking_pts', ThreePointTrackerRaw)
        self.image_tracking_pts_pub = rospy.Publisher('image_tracking_pts', Image)

        # Set up services - no really used at the moment 
        node_name = rospy.get_name()
        self.set_param_srv = rospy.Service( 
                '{0}/set_param'.format(node_name), 
                BlobFinderSetParam, 
                self.handle_set_param_srv
                )
        self.get_param_srv = rospy.Service( 
                '{0}/get_param'.format(node_name), 
                BlobFinderGetParam, 
                self.handle_get_param_srv
                )
        self.ready = True

        
    def handle_set_param_srv(self, req):
        """
        Handles requests to set the blob finder's parameters. Currently this
        is just the threshold used for binarizing the image.
        """
        with self.lock:
            self.blobFinder.threshold = req.threshold
            self.blobFinder.filter_by_area = req.filter_by_area
            self.blobFinder.min_area = req.min_area
            self.blobFinder.max_area = req.max_area
        return BlobFinderSetParamResponse(True,'')

    def handle_get_param_srv(self,req):
        """
        Handles requests for the blob finders parameters
        """
        with self.lock:
            threshold = self.blobFinder.threshold
            filter_by_area = self.blobFinder.filter_by_area
            min_area = self.blobFinder.min_area
            max_area = self.blobFinder.max_area
        resp_args = (threshold, filter_by_area, min_area, max_area)
        return  BlobFinderGetParamResponse(*resp_args)

    def camera_info_callback(self,data):
        """
        Callback for camera info topic subscription - used to get the image seq number.
        """
        stamp_tuple = data.header.stamp.secs, data.header.stamp.nsecs
        with self.lock:
            self.stamp_to_seq[stamp_tuple] = data.header.seq

    def image_callback(self,data):
        """
        Callback for image topic subscription - finds the tracking points in the image. 
        """
        if not self.ready:
            return 

        with self.lock:
            blobs_list = self.blobFinder.findBlobs(data,create_image=False)

        # Convert to opencv image
        cv_image = self.bridge.imgmsg_to_cv(data,desired_encoding="passthrough")
        ipl_image = cv.GetImage(cv_image)

        # Create trakcing points  image
        image_tracking_pts = cv.CreateImage(
                self.tracking_pts_roi_size,
                cv.IPL_DEPTH_8U, 
                3
                )
        cv.Zero(image_tracking_pts)

        num_blobs = len(blobs_list)
        if num_blobs == 3:
            found = True
            uv_list = self.get_sorted_uv_points(blobs_list)
            self.tracking_pts_roi = self.get_tracking_pts_roi(uv_list,cv.GetSize(ipl_image))
        else:
            found = False
            uv_list = [(0,0),(0,0),(0,0)]

        # Create tracking pts image using ROI around tracking points
        if self.tracking_pts_roi is not None:
            cv.SetImageROI(ipl_image,self.tracking_pts_roi)
            cv.CvtColor(ipl_image,image_tracking_pts,cv.CV_GRAY2BGR)
            cv.ResetImageROI(ipl_image)
            if found:
                for i, uv in enumerate(uv_list):
                    color = self.tracking_pts_colors[i]
                    u,v = uv 
                    u = u - self.tracking_pts_roi[0]
                    v = v - self.tracking_pts_roi[1]
                    cv.Circle(image_tracking_pts, (int(u),int(v)),3, color)


        # Convert tracking points image to rosimage
        rosimage_tracking_pts = self.bridge.cv_to_imgmsg(image_tracking_pts,encoding="passthrough")

        # Add data to pool
        stamp_tuple = data.header.stamp.secs, data.header.stamp.nsecs
        with self.lock:
            self.stamp_to_data[stamp_tuple] = {
                    'found': found,
                    'tracking_pts': uv_list,
                    'image_tracking_pts' : rosimage_tracking_pts, 
                    }
            
        # Publish calibration progress image
        self.image_tracking_pts_pub.publish(rosimage_tracking_pts)


    def get_tracking_pts_roi(self, uv_list, image_size):
        """
        Get the coordinates of region of interest about the tracking points
        """
        image_width, image_height = image_size
        roi_width, roi_height = self.tracking_pts_roi_size
        u_mid, v_mid = get_midpoint_uv_list(uv_list)
        u_min = int(math.floor(u_mid - roi_width/2))
        v_min = int(math.floor(v_mid - roi_height/2))
        u_max = u_min + roi_width  
        v_max = v_min + roi_height 
        if u_min < 0:
            u_max = u_max + (-u_min)
            u_min = 0
        if u_max >= image_width:
            u_min = u_min - (u_max - image_width + 1)
            u_max = image_width-1
        if v_min < 0:
            v_max = v_max + (-v_min)
            v_min = 0
        if v_max >= image_height:
            v_min = v_min - (v_max - image_height + 1)
            v_max = image_height-1
        return u_min, v_min, u_max-u_min, v_max-v_min


    def get_sorted_uv_points(self,blobs_list):
        """
        For blob lists with three blobs finds the identities of the blobs based on
        colinearity and distance between the points.
        """
        assert len(blobs_list) == 3, 'blobs list must contain only thre blobs'
        # Get x and y point lists
        u_list = [blob['centroid_x'] for blob in blobs_list]
        v_list = [blob['centroid_y'] for blob in blobs_list]

        # Determine x and y span
        u_min, u_max = min(u_list), max(u_list)
        v_min, v_max = min(v_list), max(v_list)
        u_span = u_max - u_min
        v_span = v_max - v_min

        # Use max span to sort points
        if u_span >= v_span:
            uv_list = zip(u_list, v_list)
            uv_list.sort()
            u_list = [u for u,v in uv_list]
            v_list = [v for u,v in uv_list]
        else:
            vu_list = zip(v_list, u_list)
            vu_list.sort()
            u_list = [u for v,u in vu_list]
            v_list = [v for v,u in vu_list]

        # Look at distances and sort so that either the large gap occurs first
        uv_list= zip(u_list,v_list)
        dist_0_to_1 = distance_2d(uv_list[0], uv_list[1])
        dist_1_to_2 = distance_2d(uv_list[1], uv_list[2])
        if self.large_step_first:
            if dist_0_to_1 <= dist_1_to_2:
                uv_list.reverse()
        else:
            if dist_0_to_1 >= dist_1_to_2:
                uv_list.reverse()
        return uv_list 

    def run(self):
        """
        Main loop - associates tracking data and time stamps  w/ image sequence
        numbers and publishes the tracking data.
        """
        while not rospy.is_shutdown():
            with self.lock:
                # Associate data with image seq numbers
                for stamp, data in self.stamp_to_data.items():
                    try:
                        seq = self.stamp_to_seq[stamp]
                    except KeyError:
                        continue
                    self.seq_to_stamp_and_data[seq] = stamp, data 
                    del self.stamp_to_data[stamp]
                    del self.stamp_to_seq[stamp]

                # Publish data
                for seq, stamp_and_data in sorted(self.seq_to_stamp_and_data.items()):

                    stamp_tuple, data = stamp_and_data 

                    # Create list of tracking points 
                    tracking_pts = []
                    for u,v in data['tracking_pts']:
                        tracking_pts.append(ImagePt(u,v))

                    # Create the tracking points message and publish
                    tracking_pts_msg = ThreePointTrackerRaw()
                    tracking_pts_msg.data.seq = seq
                    tracking_pts_msg.data.secs = stamp[0]
                    tracking_pts_msg.data.nsecs = stamp[1]
                    tracking_pts_msg.data.camera = self.camera
                    tracking_pts_msg.data.found = data['found']
                    tracking_pts_msg.data.points = tracking_pts 
                    tracking_pts_msg.image = data['image_tracking_pts']
                    self.tracking_pts_pub.publish(tracking_pts_msg)
                    
                    del self.seq_to_stamp_and_data[seq]

            # Temporary - may want to get this from frame rate.
            rospy.sleep(0.01)


# -------------------------------------------------------------------------------
def get_midpoint_uv_list(uv_list):
    """
    Gets the mid point of the sorted 2d points in uv_list.
    """
    u_mid = 0.5*(uv_list[0][0] + uv_list[-1][0])
    v_mid = 0.5*(uv_list[0][1] + uv_list[-1][1])
    return u_mid, v_mid

def distance_2d(p,q):
    """
    Returns the distance between the 2d points p and q
    """
    return math.sqrt((p[0]-q[0])**2 + (p[1]-q[1])**2)

def get_camera_from_topic(topic):
    """
    Returns the camera name given the image topic.
    """
    topic_split = topic.split('/')
    return topic_split[2]

def get_camera_info_from_image_topic(topic):
    """
    Returns the camera info topic given an image topic from that camera
    """
    topic_split = topic.split('/')
    info_topic = topic_split[:-1]
    info_topic.append('camera_info')
    info_topic = '/'.join(info_topic)
    return info_topic

# -----------------------------------------------------------------------------
if __name__ == '__main__':
    topic = sys.argv[1]
    node = ThreePointTracker(topic)
    node.run()




