#!/usr/bin/env python
from __future__ import print_function
import roslib
roslib.load_manifest('mct_blob_finder')
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
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo 
from mct_msg_and_srv.msg import ThreePointTrackerData
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

        # Get tracking parameters
        self.tracking_pts_roi_size = 200,200

        # Blob finder parameters
        self.blobFinder = BlobFinder()
        self.blobFinder.threshold = 150 
        self.blobFinder.filter_by_area = False
        self.blobFinder.min_area = 0
        self.blobFinder.max_area = 200

        # Tracking parameters
        self.tracking_pts_spacing = (0.0, 0.04774, 0.07019)
        self.tracking_pts_colors = [(0,0,255), (0,255,0), (0,255,255)]
        d0 = self.tracking_pts_spacing[1] - self.tracking_pts_spacing[0]
        d1 = self.tracking_pts_spacing[2] - self.tracking_pts_spacing[1]
        if d0 > d1:
            self.large_step_first = True
        else:
            self.large_step_first = False

        # Data dictionareis for synchronizing tracking data with image seq number
        self.stamp_to_data = {}
        self.stamp_to_seq = {}
        self.seq_to_data = {}

        self.ready = False
        self.tracking_pts_image = None
        rospy.init_node('blob_finder')
        self.image_sub = rospy.Subscriber(self.topic,Image,self.image_callback)
        self.info_sub = rospy.Subscriber(self.topic, CameraInfo, self.camera_info_callback)

        self.image_tracking_pts_pub = rospy.Publisher('image_tracking_pts', Image)
        self.tracking_data_pub = rospy.Publisher('tracking_data', ThreePointTrackerData)
        self.devel_pub = rospy.Publisher('devel_data', Float32)

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
        self.stamp_to_seq[stamp_tuple] = data.header.seq

    def image_callback(self,data):
        """
        Callback for image topic subscription - finds the tracking points in the image. 
        """
        if not self.ready:
            return 
        
        with self.lock:
            blobs_list = self.blobFinder.findBlobs(data,create_image=False)

        # Create trakcing points  image
        cv_image = self.bridge.imgmsg_to_cv(data,desired_encoding="passthrough")
        ipl_image = cv.GetImage(cv_image)


        if self.tracking_pts_image is None:
            #self.tracking_pts_image = cv.CreateImage(cv.GetSize(ipl_image), cv.IPL_DEPTH_8U, 3)
            self.tracking_pts_image = cv.CreateImage(
                    self.tracking_pts_roi_size,
                    cv.IPL_DEPTH_8U, 
                    3
                    )
        cv.Zero(self.tracking_pts_image)
        #cv.CvtColor(ipl_image,self.tracking_pts_image,cv.CV_GRAY2BGR)


        num_blobs = len(blobs_list)
        print(num_blobs)
        if num_blobs == 3:
            found_pts = True
            uv_list = self.get_sorted_uv_points(blobs_list)
            roi = self.get_tracking_pts_roi(uv_list,cv.GetSize(ipl_image))

            cv.SetImageROI(ipl_image,roi)
            cv.CvtColor(ipl_image,self.tracking_pts_image,cv.CV_GRAY2BGR)
            cv.ResetImageROI(ipl_image)

            for i, uv in enumerate(uv_list):
                color = self.tracking_pts_colors[i]
                u,v = uv 
                u = u - roi[0]
                v = v - roi[1]
                cv.Circle(self.tracking_pts_image, (int(u),int(v)),3, color)
        else:
            found_pts = False
            uv_list = [(0,0),(0,0),(0,0)]

        # Add data to pool
        stamp_tuple = data.header.stamp.secs, data.header.stamp.nsecs
        self.stamp_to_data[stamp_tuple] = {
                'uv_list': uv_list,
                'found_pts': found_pts,
                }
            
        # Publish calibration progress image
        tracking_pts_rosimage = self.bridge.cv_to_imgmsg(self.tracking_pts_image,encoding="passthrough")
        self.image_tracking_pts_pub.publish(tracking_pts_rosimage)


    def get_tracking_pts_roi(self, uv_list, image_size):
        image_width, image_height = image_size
        roi_width, roi_height = self.tracking_pts_roi_size
        u_mid, v_mid = get_midpoint_uv_list(uv_list)
        u_min = int(math.floor(u_mid - roi_width/2))
        u_max = int(math.floor(u_mid + roi_width/2))
        v_min = int(math.floor(v_mid - roi_height/2))
        v_max = int(math.floor(v_mid + roi_height/2))
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
        #print('u:', u_min, u_max, u_max - u_min)
        #print('v:', v_min, v_max, v_max - v_min)
        #print()
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
        while not rospy.is_shutdown():
            with self.lock:
                #print(len(self.stamp_to_seq), len(self.stamp_to_data), len(self.seq_to_data))

                # Associate data with image seq numbers
                for stamp, data in self.stamp_to_data.items():
                    try:
                        seq = self.stamp_to_seq[stamp]
                    except KeyError:
                        continue
                    self.seq_to_data[seq] = data
                    del self.stamp_to_data[stamp]
                    del self.stamp_to_seq[stamp]

                # Publish data
                for seq, data in sorted(self.seq_to_data.items()):
                    # To do create tracking_data message and publish
                    del self.seq_to_data[seq]

            #rospy.sleep(0.1)



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

# -----------------------------------------------------------------------------
if __name__ == '__main__':
    topic = sys.argv[1]
    node = ThreePointTracker(topic)
    node.run()




