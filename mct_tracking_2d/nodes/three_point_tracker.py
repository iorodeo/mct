#!/usr/bin/env python
from __future__ import print_function
import roslib
roslib.load_manifest('mct_tracking_2d')
import rospy
import sys
import threading
import math

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
from mct_msg_and_srv.msg import Point2d 

# Services
from mct_msg_and_srv.srv import BlobFinderSetParam
from mct_msg_and_srv.srv import BlobFinderSetParamResponse
from mct_msg_and_srv.srv import BlobFinderGetParam
from mct_msg_and_srv.srv import BlobFinderGetParamResponse

class ThreePointTracker(object):

    """
    Three point object tracker. Looks for objects in the image stream with
    three bright unequally spaced points.
    """

    def __init__(self, topic=None, max_stamp_age=1.5):
        self.topic = topic
        self.lock = threading.Lock() 
        self.bridge = CvBridge()
        self.camera = get_camera_from_topic(self.topic)
        self.camera_info_topic = get_camera_info_from_image_topic(self.topic)
        self.max_stamp_age = max_stamp_age
        self.latest_stamp = None

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
        self.tracking_pts_roi_src = None
        self.tracking_pts_roi_dst = None
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
            self.latest_stamp = stamp_tuple
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

        # Create tracking points  image
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
            self.tracking_pts_roi = self.get_tracking_pts_roi(uv_list, cv.GetSize(ipl_image))
            dist_to_image_center = self.get_dist_to_image_center(uv_list, cv.GetSize(ipl_image))
        else:
            found = False
            dist_to_image_center = 0.0
            uv_list = [(0,0),(0,0),(0,0)]

        # Create tracking pts image using ROI around tracking points
        if self.tracking_pts_roi is not None:
            src_roi, dst_roi = truncate_roi(self.tracking_pts_roi, cv.GetSize(ipl_image))
            cv.SetImageROI(ipl_image, src_roi)
            cv.SetImageROI(image_tracking_pts, dst_roi)
            cv.CvtColor(ipl_image, image_tracking_pts, cv.CV_GRAY2BGR)
            cv.ResetImageROI(ipl_image)
            cv.ResetImageROI(image_tracking_pts)
            self.tracking_pts_roi_src = src_roi
            self.tracking_pts_roi_dst = dst_roi
            if found:
                for i, uv in enumerate(uv_list):
                    color = self.tracking_pts_colors[i]
                    u,v = uv 
                    u = u - self.tracking_pts_roi[0]
                    v = v - self.tracking_pts_roi[1]
                    cv.Circle(image_tracking_pts, (int(u),int(v)),3, color)

        # Convert tracking points image to rosimage and publish
        rosimage_tracking_pts = self.bridge.cv_to_imgmsg(image_tracking_pts,encoding="passthrough")
        self.image_tracking_pts_pub.publish(rosimage_tracking_pts)

        # Add data to pool
        stamp_tuple = data.header.stamp.secs, data.header.stamp.nsecs

        # If tracking points roi doesn't exist yet just send zeros
        if self.tracking_pts_roi is None:
            tracking_pts_roi = (0,0,0,0)
            tracking_pts_roi_src = (0,0,0,0)
            tracking_pts_roi_dst = (0,0,0,0)
        else:
            tracking_pts_roi = self.tracking_pts_roi
            tracking_pts_roi_src = self.tracking_pts_roi_src
            tracking_pts_roi_dst = self.tracking_pts_roi_dst

        with self.lock:
            self.stamp_to_data[stamp_tuple] = {
                    'found': found,
                    'tracking_pts': uv_list,
                    'image_tracking_pts': rosimage_tracking_pts, 
                    'dist_to_image_center': dist_to_image_center,
                    'tracking_pts_roi': tracking_pts_roi,
                    'tracking_pts_roi_src': tracking_pts_roi_src,
                    'tracking_pts_roi_dst': tracking_pts_roi_dst,
                    }

    def get_dist_to_image_center(self, uv_list, img_size):
        """
        Computes the distance from the mid point of the tracking points to the
        middle of the image.
        """
        img_mid = 0.5*img_size[0], 0.5*img_size[1] 
        pts_mid = get_midpoint_uv_list(uv_list)
        return distance_2d(pts_mid,img_mid)


    def get_tracking_pts_roi(self, uv_list, img_size, trunc=False):
        """
        Get the coordinates of region of interest about the tracking points
        """
        img_width, img_height = img_size
        roi_width, roi_height = self.tracking_pts_roi_size
        u_mid, v_mid = get_midpoint_uv_list(uv_list)
        u_min = int(math.floor(u_mid - roi_width/2))
        v_min = int(math.floor(v_mid - roi_height/2))
        u_max = u_min + roi_width  
        v_max = v_min + roi_height 
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
        # or last based on the spacing data.
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
                        seq_found = True
                    except KeyError:
                        seq_found = False

                    if seq_found:
                        self.seq_to_stamp_and_data[seq] = stamp, data 
                        del self.stamp_to_data[stamp]
                        del self.stamp_to_seq[stamp]
                    else:
                        # Throw away data greater than the maximum allowed age
                        if self.latest_stamp is not None:
                            latest_stamp_secs = stamp_tuple_to_secs(self.latest_stamp)
                            stamp_secs = stamp_tuple_to_secs(stamp)
                            stamp_age = latest_stamp_secs - stamp_secs
                            if stamp_age > self.max_stamp_age:
                                del self.stamp_to_data[stamp]

                # Publish data
                for seq, stamp_and_data in sorted(self.seq_to_stamp_and_data.items()):

                    stamp_tuple, data = stamp_and_data 

                    # Create list of tracking points 
                    tracking_pts = []
                    for u,v in data['tracking_pts']:
                        tracking_pts.append(Point2d(u,v))

                    # Create the tracking points message and publish
                    tracking_pts_msg = ThreePointTrackerRaw()
                    tracking_pts_msg.data.seq = seq
                    tracking_pts_msg.data.secs = stamp[0]
                    tracking_pts_msg.data.nsecs = stamp[1]
                    tracking_pts_msg.data.camera = self.camera
                    tracking_pts_msg.data.found = data['found']
                    tracking_pts_msg.data.distance = data['dist_to_image_center']
                    tracking_pts_msg.data.roi = data['tracking_pts_roi']
                    tracking_pts_msg.data.roi_src = data['tracking_pts_roi_src']
                    tracking_pts_msg.data.roi_dst = data['tracking_pts_roi_dst']
                    tracking_pts_msg.data.points = tracking_pts 
                    tracking_pts_msg.image = data['image_tracking_pts']
                    self.tracking_pts_pub.publish(tracking_pts_msg)
                    
                    # Remove data for this sequence number.
                    del self.seq_to_stamp_and_data[seq]


# -------------------------------------------------------------------------------
def truncate_roi(orig_roi, src_image_size):
    """
    Returns truncated ROI for source and destination images. Crops ROI so that
    image edges are handled correctly.
    """
    # Set x position of ROI
    if orig_roi[0] < 0:
        src_x = 0
        dst_x = -orig_roi[0]
        w = orig_roi[2] + orig_roi[0]
    else:
        src_x = orig_roi[0]
        dst_x = 0
        w = orig_roi[2]

    # Set y position of ROI
    if orig_roi[1] < 0:
        src_y = 0
        dst_y = -orig_roi[1]
        h = orig_roi[3] + orig_roi[1]
    else:
        src_y = orig_roi[1]
        dst_y = 0
        h = orig_roi[3]

    # Set width of ROI
    if (src_x + w) >= src_image_size[0]:
        w = src_image_size[0] - src_x - 1

    # Set height of ROI
    if (src_y + h) >= src_image_size[1]:
        h = src_image_size[1] - src_y - 1

    # Create source and destiniatin image ROI's
    src_roi = src_x, src_y, w, h
    dst_roi = dst_x, dst_y, w, h

    return src_roi, dst_roi

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

def stamp_tuple_to_secs(stamp):
    """
    Converts a stamp tuple (secs,nsecs) to seconds.
    """
    return stamp[0] + stamp[1]*1.0e-9

# -----------------------------------------------------------------------------
if __name__ == '__main__':
    topic = sys.argv[1]
    node = ThreePointTracker(topic)
    node.run()




