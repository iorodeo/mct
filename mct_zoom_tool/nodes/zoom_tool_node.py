#!/usr/bin/env python
from __future__ import print_function
import roslib
roslib.load_manifest('mct_zoom_tool')
import rospy
import threading
import sys
import math
import cv
import cv2

from cv_bridge.cv_bridge import CvBridge 
from mct_blob_finder import BlobFinder

# Messages
from sensor_msgs.msg import Image

# Services
from mct_msg_and_srv.srv import BlobFinderSetParam
from mct_msg_and_srv.srv import BlobFinderSetParamResponse
from mct_msg_and_srv.srv import BlobFinderGetParam
from mct_msg_and_srv.srv import BlobFinderGetParamResponse

class ZoomToolNode(object):

    def __init__(self,topic=None):
        self.topic = topic
        self.lock = threading.Lock()
        self.bridge = CvBridge()
        rospy.init_node('zoom_tool')
        node_name = rospy.get_name()

        self.blobFinder = BlobFinder()
        self.blobFinder.threshold = rospy.get_param('/zoom_tool_params/threshold', 150)
        self.blobFinder.filter_by_area = rospy.get_param('/zoom_tool_params/filter_by_area', False) 
        self.blobFinder.min_area = rospy.get_param('/zoom_tool_params/min_area', 0) 
        self.blobFinder.max_area = rospy.get_param('/zoom_tool_params/max_area', 200) 

        self.circle_color = (0,0,255)
        self.text_color = (0,0,255)
        self.cv_text_font = cv.InitFont(cv.CV_FONT_HERSHEY_TRIPLEX, 0.8, 0.8,thickness=1)

        self.image_sub = rospy.Subscriber(self.topic,Image,self.image_callback)
        self.image_pub = rospy.Publisher('image_zoom_tool', Image)
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


    def image_callback(self,data):
        """
        Callback for image topic subscription - finds blobs in image.
        """
        with self.lock:
            blobs_list, blobs_rosimage = self.blobFinder.findBlobs(data)

        if len(blobs_list) == 2:

            # If two blobs are found computer the distance between them and
            # show it on the published image.

            # Create calibration data  image
            cv_image = self.bridge.imgmsg_to_cv(data,desired_encoding="passthrough")
            ipl_image = cv.GetImage(cv_image)
            calib_image = cv.CreateImage(cv.GetSize(ipl_image), cv.IPL_DEPTH_8U, 3)
            cv.CvtColor(ipl_image,calib_image,cv.CV_GRAY2BGR)
            x0 = blobs_list[0]['centroid_x']
            y0 = blobs_list[0]['centroid_y']
            x1 = blobs_list[1]['centroid_x']
            y1 = blobs_list[1]['centroid_y']
            point_list = [(x0,y0),(x1,y1)]
            dist = math.sqrt((x0-x1)**2 + (y0-y1)**2)
            for x,y in point_list:
                cv.Circle(calib_image, (int(x),int(y)),3, self.circle_color)
            message = 'dist = {0:1.1f} px'.format(dist)
            cv.PutText(calib_image,message,(10,25),self.cv_text_font,self.text_color)

            # Publish calibration progress image
            calib_rosimage = self.bridge.cv_to_imgmsg(calib_image,encoding="passthrough")
            self.image_pub.publish(calib_rosimage)
        else:
            self.image_pub.publish(data)


    def run(self):
        rospy.spin()

# -----------------------------------------------------------------------------
if __name__ == '__main__':

    topic = sys.argv[1]
    node = ZoomToolNode(topic)
    node.run()
