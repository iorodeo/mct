#!/usr/bin/env python
from __future__ import print_function
import roslib
roslib.load_manifest('mct_blob_finder')
import rospy
import threading
import sys
import math

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

        self.blobFinder = BlobFinder()
        self.blobFinder.threshold = 150
        self.blobFinder.filter_by_area = False
        self.blobFinder.min_area = 0
        self.blobFinder.max_area = 200

        self.circle_color = (0,0,255)

        rospy.init_node('zoom_tool')
        self.image_sub = rospy.Subscriber(self.topic,Image,self.image_callback)
        self.image_pub = rospy.Publisher('image_zoom_tool', Image)
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
            x0 = blobs_list[0]['centroid_x']
            y0 = blobs_list[0]['centroid_y']
            x1 = blobs_list[1]['centroid_x']
            y1 = blobs_list[1]['centroid_y']
            point_list = [(x0,y0),(x1,y1)]
            dist = math.sqrt((x0-x1)**2 + (y0-y1)**2)
            for x,y in point_list:
                cv.Circle(calib_image, (int(x),int(y)),3, self.circle_color)
            message = 'dist = {0:1.2f}'.format(dist)
            cv.PutText(calib_image,message,(10,25),self.cv_text_font,color)
        else:
            self.image_pub.publish(data)


    def run(self):
        rospy.spin()

# -----------------------------------------------------------------------------
if __name__ == '__main__':

    topic = sys.argv[1]
    node = ZoomToolNode()
    node.run()
