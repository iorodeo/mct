#!/usr/bin/env python
from __future__ import print_function
import roslib
roslib.load_manifest('mct_blob_finder')
import rospy
import threading
import cv
import cv2
import sys
from cv_bridge.cv_bridge import CvBridge 
from cv_bridge.cv_bridge import CvBridgeError
from mct_blob_finder import cvblob

# Messages
from sensor_msgs.msg import Image
from mct_msg_and_srv.msg import BlobData
from mct_msg_and_srv.msg import Blob

# Services
from mct_msg_and_srv.srv import BlobFinderSetParam
from mct_msg_and_srv.srv import BlobFinderSetParamResponse
from mct_msg_and_srv.srv import BlobFinderGetParam
from mct_msg_and_srv.srv import BlobFinderGetParamResponse

class BlobFinderNode(object):

    def __init__(self,topic=None):
        self.topic = topic
        self.lock = threading.Lock()
        self.bridge = CvBridge()
        self.threshold = 150
        self.filter_by_area = True 
        self.min_area = 0
        self.max_area = 200

        self.blob_mask  = cvblob.CV_BLOB_RENDER_COLOR 
        self.blob_mask |= cvblob.CV_BLOB_RENDER_CENTROID 
        #self.blob_mask |= cvblob.CV_BLOB_RENDER_BOUNDING_BOX 

        rospy.init_node('blob_finder')
        self.image_sub = rospy.Subscriber(self.topic,Image,self.image_callback)
        self.image_pub = rospy.Publisher('image_blobs', Image)
        self.blob_data_pub = rospy.Publisher('blob_data', BlobData)
        self.set_param_srv = rospy.Service( 
                'blob_finder_set_param', 
                BlobFinderSetParam, 
                self.handle_set_param_srv
                )
        self.get_param_srv = rospy.Service( 
                'blob_finder_get_param', 
                BlobFinderGetParam, 
                self.handle_get_param_srv
                )

    def handle_set_param_srv(self, req):
        """
        Handles requests to set the blob finder's parameters. Currently this
        is just the threshold used for binarizing the image.
        """
        with self.lock:
            self.threshold = req.threshold
            self.filter_by_area = req.filter_by_area
            self.min_area = req.min_area
            self.max_area = req.max_area
        return BlobFinderSetParamResponse(True,'')

    def handle_get_param_srv(self,req):
        """
        Handles requests for the blob finders parameters
        """
        with self.lock:
            threshold = self.threshold
            filter_by_area = self.filter_by_area
            min_area = self.min_area
            max_area = self.max_area
        resp_args = (threshold, filter_by_area, min_area, max_area)
        return  BlobFinderGetParamResponse(*resp_args)


    def image_callback(self,data):
        """
        Callback for image topic subscription - finds blobs in image.
        """
        with self.lock:
            threshold = self.threshold

        cv_image = self.bridge.imgmsg_to_cv(data,desired_encoding="passthrough")
        raw_image = cv.GetImage(cv_image)
        thresh_image = cv.CreateImage(cv.GetSize(raw_image),raw_image.depth, raw_image.channels)

        # Find blobs in image
        cv.Threshold(raw_image, thresh_image, threshold, 255, cv.CV_THRESH_BINARY)
        label_image = cv.CreateImage(cv.GetSize(raw_image), cvblob.IPL_DEPTH_LABEL, 1)

        blobs = cvblob.Blobs()
        result = cvblob.Label(thresh_image, label_image, blobs)

        # Filter blobs by area
        if self.filter_by_area:
            cvblob.FilterByArea(blobs,self.min_area,self.max_area)

        blob_image = cv.CreateImage(cv.GetSize(raw_image), cv.IPL_DEPTH_8U, 3)
        cv.CvtColor(raw_image,blob_image,cv.CV_GRAY2BGR)

        cvblob.RenderBlobs(label_image, blobs, raw_image, blob_image, self.blob_mask, 1.0)
        blob_rosimage = self.bridge.cv_to_imgmsg(blob_image,encoding="passthrough")
        self.image_pub.publish(blob_rosimage)

        # Create the blob data message and publish
        blob_data_msg = BlobData()
        blob_data_msg.header = data.header
        blob_data_msg.number_of_blobs = len(blobs.keys())

        for k in blobs:
            blob_msg = Blob()
            centroid = cvblob.Centroid(blobs[k])
            blob_msg.centroid_x = centroid[0]
            blob_msg.centroid_y = centroid[1]
            angle = cvblob.Angle(blobs[k])
            blob_msg.angle = angle
            blob_msg.area = blobs[k].area
            blob_msg.min_x = blobs[k].minx
            blob_msg.max_x = blobs[k].maxx
            blob_msg.min_y = blobs[k].miny
            blob_msg.max_y = blobs[k].maxy
            blob_data_msg.blob.append(blob_msg)

        self.blob_data_pub.publish(blob_data_msg)

    def run(self):
        rospy.spin()

# -----------------------------------------------------------------------------
if __name__ == '__main__':

    topic = sys.argv[1]
    node = BlobFinderNode(topic)
    node.run()
