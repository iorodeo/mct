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

class BlobFinderNode(object):

    def __init__(self,topic=None):
        self.topic = topic
        self.lock = threading.Lock()
        self.bridge = CvBridge()
        self.threshold = 100
        self.blob_min_area = 0
        self.blob_max_area = 100

        self.blob_mask  = cvblob.CV_BLOB_RENDER_COLOR 
        self.blob_mask |= cvblob.CV_BLOB_RENDER_CENTROID 
        #self.blob_mask |= cvblob.CV_BLOB_RENDER_BOUNDING_BOX 

        rospy.init_node('blob_finder')

        self.image_sub = rospy.Subscriber(self.topic,Image,self.image_handler)
        self.label_pub = rospy.Publisher('image_blobs', Image)

    def image_handler(self,data):
        cv_image = self.bridge.imgmsg_to_cv(data,desired_encoding="passthrough")
        raw_image = cv.GetImage(cv_image)
        cv.Threshold(raw_image, raw_image, self.threshold, 255, cv.CV_THRESH_BINARY)
        label_image = cv.CreateImage(cv.GetSize(raw_image), cvblob.IPL_DEPTH_LABEL, 1)

        blobs = cvblob.Blobs()
        result = cvblob.Label(raw_image, label_image, blobs)
        cvblob.FilterByArea(blobs,self.blob_min_area, self.blob_max_area)
        numblobs = len(blobs.keys())

        print('num: {0}'.format(numblobs))

        blob_image = cv.CreateImage(cv.GetSize(raw_image), cv.IPL_DEPTH_8U, 3)
        cv.Zero(blob_image)
        cvblob.RenderBlobs(label_image, blobs, raw_image, blob_image, self.blob_mask, 1.0)

        rosimage = self.bridge.cv_to_imgmsg(blob_image,encoding="passthrough")
        self.label_pub.publish(rosimage)


    def run(self):
        rospy.spin()

# -----------------------------------------------------------------------------
if __name__ == '__main__':

    topic = sys.argv[1]
    node = BlobFinderNode(topic)
    node.run()
