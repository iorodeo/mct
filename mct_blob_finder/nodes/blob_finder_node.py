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

# Services
from mct_msg_and_srv.srv import BlobFinderParam
from mct_msg_and_srv.srv import BlobFinderParamResponse

class BlobFinderNode(object):

    def __init__(self,topic=None):
        self.topic = topic
        self.lock = threading.Lock()
        self.bridge = CvBridge()
        self.threshold = 100

        self.blob_mask  = cvblob.CV_BLOB_RENDER_COLOR 
        self.blob_mask |= cvblob.CV_BLOB_RENDER_CENTROID 
        #self.blob_mask |= cvblob.CV_BLOB_RENDER_BOUNDING_BOX 

        rospy.init_node('blob_finder')
        self.image_sub = rospy.Subscriber(self.topic,Image,self.image_callback)
        self.image_pub = rospy.Publisher('image_blobs', Image)
        self.blob_data_pub = rospy.Publisher('blob_data', BlobData)
        self.param_srv = rospy.Service( 
                'blob_finder_param', 
                BlobFinderParam, 
                self.handle_param_srv
                )

    def handle_param_srv(self, req):
        """
        Handles requests to set the blob finder's parameters. Currently this
        is just the threshold used for binarizing the image.
        """
        with self.lock:
            self.threshold = req.threshold
        return BlobFinderParamResponse(True,'')

    def image_callback(self,data):
        """
        Callback for image topic subscription - finds blobs in image.
        """
        with self.lock:
            threshold = self.threshold

        cv_image = self.bridge.imgmsg_to_cv(data,desired_encoding="passthrough")
        raw_image = cv.GetImage(cv_image)
        cv.Threshold(raw_image, raw_image, threshold, 255, cv.CV_THRESH_BINARY)
        label_image = cv.CreateImage(cv.GetSize(raw_image), cvblob.IPL_DEPTH_LABEL, 1)
        blobs = cvblob.Blobs()
        result = cvblob.Label(raw_image, label_image, blobs)

        blob_image = cv.CreateImage(cv.GetSize(raw_image), cv.IPL_DEPTH_8U, 3)
        cv.Zero(blob_image)
        cvblob.RenderBlobs(label_image, blobs, raw_image, blob_image, self.blob_mask, 1.0)

        rosimage = self.bridge.cv_to_imgmsg(blob_image,encoding="passthrough")
        self.image_pub.publish(rosimage)

        blob_data = BlobData()
        blob_data.header = data.header
        blob_data.number_of_blobs = len(blobs.keys())
        for k in blobs:
            centroid = cvblob.Centroid(blobs[k])
            blob_data.centroid_x.append(centroid[0])
            blob_data.centroid_y.append(centroid[1])
            angle = cvblob.Angle(blobs[k])
            blob_data.angle.append(angle)
            blob_data.area.append(blobs[k].area)
            blob_data.min_x.append(blobs[k].minx)
            blob_data.max_x.append(blobs[k].maxx)
            blob_data.min_y.append(blobs[k].miny)
            blob_data.max_y.append(blobs[k].maxy)
        self.blob_data_pub.publish(blob_data)

    def run(self):
        rospy.spin()

# -----------------------------------------------------------------------------
if __name__ == '__main__':

    topic = sys.argv[1]
    node = BlobFinderNode(topic)
    node.run()
