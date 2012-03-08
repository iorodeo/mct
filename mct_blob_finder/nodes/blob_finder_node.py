#!/usr/bin/env python
from __future__ import print_function
import roslib
roslib.load_manifest('mct_blob_finder')
import rospy
import threading
import sys

from mct_blob_finder import BlobFinder

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

        self.blobFinder = BlobFinder()
        self.blobFinder.threshold = 150
        self.blobFinder.filter_by_area = False
        self.blobFinder.min_area = 0
        self.blobFinder.max_area = 200

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

        # Publish image of blobs
        self.image_pub.publish(blobs_rosimage)

        # Create the blob data message and publish
        blob_data_msg = BlobData()
        blob_data_msg.header = data.header
        blob_data_msg.number_of_blobs = len(blobs_list)
        for b in blobs_list:
            blob_msg = Blob()
            for k, v in b.iteritems():
                setattr(blob_msg,k,v)
            blob_data_msg.blob.append(blob_msg)
        self.blob_data_pub.publish(blob_data_msg)

    def run(self):
        rospy.spin()

# -----------------------------------------------------------------------------
if __name__ == '__main__':

    topic = sys.argv[1]
    node = BlobFinderNode(topic)
    node.run()
