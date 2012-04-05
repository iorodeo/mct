#!/usr/bin/env python
from __future__ import print_function
import roslib
roslib.load_manifest('mct_tracking_2d')
import rospy
import sys
import threading
from mct_utilities import file_tools

from sensor_msgs.msg import Image
from mct_msg_and_srv.msg import SeqAndImage
from mct_msg_and_srv.msg import ThreePointTracker


class StitchedImageLabeler(object):

    def __init__(self, stitched_image_topic, tracking_pts_topic):
        self.lock = threading.Lock()
        self.stitched_image_topic = stitched_image_topic
        self.tracking_pts_topic = tracking_pts_topic
        self.ready = False
        rospy.init_node('stitched_image_labeler')

        # Subscribe to stitched image and tracking pts topics
        self.image_sub = rospy.Subscriber(
                self.stitched_image_topic,
                SeqAndImage,
                self.stitched_image_handler
                )
        self.tracking_pts_sub = rospy.Subscriber(
                self.tracking_pts_topic, 
                ThreePointTracker, 
                self.tracking_pts_handler
                )
        self.ready = True

    def stitched_image_handler(self,data):
        print('stitched_image_handler',data.seq)
        if not self.ready:
            return
        with self.lock:
            pass

    def tracking_pts_handler(self,data):
        print('tracking_pts_handler',data.seq)

        if not self.ready:
            return
        with self.lock:
            pass

    def run(self):
        while not rospy.is_shutdown():
            rospy.sleep(0.1)

# -----------------------------------------------------------------------------
if __name__ == '__main__':
    stitched_image_topic = sys.argv[1]
    tracking_pts_topic = sys.argv[2]
    node = StitchedImageLabeler(stitched_image_topic, tracking_pts_topic)
    node.run()
