#!/usr/bin/env python
from __future__ import print_function
import roslib
roslib.load_manifest('mct_tracking_2d')
import rospy
import sys
import functools
import threading

import mct_introspection
from mct_utilities import file_tools

from mct_msg_and_srv.msg import ThreePointTrackerRaw

class ThreePointTracker_Synchronizer:

    def __init__(self,region,max_seq_age=200):
        self.lock = threading.Lock()
        self.region = region
        regions_dict = file_tools.read_tracking_2d_regions()
        self.camera_list = regions_dict[region]
        self.create_camera_to_tracking_dict()
        self.latest_seq = None
        self.max_seq_age = max_seq_age
        self.tracking_pts_pool = {}

        self.ready = False
        rospy.init_node('three_point_tracker_synchronizer')

        # Subscribe to tracking pts topics
        self.tracking_pts_sub = {}
        for camera, topic in self.camera_to_tracking.iteritems():
            handler = functools.partial(self.tracking_pts_handler, camera)
            self.tracking_pts_sub[camera] = rospy.Subscriber(
                    topic,
                    ThreePointTrackerRaw,
                    handler
                    )

        self.ready = True

    def create_camera_to_tracking_dict(self):
        """
        Creates a dictionary relating the cameras in the tracking region to their
        corresponding three point tracker nodes.
        """
        self.camera_to_tracking = {}
        tracking_pts_topics = mct_introspection.find_topics_w_name('tracking_pts')
        for topic in tracking_pts_topics:
            topic_split = topic.split('/')
            for camera in self.camera_list:
                if camera in topic_split:
                    self.camera_to_tracking[camera] = topic


    def tracking_pts_handler(self,camera,msg):
        """
        Handler for messages from the individual tracker nodes. Sticks the tracking
        point data into a dictionary by sequence number and camera.
        """
        if not self.ready:
            return 
        with self.lock:
            self.latest_seq = msg.data.seq
            try:
                self.tracking_pts_pool[msg.data.seq][camera] = msg
            except KeyError:
                self.tracking_pts_pool[msg.data.seq] = {camera: msg}


    def process_tracking_pts(self,tracking_pts_dict):
        found = False
        for camera, msg in sorted(tracking_pts_dict.items(),cmp=camera_name_cmp):
            found |= msg.data.found
            print(camera, msg.data.found)
        print(found)
        print()

    def run(self):
        """
        Node main loop - consolidates the tracking points messages by acquisition
        sequence number.
        """
        while not rospy.is_shutdown():
            #print('len(tracking_pts_pool', len(self.tracking_pts_pool))
            with self.lock:
                for seq, tracking_pts_dict in sorted(self.tracking_pts_pool.items()):
                    # For sequences where we have data from all cameras process the 
                    # the tracking points messages
                    if len(tracking_pts_dict) == len(self.camera_list):
                        self.process_tracking_pts(tracking_pts_dict)
                        del self.tracking_pts_pool[seq]
                    # Check to see if the sequence is older that maximum allowed age 
                    # and if so throw it away.
                    if self.latest_seq - seq > self.max_seq_age:
                        print('Thowing away: ', seq)
                        del self.tracking_pts_pool[seq]

def camera_name_cmp(x,y):
    num_x = int(x[0].split('_')[1])
    num_y = int(y[0].split('_')[1])
    if num_x > num_y:
        return 1
    elif num_y > num_x:
        return -1
    else:
        return 0


# -----------------------------------------------------------------------------
if __name__  == '__main__':
    region = sys.argv[1]
    node = ThreePointTracker_Synchronizer(region)
    node.run()



