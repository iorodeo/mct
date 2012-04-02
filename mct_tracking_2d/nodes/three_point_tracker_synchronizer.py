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
from mct_transform_2d import transform_2d

from mct_msg_and_srv.msg import Point2d
from mct_msg_and_srv.msg import ThreePointTracker
from mct_msg_and_srv.msg import ThreePointTrackerRaw

class ThreePointTracker_Synchronizer:
    """
    Synchronization node for all three point tracker in a given tracking region.
    """

    def __init__(self,region,max_seq_age=200):
        self.lock = threading.Lock()
        self.region = region
        regions_dict = file_tools.read_tracking_2d_regions()
        self.camera_list = regions_dict[region]
        self.create_camera_to_tracking_dict()
        self.latest_seq = None
        self.max_seq_age = max_seq_age
        self.tracking_pts_pool = {}

        # Get transforms from cameras to tracking and stitched image planes
        self.tf2d = transform_2d.Transform2d()

        self.ready = False
        rospy.init_node('three_point_tracker_synchronizer')

        # Subscribe to raw tracking pts topics
        self.tracking_pts_sub = {}
        for camera, topic in self.camera_to_tracking.iteritems():
            handler = functools.partial(self.tracking_pts_handler, camera)
            self.tracking_pts_sub[camera] = rospy.Subscriber(
                    topic,
                    ThreePointTrackerRaw,
                    handler
                    )

        # Create publishers
        self.tracking_pts_pub = rospy.Publisher('tracking_pts', ThreePointTracker)

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
        """
        Determines whether the object has been found in any of the three point
        trackers for the region. If is has been found than best tracking point
        data is selected in a winner takes all fashion. The best tracking point
        data is that which is nearest to the center of the image on camera upon
        which is was captured. 
        """
        # Get list of messages in which the object was found
        found_list = [msg for msg in tracking_pts_dict.values() if msg.data.found]
        tracking_pts_msg = ThreePointTracker()
        if found_list:
            # Object found - select points whose distance to center of image is the smallest 
            found_list.sort(cmp=tracking_pts_sort_cmp)
            best = found_list[0]
            camera = best.data.camera

            # Get coords of points in tracking and stitching planes
            pts_tracking_plane, pts_stitching_plane = [], []
            for p in best.data.points:
                x,y = self.tf2d.camera_pts_to_anchor_plane(camera, p.x, p.y)
                pts_tracking_plane.append(Point2d(x,y))
                x,y = self.tf2d.camera_pts_to_stitching_plane(camera, p.x, p.y)
                pts_stitching_plane.append(Point2d(x,y))

            # Get mid point of object in tracking and stitching planes
            midpt_tracking_plane = get_midpoint(pts_tracking_plane)
            midpt_stitching_plane = get_midpoint(pts_stitching_plane)

            # Transform image to stitching plane

            # Publish tracking data
            tracking_pts_msg.found = True
            tracking_pts_msg.camera = camera
            tracking_pts_msg.pts_tracking_plane = pts_tracking_plane
            tracking_pts_msg.pts_stitching_plane = pts_stitching_plane
            tracking_pts_msg.midpt_tracking_plane = midpt_tracking_plane
            tracking_pts_msg.midpt_stitching_plane = midpt_stitching_plane
        else:
            tracking_pts_msg.found = False

        self.tracking_pts_pub.publish(tracking_pts_msg)

    def run(self):
        """
        Node main loop - consolidates the tracking points messages by
        acquisition sequence number and proccess them by passing them to the
        process_tracking_pts function.
        """
        while not rospy.is_shutdown():

            with self.lock:

                for seq, tracking_pts_dict in sorted(self.tracking_pts_pool.items()):

                    # Check if we have all the tracking data for the given sequence number
                    if len(tracking_pts_dict) == len(self.camera_list):
                        self.process_tracking_pts(tracking_pts_dict)
                        del self.tracking_pts_pool[seq]

                    # Throw away tracking data greater than the maximum allowed age  
                    if self.latest_seq - seq > self.max_seq_age:
                        print('Thowing away: ', seq)
                        del self.tracking_pts_pool[seq]

# Utility functions
# -----------------------------------------------------------------------------
def tracking_pts_sort_cmp(msg_x, msg_y):
    """
    Comparison function for sorting the the tracking pts messages. Used for sorting
    the based on the distance to the center of the image in which they were found.
    """
    dist_x = msg_x.data.distance
    dist_y = msg_y.data.distance
    if dist_x > dist_y:
        return 1
    elif dist_y > dist_x:
        return -1
    else:
        return 0

def get_midpoint(point_list):
    """
    Gets the mid point of the sorted 2d points in the given list.
    """
    x_mid = 0.5*(point_list[0].x + point_list[-1].x)
    y_mid = 0.5*(point_list[0].y + point_list[-1].y)
    return Point2d(x_mid, y_mid)
# -----------------------------------------------------------------------------
if __name__  == '__main__':
    region = sys.argv[1]
    node = ThreePointTracker_Synchronizer(region)
    node.run()



