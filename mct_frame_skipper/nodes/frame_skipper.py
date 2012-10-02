#!/usr/bin/env python
from __future__ import print_function
import roslib
roslib.load_manifest('mct_frame_skipper')
import rospy
import sys
import threading

import cv
from cv_bridge.cv_bridge import CvBridge 
from mct_blob_finder import BlobFinder
from mct_utilities import file_tools

# Messages
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo


class FrameSkipper(object):

    """
    Frame skipper node - subscribes to the given image topic and re-published only 
    those frames which are divisible by the frame skip parameter.
    """

    def __init__(self, topic=None, skip_param=3, max_stamp_age=1.5):
        self.ready = False
        self.topic = topic
        self.skip_param = skip_param
        self.lock = threading.Lock() 
        self.camera_info_topic = get_camera_info_from_image_topic(self.topic)
        self.max_stamp_age = max_stamp_age
        self.latest_stamp = None


        # Data dictionareis for synchronizing tracking data with image seq number
        self.stamp_to_image = {}
        self.stamp_to_seq = {}
        self.seq_to_stamp_and_image = {}

        rospy.init_node('frame_skipper')

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

        # Set up image publisher
        repub_topic = '{0}_skip'.format(self.topic)
        print(repub_topic)
        self.image_repub = rospy.Publisher(repub_topic,Image)

        self.ready = True


    def camera_info_callback(self,data):
        """
        Callback for camera info topic subscription - used to associate stamp
        with the image seq number.
        """
        if not self.ready:
            return
        stamp_tuple = data.header.stamp.secs, data.header.stamp.nsecs
        with self.lock:
            self.latest_stamp = stamp_tuple
            self.stamp_to_seq[stamp_tuple] = data.header.seq

    def image_callback(self,data):
        """
        Callback for image topic subscription - used to associate the stamp tuple with
        the imcomming image.
        """
        if not self.ready:
            return 
        stamp_tuple = data.header.stamp.secs, data.header.stamp.nsecs
        with self.lock:
            self.stamp_to_image[stamp_tuple] = data


    def run(self):
        """
        Main loop - associates images and time stamps  w/ image sequence
        numbers and re-publishes images whose sequence numbers are divisible by
        the frame skip parameter.
        """
        while not rospy.is_shutdown():

            with self.lock:

                # Associate data with image seq numbers
                for stamp, image in self.stamp_to_image.items():
                    try:
                        seq = self.stamp_to_seq[stamp]
                        seq_found = True
                    except KeyError:
                        seq_found = False

                    if seq_found:
                        self.seq_to_stamp_and_image[seq] = stamp, image 
                        try:
                            del self.stamp_to_image[stamp]
                        except KeyError:
                            pass
                        try:
                            del self.stamp_to_seq[stamp]
                        except KeyError:
                            pass
                       
                    else:
                        # Throw away data greater than the maximum allowed age
                        if self.latest_stamp is not None:
                            latest_stamp_secs = stamp_tuple_to_secs(self.latest_stamp)
                            stamp_secs = stamp_tuple_to_secs(stamp)
                            stamp_age = latest_stamp_secs - stamp_secs
                            if stamp_age > self.max_stamp_age:
                                try:
                                    del self.stamp_to_image[stamp]
                                except KeyError:
                                    pass

                # Re-publish image 
                for seq, stamp_and_image in sorted(self.seq_to_stamp_and_image.items()):
                    stamp_tuple, image = stamp_and_image 
                    if seq%self.skip_param == 0:
                        self.image_repub.publish(image)
                    del self.seq_to_stamp_and_image[seq]


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
    skip_param = int(sys.argv[2])
    node = FrameSkipper(topic,skip_param)
    node.run()




