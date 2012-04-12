#!/usr/bin/env python
from __future__ import print_function
import roslib
roslib.load_manifest('mct_watchdog')
import rospy
import threading
import functools
import mct_introspection
import numpy

# Services
from std_srvs.srv import Empty
from std_srvs.srv import EmptyResponse

# Messages
from sensor_msgs.msg import CameraInfo
from mct_msg_and_srv.msg import TimeStampWatchDog

class TimeStampWatchdog(object):

    """
    Watchdog for hardware triggered cameras. Checks for correct sequence order
    and maximum allowed time stamp error.
    """
    def __init__(self, frame_rate=30.0, max_allowed_error=1.0e-3, max_seq_age=200):


        self.lock = threading.Lock()
        self.frame_rate = frame_rate
        self.max_allowed_error = max_allowed_error
        self.max_seq_age = max_seq_age

        self.seq_to_camera_stamps = {}
        self.stamp_last = {}

        self.max_error_by_camera = {}
        self.max_error = 0.0

        self.most_recent_seq = None
        self.last_checked_seq = None

        self.ok = True
        self.seq_fail = 0 
        self.error_msg = ''

        self.ready = False
        rospy.init_node('time_stamp_watchdog')

        # Subscribe to camera info topics
        camera_info_topics = mct_introspection.find_camera_info_topics()
        self.camera_info_sub = {}
        for topic in camera_info_topics:
            camera = get_camera_from_topic(topic)
            handler = functools.partial(self.camera_info_handler, camera)
            self.camera_info_sub[camera] = rospy.Subscriber(topic, CameraInfo, handler)
        self.number_of_cameras = len(camera_info_topics)

        # Create reset service
        self.reset_srv = rospy.Service('time_stamp_watchdog_reset', Empty, self.reset_handler)

        # Create time stamp watchdog publication
        self.watchdog_pub = rospy.Publisher('time_stamp_watchdog', TimeStampWatchDog)
        self.ready = True

    def reset_handler(self,req):
        """
        Handles reset service
        """
        with self.lock:
            self.seq_to_camera_stamps = {}
            self.stamp_last = {}
            self.max_error_by_camera = {}
            self.max_error = 0.0
            self.most_recent_seq = None
            self.last_checked_seq = None
            self.ok = True
            self.seq_fail = 0 
            self.error_msg = ''
        return EmptyResponse()

    def camera_info_handler(self, camera, data):
        """
        Handler for camera info messages. Stores the time stamps based on
        seqeunce number and camera.
        """
        if not self.ready:
            return

        with self.lock:
            seq = data.header.seq
            stamp = data.header.stamp
            try:
                self.seq_to_camera_stamps[seq][camera] = stamp
            except KeyError:
                self.seq_to_camera_stamps[seq] = {camera: stamp}
            self.most_recent_seq = seq

    def check_camera_stamps(self, seq, camera_stamps):
        """
        Checks the time stamps for a given sequence number. Examines the
        sequence number to ensure it is the next expected and exaimes the
        spread of the time stamps to ensure that is less than the allowed
        maximum. The results are published on the watchdog topic.
        """
        cur_seq_ok = True
        error_list = []
        # Check for correct sequence order
        if self.last_checked_seq is not None:
            if seq != self.last_checked_seq+1:
                cur_seq_ok = False
                self.seq_fail = seq
                error_list.append('sequence out of order')
        self.last_checked_seq = seq

        # Get the max and min time stamps for cameras on the same machine
        stamp_delta = {}
        stamp_error = {}

        for camera, stamp in camera_stamps.iteritems():
            # Get differenece between current and last time stamps and compute the stamp error
            try:
                stamp_delta[camera] = stamp.to_sec() - self.stamp_last[camera].to_sec()
            except KeyError:
                stamp_delta[camera] = 1.0/self.frame_rate 
            stamp_error[camera] = abs(1.0/self.frame_rate - stamp_delta[camera])

            # Find the maximum error so far for each camera
            try:
                self.max_error_by_camera[camera] = max([self.max_error_by_camera[camera], stamp_error[camera]])
            except KeyError:
                self.max_error_by_camera[camera] = stamp_error[camera]

            self.stamp_last[camera] = stamp

        # Compute the maximum error so far for all cameras
        self.max_error = max([err for err in self.max_error_by_camera.values()])
        if self.max_error > self.max_allowed_error:
            cur_seq_ok = False
            self.seq_fail = seq
            error_list.append('time stamp outside of expected range')


        # If this is the first seq with an error set the error message
        if self.ok and not cur_seq_ok:
            self.error_msg = ', '.join(error_list)

        # Publish watchdog message
        watchdog_msg = TimeStampWatchDog()
        watchdog_msg.seq = seq
        watchdog_msg.ok = self.ok 
        watchdog_msg.frame_rate = self.frame_rate
        watchdog_msg.max_allowed_error = self.max_allowed_error
        watchdog_msg.max_error = self.max_error
        watchdog_msg.seq_fail = self.seq_fail
        watchdog_msg.error_msg = self.error_msg
        self.watchdog_pub.publish(watchdog_msg)


    def process_camera_stamps(self):
        """
        Processes the camera time stamps. First, looks to see if all the images
        have been recieved for a given sequence number. Second, check if any of
        the sequences are older than the maximum allowed age and if so throws
        them away. 
        """
        for seq, camera_stamps in sorted(self.seq_to_camera_stamps.items()):
            if len(camera_stamps) == self.number_of_cameras:
                self.check_camera_stamps(seq, camera_stamps)
                del self.seq_to_camera_stamps[seq]
            else:
                if self.most_recent_seq is not None:
                    seq_age = self.most_recent_seq - seq
                    if seq_age > self.max_seq_age:
                        del self.seq_to_camera_stamps[seq]

    def run(self):
        """
        Node, main loop.  While the node
        """
        while not rospy.is_shutdown():
            with self.lock:
                self.process_camera_stamps()

# Utility functions
# ----------------------------------------------------------------------------

def get_camera_from_topic(topic):
    camera = topic.split('/')[2]
    return camera

def get_machine_from_topic(topic):
    machine = topic.split('/')[1]
    return machine

# -----------------------------------------------------------------------------
if __name__ == '__main__':

    node = TimeStampWatchdog()
    node.run()
