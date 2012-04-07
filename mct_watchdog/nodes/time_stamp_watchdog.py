#!/usr/bin/env python
from __future__ import print_function
import roslib
roslib.load_manifest('mct_watchdog')
import rospy
import threading
import functools
import mct_introspection
import numpy

from sensor_msgs.msg import CameraInfo
from mct_msg_and_srv.msg import TimeStampWatchDog

class TimeStampWatchdog(object):

    """
    Watchdog for hardware triggered cameras. Checks for correct sequence order
    and maximum allowed time stamp spread.
    """
    def __init__(self, max_seq_age=200, max_allowed_stamp_spread=10.0e-3):
        self.max_seq_age = max_seq_age
        self.max_allowed_stamp_spread = max_allowed_stamp_spread
        self.max_stamp_spread = 0.0
        self.lock = threading.Lock()
        self.seq_to_camera_stamps = {}
        self.ready = False
        self.most_recent_seq = None
        self.last_checked_seq = None
        self.seq_fail = 0 
        self.all_ok = True
        self.error_msg = ''
        rospy.init_node('time_stamp_watchdog')

        # Subscribe to camera info topics
        camera_info_topics = mct_introspection.find_camera_info_topics()
        self.camera_info_sub = {}
        for topic in camera_info_topics:
            camera = get_camera_from_topic(topic)
            handler = functools.partial(self.camera_info_handler,camera)
            self.camera_info_sub[camera] = rospy.Subscriber(topic, CameraInfo, handler)
        self.number_of_cameras = len(camera_info_topics)

        # Create time stamp watchdog publication
        self.watchdog_pub = rospy.Publisher('time_stamp_watchdog', TimeStampWatchDog)
        self.ready = True

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
        cur_ok = True
        error_list = []
        # Check for correct sequence order
        if self.last_checked_seq is not None:
            if seq != self.last_checked_seq+1:
                cur_ok = False
                error_list.append('sequence out of order')

        # Compute time stamp spread
        print('seq:', seq)
        for i, camera in enumerate(camera_stamps):
            stamp = camera_stamps[camera]
            print(' camera:', camera)
            print('  stamp.secs:', stamp.secs)
            print('  stamp.nsecs:', stamp.nsecs)
            if i == 0:
                stamp_max = stamp 
                stamp_min = stamp
            else:
                stamp_max = stamp if stamp.to_sec() > stamp_max.to_sec() else stamp_max 
                stamp_mim = stamp if stamp.to_sec() < stamp_min.to_sec() else stamp_min 
        print()

        stamp_max_sec = stamp_max.to_sec()
        stamp_min_sec = stamp_min.to_sec()
        stamp_spread = stamp_max_sec - stamp_min_sec
        self.max_stamp_spread = max([self.max_stamp_spread, stamp_spread])

        # Check to see if time stamp spread in within range
        if stamp_spread > self.max_allowed_stamp_spread:
            cur_ok = False
            error_list.append('maximum allowed stamp spread exceeded')

        # Record first occurance of failed time stamp.
        if not cur_ok and self.all_ok:
            self.all_ok = False 
            self.seq_fail = seq
            self.error_msg = ', '.join(error_list)

        self.last_checked_seq = seq

        # Create watchdog message and publish
        watchdog_msg = TimeStampWatchDog()
        watchdog_msg.seq = seq
        watchdog_msg.seq_fail = self.seq_fail
        watchdog_msg.all_ok = self.all_ok
        watchdog_msg.cur_ok = cur_ok
        watchdog_msg.max_spread = self.max_stamp_spread
        watchdog_msg.cur_spread = stamp_spread
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

# -----------------------------------------------------------------------------
if __name__ == '__main__':

    node = TimeStampWatchdog()
    node.run()
