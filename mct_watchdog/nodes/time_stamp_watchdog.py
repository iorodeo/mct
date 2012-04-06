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

class TimeStampWatchdog(object):

    def __init__(self, max_seq_age=200):
        self.max_seq_age = max_seq_age
        self.lock = threading.Lock()
        self.seq_to_camera_stamps = {}
        self.ready = False
        self.most_recent_seq = None
        self.last_checked_seq = None
        self.ok = True
        rospy.init_node('time_stamp_watchdog')

        # Subscribe to camera info topics
        camera_info_topics = mct_introspection.find_camera_info_topics()
        self.camera_info_sub = {}
        for topic in camera_info_topics:
            camera = get_camera_from_topic(topic)
            handler = functools.partial(self.camera_info_handler,camera)
            self.camera_info_sub[camera] = rospy.Subscriber(topic, CameraInfo, handler)

        self.number_of_cameras = len(camera_info_topics)
        self.ready = True

    def camera_info_handler(self, camera, data):

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

        if self.ok:

            # Check for correct sequence order
            if self.last_checked_seq is not None:
                if seq != self.last_checked_seq+1:
                    self.ok = False

            for i, camera in enumerate(camera_stamps):
                stamp = camera_stamps[camera]
                if i == 0:
                    stamp_max = stamp 
                    stamp_min = stamp
                else:
                    stamp_max = stamp if stamp.to_sec() > stamp_max.to_sec() else stamp_max 
                    stamp_mim = stamp if stamp.to_sec() < stamp_min.to_sec() else stamp_min 
            stamp_max = stamp_max.to_sec()
            stamp_min = stamp_min.to_sec()
            stamp_delta = stamp_max - stamp_min

            print(stamp_max)
            print(stamp_min)
            print(stamp_delta)
            print()



        self.last_checked_seq = seq
        print('checking stamps for seq: {0}, ok = {1}'.format(seq, self.ok))


    def process_camera_stamps(self):
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
