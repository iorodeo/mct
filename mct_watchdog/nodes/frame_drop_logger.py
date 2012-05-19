#!/usr/bin/env python
from __future__ import print_function
import roslib
roslib.load_manifest('mct_watchdog')
import rospy
import threading
import os

from mct_msg_and_srv.msg import TimeStampWatchDog
from std_srvs.srv import Empty

class FrameDropLogger(object):

    def __init__(self):
        self.error_count = 0
        self.timestamp_sub = rospy.Subscriber(
                'time_stamp_watchdog', 
                TimeStampWatchDog,
                self.handle_watchdog_msg,
                )
        self.filename = os.path.join(
                os.environ['HOME'],
                'time_stamp_error_log.txt'
                )
        self.fid = open(self.filename,'w')
        self.watchdog_reset = rospy.ServiceProxy(
                'time_stamp_watchdog_reset',
                Empty,
                )

        rospy.init_node('frame_drop_test')
        rospy.on_shutdown(self.on_shutdown)

    def handle_watchdog_msg(self,data):
        if not data.ok:
            self.error_count += 1
            self.fid.write('ERROR {0}\n'.format(self.error_count))
            self.fid.write(str(data))
            self.fid.write('\n\n')
            self.fid.flush()
            self.watchdog_reset()

    def on_shutdown(self):
        self.fid.close()

    def run(self):
        rospy.spin()

# -----------------------------------------------------------------------------
if __name__ == '__main__':

    node = FrameDropLogger()
    node.run()
