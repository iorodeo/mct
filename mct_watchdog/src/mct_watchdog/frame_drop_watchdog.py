#!/usr/bin/env python
from __future__ import print_function
import roslib
roslib.load_manifest('mct_watchdog')
import rospy

from std_srvs.srv import Empty

def reset():
    proxy = rospy.ServiceProxy('frame_drop_watchdog_reset',Empty)
    proxy()

# -----------------------------------------------------------------------------
if __name__ == '__main__':
    reset()


