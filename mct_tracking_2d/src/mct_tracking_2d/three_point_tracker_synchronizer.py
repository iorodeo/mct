#!/usr/bin/env python
from __future__ import print_function
import roslib
roslib.load_manifest('mct_tracking_2d')
import rospy

from std_srvs.srv import Empty

def reset(namespace):
    if namespace is None:
        service_name = 'reset_tracker_synchronizer'
    elif 'reset_tracker_synchronizer' in namespace:
        service_name = namespace
    else:
        service_name = '{0}/reset_tracker_synchronizer'.format(namespace)
    proxy = rospy.ServiceProxy(service_name,Empty)
    proxy()

# -----------------------------------------------------------------------------
if __name__ == '__main__':

    import sys
    try:
        namespace = sys.argv[1]
    except IndexError:
        namespace = None
    reset(namespace)
