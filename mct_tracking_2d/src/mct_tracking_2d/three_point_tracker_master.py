#!/usr/bin/env python
from __future__ import print_function
import roslib
roslib.load_manifest('mct_tracking_2d')
import rospy

from mct_msg_and_srv.srv import CommandString 

def three_point_tracker_master_proxy(cmd):
    srv = '/three_point_tracker_master'
    proxy = rospy.ServiceProxy(srv,CommandString)
    try:
        resp = proxy(cmd)
        flag = resp.flag
        message = resp.message
    except rospy.ServiceException, e:
        flag = False
        message = str(e)
    return flag, message

def start_three_point_trackers():
    three_point_tracker_master_proxy('start')

def stop_three_point_trackers():
    three_point_tracker_master_proxy('stop')


# -----------------------------------------------------------------------
if __name__ == '__main__':
    """ Testing  """
    import sys
    cmd = sys.argv[1]
    if cmd == 'start':
        start_three_point_trackers()
    elif cmd == 'stop':
        stop_three_point_trackers()
    else:
        print('Error: unknown command {0}'.format(cmd))

