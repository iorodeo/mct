#!/usr/bin/env python
from __future__ import print_function
import roslib
roslib.load_manifest('mct_image_stitcher')
import rospy

from mct_msg_and_srv.srv import CommandString 


def image_stitcher_master_proxy(cmd):
    srv = '/image_stitcher_master'
    proxy = rospy.ServiceProxy(srv,CommandString)
    try:
        resp = proxy(cmd)
        flag = resp.flag
        message = resp.message
    except rospy.ServiceException, e:
        flag = False
        message = str(e)
    return flag, message

def start_image_stitchers():
    image_stitcher_master_proxy('start')

def stop_image_stitchers():
    image_stitcher_master_proxy('stop')


# -----------------------------------------------------------------------
if __name__ == '__main__':
    """ Testing  """
    import sys
    cmd = sys.argv[1]
    if cmd == 'start':
        start_image_stitchers()
    elif cmd == 'stop':
        stop_image_stitchers()
    else:
        print('Error: unknown command {0}'.format(cmd))

