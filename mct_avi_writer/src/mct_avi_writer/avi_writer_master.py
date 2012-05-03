#!/usr/bin/env python
from __future__ import print_function
import roslib
roslib.load_manifest('mct_avi_writer')
import rospy

from mct_msg_and_srv.srv import CommandString 

def avi_writer_master_proxy(cmd):
    srv = '/avi_writer_master'
    proxy = rospy.ServiceProxy(srv,CommandString)
    try:
        resp = proxy(cmd)
        flag = resp.flag
        message = resp.message
    except rospy.ServiceException, e:
        flag = False
        message = str(e)
    return flag, message

def start_avi_writers():
    avi_writer_master_proxy('start')

def stop_avi_writers():
    avi_writer_master_proxy('stop')


# -----------------------------------------------------------------------
if __name__ == '__main__':
    """ Testing  """
    import sys
    cmd = sys.argv[1]
    if cmd == 'start':
        start_avi_writers()
    elif cmd == 'stop':
        stop_avi_writers()
    else:
        print('Error: unknown command {0}'.format(cmd))

