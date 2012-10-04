#!/usr/bin/env python
from __future__ import print_function
import roslib
roslib.load_manifest('mct_frame_drop_corrector')
import rospy

# Services
from mct_msg_and_srv.srv import CommandString 

def frame_drop_corrector_master_proxy(cmd):
    srv = '/frame_drop_corrector_master'
    proxy = rospy.ServiceProxy(srv,CommandString)
    try:
        resp = proxy(cmd)
        flag = resp.flag
        message = resp.message
    except rospy.ServiceException, e:
        flag = False
        message = str(e)
    return flag, message

def start():
    frame_drop_corrector_master_proxy('start')

def stop():
    frame_drop_corrector_master_proxy('stop')

# -----------------------------------------------------------------------
if __name__ == '__main__':

    """ Testing  """

    import sys
    cmd = sys.argv[1]
    if cmd == 'start':
        start()
    elif cmd == 'stop':
        stop()
    else:
        print('Error: unknown command {0}'.format(cmd))

    

