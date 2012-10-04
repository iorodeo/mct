#!/usr/bin/env python
from __future__ import print_function
import roslib
roslib.load_manifest('mct_frame_drop_corrector')
import rospy

from std_srvs.srv import Empty
from mct_msg_and_srv.srv import FrameDropInfo

def info(namespace):
    service_name = '{0}/frame_drop_info'.format(namespace)
    proxy = rospy.ServiceProxy(service_name, FrameDropInfo)
    resp = proxy()
    return list(resp.seq_list)

def reset(namespace):
    service_name = '{0}/frame_drop_reset'.format(namespace)
    proxy = rospy.ServiceProxy(service_name, Empty)
    proxy()

# -----------------------------------------------------------------------------
if __name__ == '__main__':
    """
    Testing
    """
    import sys
    namespace = sys.argv[1]
    service = sys.argv[2]
    if service == 'frame_drop_info':
        val = info(namespace)
        print(val)
    elif service == 'frame_drop_reset':
        reset(namespace)
    else:
        raise ValueError, 'unknown service: {0}'.format(service)

