#!/usr/bin/env python
from __future__ import print_function
import roslib
roslib.load_manifest('mct_frame_drop_corrector')
import rospy
import mct_introspection

from std_srvs.srv import Empty
from mct_msg_and_srv.srv import FrameDropInfo

def info(namespace):
    if 'frame_drop_info' in namespace:
        service_name = namespace
    else:
        service_name = '{0}/frame_drop_info'.format(namespace)
    proxy = rospy.ServiceProxy(service_name, FrameDropInfo)
    resp = proxy()
    return list(resp.seq_list)

def reset(namespace):
    if 'frame_drop_reset' in namespace:
        service_name = namespace
    else:
        service_name = '{0}/frame_drop_reset'.format(namespace)
    proxy = rospy.ServiceProxy(service_name, Empty)
    proxy()

def reset_all():
    service_name_list = mct_introspection.get_services()
    [reset(x) for x in service_name_list if 'frame_drop_reset' in x]

# -----------------------------------------------------------------------------
if __name__ == '__main__':
    """
    Testing
    """
    import sys
    if len(sys.argv) == 2:
        service = sys.argv[1]
    else:
        namespace = sys.argv[1]
        service = sys.argv[2]

    if service == 'info':
        val = info(namespace)
        print(val)
    elif service == 'reset':
        reset(namespace)
    elif service == 'reset_all':
        reset_all()
    else:
        raise ValueError, 'unknown service: {0}'.format(service)

