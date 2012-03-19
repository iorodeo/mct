from __future__ import print_function
import roslib
roslib.load_manifest('mct_transform_2d')
import rospy

from mct_msg_and_srv.srv import GetBool
from mct_msg_and_srv.srv import GetFlagAndMessage
from mct_msg_and_srv.srv import GetTransform2d

def start(node_name):
    service_name = '{0}/start'.format(node_name)
    proxy = rospy.ServiceProxy(service_name, GetFlagAndMessage)
    resp = proxy()
    return resp.flag, resp.message

def get_transform_2d(node_name):
    service_name = '{0}/get_transform_2d'.format(node_name)
    proxy = rospy.ServiceProxy(service_name, GetTransform2d)
    resp = proxy()
    return resp.rotation, resp.translation_x, resp.translation_y

def is_calibrated(node_name):
    service_name = '{0}/is_calibrated'.format(node_name)
    proxy = rospy.ServiceProxy(service_name, GetBool)
    resp = proxy()
    return resp.value 

# -----------------------------------------------------------------------------
if __name__ == "__main__":

    import sys
    node_name, cmd = sys.argv[1], sys.argv[2]
    if cmd == 'start':
        print(start(node_name))
    elif cmd == 'is_calibrated':
        print(is_calibrated(node_name))
    elif cmd == 'get_transform_2d':
        print(get_transform_2d(node_name))

