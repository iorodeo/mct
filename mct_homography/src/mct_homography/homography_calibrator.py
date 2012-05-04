from __future__ import print_function
import roslib
roslib.load_manifest('mct_homography')
import rospy

# Services
from mct_msg_and_srv.srv import GetMatrix
from mct_msg_and_srv.srv import GetBool
from mct_msg_and_srv.srv import GetFlagAndMessage


def start(namespace):
    service_name = '{0}/start'.format(namespace)
    proxy = rospy.ServiceProxy(service_name, GetFlagAndMessage)
    resp = proxy()
    return resp.flag, resp.message


def get_matrix(namespace):
    service_name = '{0}/get_matrix'.format(namespace)
    proxy = rospy.ServiceProxy(service_name, GetMatrix)
    resp = proxy()
    return resp.num_row, resp.num_col, resp.data

def is_calibrated(namespace):
    service_name = '{0}/is_calibrated'.format(namespace)
    proxy = rospy.ServiceProxy(service_name, GetBool)
    resp = proxy()
    return resp.value 

# -----------------------------------------------------------------------------
if __name__ == '__main__':

    import sys

    namespace = sys.argv[1]
    service = sys.argv[2]

    if service == 'start':
        val = start(namespace)
        print(val)
    elif service == 'get_matrix':
        val = get_matrix(namespace)
        print(val)
    elif service == 'is_calibrated':
        val = is_calibrated(namespace)
        print(val)


