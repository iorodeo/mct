from __future__ import print_function
import roslib
roslib.load_manifest('mct_homography')
import rospy

# Services
from mct_msg_and_srv.srv import GetMatrix
from mct_msg_and_srv.srv import GetBool
from mct_msg_and_srv.srv import GetFlagAndMessage


def start(node_name):
    service_name = '{0}/start'.format(node_name)
    proxy = rospy.ServiceProxy(service_name, GetFlagAndMessage)
    resp = proxy()
    return resp.flag, resp.message


def get_matrix(node_name):
    service_name = '{0}/get_matrix'.format(node_name)
    proxy = rospy.ServiceProxy(service_name, GetMatrix)
    resp = proxy()
    return resp.num_row, resp.num_col, resp.data

def is_calibrated(node_name):
    service_name = '{0}/is_calibrated'.format(node_name)
    proxy = rospy.ServiceProxy(service_name, GetBool)
    resp = proxy()
    return resp.value 

# -----------------------------------------------------------------------------
if __name__ == '__main__':

    import sys

    node_name = sys.argv[1]
    service = sys.argv[2]

    if service == 'start':
        val = start(node_name)
        print(val)
    elif service == 'get_matrix':
        val = get_matrix(node_name)
        print(val)
    elif service == 'is_calibrated':
        val = is_calibrated(node_name)
        print(val)


