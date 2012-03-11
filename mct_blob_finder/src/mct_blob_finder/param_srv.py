from __future__ import print_function
import roslib
roslib.load_manifest('mct_blob_finder')
import rospy

# Services
from mct_msg_and_srv.srv import BlobFinderSetParam
from mct_msg_and_srv.srv import BlobFinderGetParam

def set_param(node_name, threshold, filter_by_area, min_area, max_area):
    """
    Sets the blob finder parameters.
    """
    service = '{0}/set_param'.format(node_name)
    rospy.wait_for_service(service)
    proxy = rospy.ServiceProxy(service,BlobFinderSetParam)
    resp = proxy(threshold, filter_by_area, min_area, max_area)
    return resp.status, resp.message

def get_param(node_name):
    """
    Gets the blob finders parametes
    """
    service = '{0}/get_param'.format(node_name)
    rospy.wait_for_service(service)
    proxy = rospy.ServiceProxy(service,BlobFinderGetParam)
    resp = proxy()
    return resp.threshold, resp.filter_by_area, resp.min_area, resp.max_area

# -----------------------------------------------------------------------------
if __name__ == "__main__":

    threshold = 150
    filter_by_area = True
    min_area = 0
    max_area = 100

    status, message = set_param('/blob_finder', threshold,filter_by_area,min_area,max_area)
    print(status)
    print(message)
    print()

    threshold, filter_by_area, min_area, max_area = get_param('/blob_finder')
    print('threshold', threshold)
    print('filter_by_area', filter_by_area)
    print('min_area', min_area)
    print('max_area', max_area)



