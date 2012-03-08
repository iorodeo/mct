from __future__ import print_function
import roslib
roslib.load_manifest('mct_blob_finder')
import rospy

# Services
from mct_msg_and_srv.srv import BlobFinderSetParam
from mct_msg_and_srv.srv import BlobFinderGetParam

def set_param(threshold, filter_by_area, min_area, max_area, service='/blob_finder_set_param'):
    """
    Sets the blob finder parameters.
    """
    rospy.wait_for_service(service)
    proxy = rospy.ServiceProxy(service,BlobFinderSetParam)
    resp = proxy(threshold, filter_by_area, min_area, max_area)
    return resp.status, resp.message

def get_param(service='/blob_finder_get_param'):
    """
    Gets the blob finders parametes
    """
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

    status, message = set_param(threshold,filter_by_area,min_area,max_area)
    print(status)
    print(message)
    print()

    threshold, filter_by_area, min_area, max_area = get_param()
    print('threshold', threshold)
    print('filter_by_area', filter_by_area)
    print('min_area', min_area)
    print('max_area', max_area)



