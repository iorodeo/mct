from __future__ import print_function
import roslib
roslib.load_manifest('mct_blob_finder')
import rospy

# Services
from mct_msg_and_srv.srv import BlobFinderParam

def set_threshold(threshold):
    """
    Sets the blob finder threshold parameter.
    """
    rospy.wait_for_service('blob_finder_param')
    proxy = rospy.ServiceProxy('blob_finder_param',BlobFinderParam)
    resp = proxy(threshold)
    return resp.status, resp.message

# -----------------------------------------------------------------------------
if __name__ == "__main__":

    status, message = set_threshold(150)
    print(status)
    print(message)

