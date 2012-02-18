from __future__ import print_function
import roslib 
roslib.load_manifest('mct_camera_calibrator')
import rospy

from mct_msg_and_srv.srv import GetBool
from mct_msg_and_srv.srv import GetString

def good_enough(calibrator):
    """
    Wraper for the good_enough service provided by the cameracalibrator nodes.  
    Given a the camera calibrator, e.g. '/mct_master/camera_1/camera/camera_calibrator',  
    returns boolean value (True or False) indicating whether or not the camera data 
    collected good enough to calculate the camera calibration. 
    """
    srv_name = '{0}/good_enough'.format(str(calibrator))
    rospy.wait_for_service(srv_name)
    proxy = rospy.ServiceProxy(srv_name,GetBool)
    try:
        response = proxy()
        value = response.value
    except rospy.ServiceException, e:
        rospy.logerr('service request failed: {0}'.format(str(e)))
        value = None
    return value

def calibrate(calibrator):
    """
    Wrapper for the calibrate service provided by the cameracalibrator nodes.  
    Given a the camera calibrator, e.g. '/mct_master/camera_1/camera/camera_calibrator',  
    this function requests that the node calculate the camera calibration given the data
    collected so far. Returns True if a calibration can be calculated and False otherwise.
    """
    srv_name = '{0}/calibrate'.format(str(calibrator))
    rospy.wait_for_service(srv_name)
    proxy = rospy.ServiceProxy(srv_name,GetBool)
    try:
        response = proxy()
        value = response.value
    except rospy.ServiceException, e:
        rospy.logerr('service request failed: {0}'.format(str(e)))
        value = None
    return value


def get_calibration(calibrator):
    """
    Wrapper for the get_calibration service proviced by the
    cameracalibrator nodes.  Given a camera calibrator, e.g.,
    '/mct_master/camera_1/camera/camera_calibrator', returns the camera
    calibration or an empty string if a calibration has not yet been calculated. 
    """
    srv_name = '{0}/get_calibration'.format(str(calibrator))
    rospy.wait_for_service(srv_name)
    proxy = rospy.ServiceProxy(srv_name,GetString)
    try:
        response = proxy()
        data = response.data
    except rospy.ServiceException, e:
        rospy.logerr('service request failed: {0}'.format(str(e)))
        data = None
    return data 

