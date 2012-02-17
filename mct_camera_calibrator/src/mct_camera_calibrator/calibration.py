from __future__ import print_function
import roslib 
roslib.load_manifest('mct_camera_calibrator')
import rospy

from mct_msg_and_srv.srv import GetBool
from mct_msg_and_srv.srv import GetString

def good_enough(camera):
    srv_name = '{0}/good_enough'.format(str(camera))
    rospy.wait_for_service(srv_name)
    proxy = rospy.ServiceProxy(srv_name,GetBool)
    try:
        value = proxy()
    except rospy.ServiceException, e:
        rospy.logerr('service request failed: {0}'.format(str(e)))
        value = None
    return value

def get_calibration(camera):
    srv_name = '{0}/get_calibration'.format(str(camera))
    rospy.wait_for_service(srv_name)
    proxy = rospy.ServiceProxy(srv_name,GetString)
    try:
        value = proxy()
    except rospy.ServiceException, e:
        rospy.logerr('service request failed: {0}'.format(str(e)))
        value = None
    return value

