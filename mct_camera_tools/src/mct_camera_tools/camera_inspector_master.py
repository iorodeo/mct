from __future__ import print_function
import roslib
roslib.load_manifest('mct_camera_tools')
import rospy
from mct_msg_and_srv.srv import CommandString 

def inspector_camera_srv(cmd):
    rospy.wait_for_service('master_inspector_cameras')
    proxy = rospy.ServiceProxy('master_inspector_cameras',CommandString)
    response = None 
    try:
        response = proxy(cmd)
    except rospy.ServiceException, e:
        print('ERROR: service request failed, {0}'.format(str(e)))
    return response


def start_cameras():
    return inspector_camera_srv('start')

def stop_cameras():
    return inspector_camera_srv('stop')
