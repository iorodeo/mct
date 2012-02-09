from __future__ import print_function
import roslib
roslib.load_manifest('mct_camera_tools')
import rospy
from mct_msg_and_srv.srv import CommandString 

def inspector_camera_srv(cmd):
    rospy.wait_for_service('camera_master')
    proxy = rospy.ServiceProxy('camera_master',CommandString)
    response = None 
    try:
        response = proxy(cmd)
    except rospy.ServiceException, e:
        print('ERROR: service request failed, {0}'.format(str(e)))
    return response


def start_cameras():
    """
    Starts the camera nodes using the settings in the camera_assignment.yaml file.

    Cameras are set to use internal triggering.
    """
    return inspector_camera_srv('start')

def start_cameras_w_trigger():
    """
    Starts the camera nodes using the settings in the camera_assignment.yaml file.

    Cameras are set to use external hardware triggers.
    """
    return inspector_camera_srv('start')

def stop_cameras():
    """
    Stops the camera nodes.
    """
    return inspector_camera_srv('stop')

