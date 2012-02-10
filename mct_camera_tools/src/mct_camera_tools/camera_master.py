from __future__ import print_function
import roslib
roslib.load_manifest('mct_camera_tools')
import rospy

from mct_msg_and_srv.srv import CommandString 
from mct_msg_and_srv.srv import SetCameraLaunchParam
from mct_msg_and_srv.srv import GetCameraLaunchParam


def camera_master_srv(cmd):
    srv_name = 'camera_master'
    rospy.wait_for_service(srv_name)
    proxy = rospy.ServiceProxy(srv_name,CommandString)
    response = None 
    try:
        response = proxy(cmd)
    except rospy.ServiceException, e:
        print('ERROR: service request failed, {0}'.format(str(e)))
    return response

def camera_master_set_param_srv(frame_rate, trigger):
    srv_name = 'camera_master_set_param'
    rospy.wait_for_service(srv_name)
    proxy = rospy.ServiceProxy(srv_name,SetCameraLaunchParam)
    response = None
    try:
        response = proxy(frame_rate, trigger)
    except rospy.ServiceException, e:
        print('ERROR: service request failed, {0}'.format(str(e)))
    return response

def camera_master_get_param_srv():
    srv_name = 'camera_master_get_param'
    rospy.wait_for_service(srv_name)
    proxy = rospy.ServiceProxy(srv_name,GetCameraLaunchParam)
    try:
        response = proxy()
    except rospy.ServiceException, e:
        print('ERROR: service request failed, {0}'.format(str(e)))
        response = None
    return response 

def start_cameras():
    """
    Starts the camera nodes using the settings in the camera_assignment.yaml file.

    Cameras are set to use internal triggering.
    """
    return camera_master_srv('start')

def stop_cameras():
    """
    Stops the camera nodes.
    """
    return camera_master_srv('stop')

def set_camera_launch_param(frame_rate='default', trigger=False):
    """
    Sets the camera launch parameters.

    frame_rate = string indicating frame rate setting
    trigger = hardware trigger True/False
    """
    camera_master_set_param_srv(frame_rate, trigger)

def get_camera_launch_param():
    """
    Get the current camera launch parameters from the camera master.
    """
    return camera_master_get_param_srv()



