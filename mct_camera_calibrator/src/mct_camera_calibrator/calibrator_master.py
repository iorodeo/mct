from __future__ import print_function
import roslib 
roslib.load_manifest('mct_camera_calibrator')
import rospy

from mct_msg_and_srv.srv import CameraCalibratorCmd

def cameracalibrator_control(command,camera,image,size,square):
    """
    Wrapper for camera calbrator service proxy.
    """
    rospy.wait_for_service('cameracalibrator_control')
    proxy = rospy.ServiceProxy('cameracalibrator_control',CameraCalibratorCmd)
    try:
        response = proxy(command,camera,image,size,square)
    except rospy.ServiceException, e:
        print('ERROR: service request failed')
        response = None
    return response


def start(camera, image, size, square):
    """
    Start camera calibrator given the camera, image, target size (e.g. '8x6'), and
    target sqaure size in m (e.g. '0.254'). 
    """
    cameracalibrator_control('start', camera, image, size, square)

def stop():
    """
    Stop the currently running camera calibrator.
    """
    cameracalibrator_control('stop', '', '', '', '')
