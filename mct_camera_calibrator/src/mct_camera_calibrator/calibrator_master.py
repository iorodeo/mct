from __future__ import print_function
import roslib 
roslib.load_manifest('mct_camera_calibrator')
import rospy

from mct_msg_and_srv.srv import CameraCalibratorCmd

def cameracalibrator_master_srv(command,chessboard_size,chessboard_square):
    """
    Wrapper for camera calbrator service proxy.
    """
    rospy.wait_for_service('cameracalibrator_master')
    proxy = rospy.ServiceProxy('cameracalibrator_master',CameraCalibratorCmd)
    try:
        response = proxy(command,chessboard_size,chessboard_square)
    except rospy.ServiceException, e:
        print('ERROR: service request failed')
        response = None
    return response


def start(chessboard_size, chessboard_square):
    """
    Start camera calibrator given the camera, image, target chessboard_size (e.g. '8x6'), and
    target sqaure chessboard_size in m (e.g. '0.254'). 
    """
    cameracalibrator_master_srv('start', chessboard_size, chessboard_square)

def stop():
    """
    Stop the currently running camera calibrator.
    """
    cameracalibrator_master_srv('stop', '', '', '', '')
