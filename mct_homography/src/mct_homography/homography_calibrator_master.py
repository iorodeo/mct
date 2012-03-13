from __future__ import print_function
import roslib
roslib.load_manifest('mct_homography')
import rospy

# Services
from mct_msg_and_srv.srv import CommandString 

def homography_calibrator_master(cmd):
    proxy = rospy.ServiceProxy('/homography_calibrator_master', CommandString)
    resp = proxy(cmd)
    return resp.flag, resp.message

def start():
    return homography_calibrator_master('start')

def stop():
    return homography_calibrator_master('stop')

# -----------------------------------------------------------------------------
if __name__ == '__main__':

    import sys

    if sys.argv[1] == 'start':
        start()
    else:
        stop()




