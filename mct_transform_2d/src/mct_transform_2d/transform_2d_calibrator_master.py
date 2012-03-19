from __future__ import print_function
import roslib
roslib.load_manifest('mct_transform_2d')
import rospy

from mct_msg_and_srv.srv import CommandString 

def transform_2d_calibrator_master_srv(cmd):
    proxy = rospy.ServiceProxy('/transform_2d_calibrator_master', CommandString)
    resp = proxy(cmd)
    return resp.flag, resp.message

def start():
    return transform_2d_calibrator_master_srv('start')

def stop():
    return transform_2d_calibrator_master_srv('stop')

# -----------------------------------------------------------------------------
if __name__ == '__main__':

    import sys
    cmd = sys.argv[1]
    if cmd == 'start':
        print(start())
    elif cmd == 'stop':
        print(stop())
