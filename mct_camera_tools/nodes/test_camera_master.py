from __future__ import print_function
import roslib
roslib.load_manifest('mct_camera_tools')
import rospy
import sys
from mct_camera_tools import camera_master

if __name__ == '__main__':
    cmd = sys.argv[1]
    cmd = cmd.lower()
    if cmd == 'start':
        resp = camera_master.start_cameras()
        print(resp)
    elif cmd == 'stop':
        resp = camera_master.stop_cameras()
        print(resp)
    else:
        print('unknown command {0}'.format(cmd))
