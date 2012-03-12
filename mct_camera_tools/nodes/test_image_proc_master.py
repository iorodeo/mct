from __future__ import print_function
import roslib
roslib.load_manifest('mct_camera_tools')
import rospy
import sys
from mct_camera_tools import image_proc_master

if __name__ == '__main__':

    cmd = sys.argv[1]
    cmd = cmd.lower()

    if cmd == 'start':
        resp = image_proc_master.start_image_proc()
        print(resp)
    elif cmd == 'stop':
        resp = image_proc_master.stop_image_proc()
        print(resp)
    else:
        print('unknown command {0}'.format(cmd))
