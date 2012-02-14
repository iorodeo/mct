from __future__ import print_function
import roslib 
roslib.load_manifest('mct_camera_calibrator')
import rospy

from mct_camera_calibrator import calibrator_master

if __name__ == '__main__':

    import sys

    cmd = sys.argv[1].lower()
    if not cmd in ('start','stop'):
        print('ERROR: command must be start or stop')
        sys.exit(0)

    if cmd == 'start':
        size = '8x6'
        square = '0.0254'
        calibrator_master.start(size,square)
    else:
        calibrator_master.stop()
        
