from __future__ import print_function
import roslib 
roslib.load_manifest('mct_camera_calibrator')
import rospy

from mct_camera_calibrator import calibrator_master
from mct_msg_and_srv.srv import CameraCalibratorCmd

#def cameracalibrator_control(command,camera,image,size,square):
#    rospy.wait_for_service('cameracalibrator_control')
#    proxy = rospy.ServiceProxy('cameracalibrator_control',CameraCalibratorCmd)
#    try:
#        response = proxy(command,camera,image,size,square)
#    except rospy.ServiceException, e:
#        print('ERROR: service request failed')
#        response = None
#    return response

# -----------------------------------------------------------------------------
if __name__ == '__main__':
    import sys

    cmd = sys.argv[1].lower()
    if not cmd in ('start','stop'):
        print('ERROR: command must be start or stop')
        sys.exit(0)

    if cmd == 'start':
        camera = '/mct_slave2/camera_10/camera'
        image = '/mct_slave2/camera_10/camera/image_raw'
        size = '8x6'
        square = '0.0254'
        calibrator_master.start(camera,image,size,square)
    else:
        calibrator_master.stop()
        
        #response = cameracalibrator_control(command,camera,image,size,square)
        #print('response: {0}'.format(response))
        
