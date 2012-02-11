from __future__ import print_function
import roslib 
PKG = 'mct_camera_calibrator' 
roslib.load_manifest(PKG)
import rospy

from mct_msg_and_srv.srv import CameraCalibratorCmd
from mct_msg_and_srv.srv import CameraCalibratorCmdResponse

class CameraCalibratorMaster(object):

    def __init__(self):
        rospy.on_shutdown(self.clean_up)
        rospy.init_node('cameracalibrator_master')
        self.camera_srv = rospy.Service(
                'cameracalibrator_master',
                CommandString,
                self.handle_calibrator_srv,
                )

    def handle_calibrator_srv(self,req):
        flag = True
        message = ''
        command = req.command.lower()
        if req == 'start':
            # Start calibration node
            pass
        elif req == 'stop':
            # Stop calilbration node
            pass
        else:
            flag = False
            message = 'unknown command string {0}'.format(cmd)
        return CameraCalibratorCmdResponse(flag,message)

    def run(self):
        rospy.spin()

    def clean_up(self):
        pass


# -----------------------------------------------------------------------------
if __name__ == '__main__':
    node = CameraCalibratorMaster()
    node.run()
