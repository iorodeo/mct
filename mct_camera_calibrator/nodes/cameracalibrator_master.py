#!/usr/bin/env python
from __future__ import print_function
import roslib 
roslib.load_manifest('mct_camera_calibrator')
import rospy
import subprocess

from mct_msg_and_srv.srv import CameraCalibratorCmd
from mct_msg_and_srv.srv import CameraCalibratorCmdResponse

class CameraCalibratorMaster(object):

    def __init__(self):
        self.calibrator_popen = None
        rospy.on_shutdown(self.clean_up)
        rospy.init_node('cameracalibrator_master')
        self.camera_srv = rospy.Service(
                'cameracalibrator_control',
                CameraCalibratorCmd,
                self.handle_calibrator_srv,
                )

    def handle_calibrator_srv(self,req):
        flag = True
        message = ''
        command = req.command.lower()
        if command == 'start':
            # Start calibration node
            if self.calibrator_popen is None:
                self.start_calibrator_node(req)
            else:
                flag = False
                message = 'calibrator node already running'
        elif command == 'stop':
            # Stop calilbration node
            if self.calibrator_popen is not None:
                self.kill_calibrator_node()
            else:
                flag = False
                message = 'calibrator node is not running'
        else:
            flag = False
            message = 'unknown command string {0}'.format(command)
        return CameraCalibratorCmdResponse(flag,message)

    def start_calibrator_node(self,req):
        """
        Start camera calibrator node as subprocess.
        """
        #print('starting camera calibrator node')
        popen_list = [
                'rosrun',
                'mct_camera_calibrator',
                'cameracalibrator.py',
                '--size',
                '{0}'.format(req.size),
                '--square',
                '{0}'.format(req.square),
                'camera:={0}'.format(req.camera),
                'image:={0}'.format(req.image),
                ]
        #print('popen list: {0}'.format(popen_list))
        self.calibrator_popen = subprocess.Popen(popen_list) 

    def kill_calibrator_node(self):
        """
        Kill camera calibrator node subprocess.
        """
        #print('killing camera calibrator node')
        self.calibrator_popen.send_signal(subprocess.signal.SIGINT)
        self.calibrator_popen = None

    def run(self):
        rospy.spin()

    def clean_up(self):
        if self.calibrator_popen is not None:
            self.kill_calibrator_node()

# -----------------------------------------------------------------------------
if __name__ == '__main__':
    node = CameraCalibratorMaster()
    node.run()
