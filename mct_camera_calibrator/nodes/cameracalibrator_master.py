#!/usr/bin/env python
from __future__ import print_function
import roslib 
roslib.load_manifest('mct_camera_calibrator')
import rospy
import subprocess

import os
import os.path
import tempfile
import subprocess

import mct_xml_tools.launch as launch
import mct_introspection

# Services
from mct_msg_and_srv.srv import CameraCalibratorCmd
from mct_msg_and_srv.srv import CameraCalibratorCmdResponse

class CameraCalibratorMaster(object):

    def __init__(self):

        self.tmp_dir = tempfile.gettempdir()
        self.launch_file = os.path.join(self.tmp_dir,'camera_calibrator.launch')
        self.calibrator_popen = None

        rospy.on_shutdown(self.clean_up)
        rospy.init_node('cameracalibrator_master')
        self.camera_srv = rospy.Service(
                'cameracalibrator_master',
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
                chessboard_size = req.chessboard_size
                chessboard_square = req.chessboard_square
                self.start_calibrator_nodes(chessboard_size,chessboard_square)
            else:
                flag = False
                message = 'calibrator node already running'
        elif command == 'stop':
            # Stop calilbration node
            if self.calibrator_popen is not None:
                self.kill_calibrator_nodes()
            else:
                flag = False
                message = 'calibrator node is not running'
        else:
            flag = False
            message = 'unknown command string {0}'.format(command)
        return CameraCalibratorCmdResponse(flag,message)

    def start_calibrator_nodes(self,chessboard_size,chessboard_square):
        """
        Start camera calibrator nodes as launch subprocess given the size of the calibration 
        chess boardi, chessboard_size = 'NxM',  and the square dimension in (m), e.g., 
        chessboard_square='0.0254'.
        """
        if self.calibrator_popen is None:
            image_topics = mct_introspection.find_camera_image_topics(transport='image_raw')
            launch.create_camera_calibrator_launch(
                    self.launch_file, 
                    image_topics, 
                    chessboard_size, 
                    chessboard_square
                    ) 
            self.calibrator_popen = subprocess.Popen(['roslaunch', self.launch_file])

    def kill_calibrator_nodes(self):
        """
        Kill camera calibrator node subprocess.
        """ 
        if self.calibrator_popen is not None:
            self.calibrator_popen.send_signal(subprocess.signal.SIGINT)
            self.calibrator_popen = None

    def run(self):
        rospy.spin()

    def clean_up(self):
        if self.calibrator_popen is not None:
            self.kill_calibrator_nodes()

# -----------------------------------------------------------------------------
if __name__ == '__main__':
    node = CameraCalibratorMaster()
    node.run()
