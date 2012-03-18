#!/usr/bin/env python
from __future__ import print_function
import roslib
roslib.load_manifest('mct_transform_2d')
import rospy

import os
import os.path
import tempfile
import subprocess

from mct_xml_tools import launch

# Services
from mct_msg_and_srv.srv import CommandString 
from mct_msg_and_srv.srv import CommandStringResponse


class Transform_2D_Calibrator_Master(object):

    def __init__(self):
        self.tmp_dir = tempfile.gettempdir()
        self.launch_file = os.path.join(self.tmp_dir,'transform_2d_calibrator.launch')
        self.calibrator_popen = None

        rospy.on_shutdown(self.clean_up)
        rospy.init_node('transform_2d_calibrator_master')

        self.camera_srv = rospy.Service(
                'transform_2d_calibrator_master',
                CommandString,
                self.handle_calibrator_srv,
                )

    def handle_calibrator_srv(self,req):
        """
        Handles requests to launch/kill the transform 2d calibrator nodes.
        """
        command = req.command.lower()
        response = True
        message = ''
        if command == 'start':
            if self.calibrator_popen is None:
                self.launch_calibrators()
            else:
                response = False
                message = 'transform 2d calibrators already running'
        elif command == 'stop':
            if self.calibrator_popen is not None:
                self.kill_calibrators()
            else:
                response = False
                message = 'transform 2d calibrators not running'
        return CommandStringResponse(response,message)

    def launch_calibrators(self):
        """
        Launches the transform calibrator nodes.
        """
        if self.calibrator_popen is None:
            launch.create_transform_2d_calibrator_launch(self.launch_file)
            self.calibrator_popen = subprocess.Popen(['roslaunch',self.launch_file])

    def kill_calibrators(self):
        """
        Kills the transform calibrator nodes.
        """
        if self.calibrator_popen is not None:
            self.calibrator_popen.send_signal(subprocess.signal.SIGINT)
            self.calibrator_popen = None
            try:
                os.remove(self.launch_file)
            except OSError, e:
                rospy.logwarn('Error removing transform 2d calibrator launch file: {0}'.format(str(e)))

    def run(self):
        rospy.spin()

    def clean_up(self):
        self.kill_calibrators()

# -----------------------------------------------------------------------------
if __name__ == '__main__':

    node = Transform_2D_Calibrator_Master()
    node.run()
