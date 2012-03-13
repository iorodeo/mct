#!/usr/bin/env python
from __future__ import print_function
import roslib
roslib.load_manifest('mct_homography')
import rospy

import os
import os.path
import tempfile
import subprocess

from mct_xml_tools import launch

# Services
from mct_msg_and_srv.srv import CommandString 
from mct_msg_and_srv.srv import CommandStringResponse


class Homography_Calibrator_Master(object):

    def __init__(self):
        self.tmp_dir = tempfile.gettempdir()
        self.launch_file = os.path.join(self.tmp_dir,'homography_calibrator.launch')
        self.calibrator_popen = None

        rospy.on_shutdown(self.clean_up)
        rospy.init_node('homography_calibrator_master')

        self.camera_srv = rospy.Service(
                'homography_calibrator_master',
                CommandString,
                self.handle_calibrator_srv,
                )

    def handle_calibrator_srv(self,req):
        """
        Handles requests to launch/kill the homography calibrator nodes.
        """
        command = req.command.lower()
        response = True
        message = ''
        if command == 'start':
            if self.calibrator_popen is None:
                self.launch_calibrators()
            else:
                response = False
                message = 'homography calibrators already running'
        elif command == 'stop':
            if self.calibrator_popen is not None:
                self.kill_calibrators()
            else:
                response = False
                message = 'homography calibrators not running'
        return CommandStringResponse(response,message)

    def launch_calibrators(self):
        """
        Launches the homography calibrator nodes.
        """
        if self.calibrator_popen is None:
            launch.create_homography_calibrator_launch(self.launch_file)
            self.calibrator_popen = subprocess.Popen(['roslaunch',self.launch_file])

    def kill_calibrators(self):
        """
        Kills the homography calibrator nodes.
        """
        if self.calibrator_popen is not None:
            self.calibrator_popen.send_signal(subprocess.signal.SIGINT)
            self.calibrator_popen = None
            try:
                os.remove(self.launch_file)
            except OSError, e:
                rospy.logwarn('Error removing homography calibraotor launch file: {0}'.format(str(e)))

    def run(self):
        rospy.spin()

    def clean_up(self):
        self.kill_calibrators()

# -----------------------------------------------------------------------------
if __name__ == '__main__':

    node = Homography_Calibrator_Master()
    node.run()
