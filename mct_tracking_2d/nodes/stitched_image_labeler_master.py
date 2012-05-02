#!/usr/bin/env python
from __future__ import print_function
import roslib
roslib.load_manifest('mct_tracking_2d')
import rospy

import os
import os.path
import tempfile
import subprocess

from mct_xml_tools import launch

# Services
from mct_msg_and_srv.srv import CommandString 
from mct_msg_and_srv.srv import CommandStringResponse


class Stitched_Image_Labeler_Master(object):

    """
    Master controller for the stitched image labeler nodes    
    """

    def __init__(self):
        self.tmp_dir = tempfile.gettempdir()
        self.launch_file = os.path.join(self.tmp_dir,'stitched_image_labeler.launch')
        self.labeler_popen = None

        rospy.on_shutdown(self.clean_up)
        rospy.init_node('stitched_image_labeler_master')

        self.camera_srv = rospy.Service(
                'stitched_image_labeler_master',
                CommandString,
                self.handle_labeler_srv,
                )

    def handle_labeler_srv(self,req):
        """
        Handles requests to launch/kill the stitched image labeler nodes
        """
        command = req.command.lower()
        response = True
        message = ''
        if command == 'start':
            if self.labeler_popen is None:
                self.launch_labelers()
            else:
                response = False
                message = 'stitched image labelers already running'
        elif command == 'stop':
            if self.labeler_popen is not None:
                self.kill_labelers()
            else:
                response = False
                message = 'stitched image labelers not running'
        return CommandStringResponse(response,message)

    def launch_labelers(self):
        """
        Launches the stitched image labeler nodes.
        """
        if self.labeler_popen is None:
            launch.create_stitched_image_labeler_launch(self.launch_file)
            self.labeler_popen = subprocess.Popen(['roslaunch',self.launch_file])

    def kill_labelers(self):
        """
        Kills the stitched image labeler nodes.
        """
        if self.labeler_popen is not None:
            self.labeler_popen.send_signal(subprocess.signal.SIGINT)
            self.labeler_popen = None
            try:
                os.remove(self.launch_file)
            except OSError, e:
                rospy.logwarn('Error removing stitche image labeler launch file: {0}'.format(str(e)))

    def run(self):
        rospy.spin()

    def clean_up(self):
        self.kill_labelers()

# -----------------------------------------------------------------------------
if __name__ == '__main__':

    node = Stitched_Image_Labeler_Master()
    node.run()
