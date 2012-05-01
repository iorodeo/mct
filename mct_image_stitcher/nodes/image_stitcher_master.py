#!/usr/bin/env python
from __future__ import print_function
import roslib
roslib.load_manifest('mct_image_stitcher')
import rospy

import os
import os.path
import tempfile
import subprocess

from mct_xml_tools import launch

# Services
from mct_msg_and_srv.srv import CommandString 
from mct_msg_and_srv.srv import CommandStringResponse

class Image_Stitcher_Master(object):

    def __init__(self):
        self.tmp_dir = tempfile.gettempdir()
        self.launch_file = os.path.join(self.tmp_dir,'image_stitcher.launch')
        self.stitcher_popen = None

        rospy.on_shutdown(self.clean_up)
        rospy.init_node('image_stitcher_master')

        self.camera_srv = rospy.Service(
                'image_stitcher_master',
                CommandString,
                self.handle_stitcher_srv,
                )

    def handle_stitcher_srv(self,req):
        """
        Handles requests to launch/kill the image stitcher nodes.
        """
        command = req.command.lower()
        response = True
        message = ''
        if command == 'start':
            if self.stitcher_popen is None:
                self.launch_image_stitchers()
            else:
                response = False
                message = 'image stitchers already running'
        elif command == 'stop':
            if self.stitcher_popen is not None:
                self.kill_image_stitchers()
            else:
                response = False
                message = 'image stitchers not running'
        return CommandStringResponse(response,message)

    def launch_image_stitchers(self):
        """
        Launches the image stitcher  nodes.
        """
        if self.stitcher_popen is None:
            launch.create_image_stitcher_launch(self.launch_file)
            self.stitcher_popen = subprocess.Popen(['roslaunch',self.launch_file])

    def kill_image_stitchers(self):
        """
        Kills the image stitchers nodes.
        """
        if self.stitcher_popen is not None:
            self.stitcher_popen.send_signal(subprocess.signal.SIGINT)
            self.stitcher_popen = None
            try:
                os.remove(self.launch_file)
            except OSError, e:
                rospy.logwarn('Error removing image stitcher launch file: {0}'.format(str(e)))

    def run(self):
        rospy.spin()

    def clean_up(self):
        self.kill_image_stitchers()

# -----------------------------------------------------------------------------
if __name__ == '__main__':

    node = Image_Stitcher_Master()
    node.run()
