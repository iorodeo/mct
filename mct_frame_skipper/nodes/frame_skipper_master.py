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

class Frame_Skipper_Master(object):

    def __init__(self):
        self.tmp_dir = tempfile.gettempdir()
        self.launch_file = os.path.join(self.tmp_dir,'frame_skipper.launch')
        self.skipper_popen = None

        rospy.on_shutdown(self.clean_up)
        rospy.init_node('frame_skipper_master')

        self.camera_srv = rospy.Service(
                'frame_skipper_master',
                CommandString,
                self.handle_skipper_srv,
                )

    def handle_skipper_srv(self,req):
        """
        Handles requests to launch/kill the image stitcher nodes.
        """
        command = req.command.lower()
        response = True
        message = ''
        if command == 'start':
            if self.skipper_popen is None:
                self.launch_frame_skippers()
            else:
                response = False
                message = 'frame skippers already running'
        elif command == 'stop':
            if self.skipper_popen is not None:
                self.kill_frame_skippers()
            else:
                response = False
                message = 'frame skippers not running'
        return CommandStringResponse(response,message)

    def launch_frame_skippers(self):
        """
        Launches the image stitcher  nodes.
        """
        if self.skipper_popen is None:
            launch.create_image_stitcher_launch(self.launch_file)
            self.skipper_popen = subprocess.Popen(['roslaunch',self.launch_file])

    def kill_frame_skippers(self):
        """
        Kills the image stitchers nodes.
        """
        if self.skipper_popen is not None:
            self.skipper_popen.send_signal(subprocess.signal.SIGINT)
            self.skipper_popen = None
            try:
                os.remove(self.launch_file)
            except OSError, e:
                rospy.logwarn('Error removing image stitcher launch file: {0}'.format(str(e)))

    def run(self):
        rospy.spin()

    def clean_up(self):
        self.kill_frame_skippers()

# -----------------------------------------------------------------------------
if __name__ == '__main__':

    node = Frame_Skipper_Master()
    node.run()
