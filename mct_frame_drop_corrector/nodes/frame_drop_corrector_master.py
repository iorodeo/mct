#!/usr/bin/env python
from __future__ import print_function
import roslib
roslib.load_manifest('mct_frame_drop_corrector')
import rospy

import os
import os.path
import sys
import tempfile
import subprocess

from mct_xml_tools import launch
from mct_utilities import file_tools

# Services
from mct_msg_and_srv.srv import CommandString 
from mct_msg_and_srv.srv import CommandStringResponse

class Frame_Drop_Corrector_Master(object):

    def __init__(self, framerate_mode):
        self.tmp_dir = tempfile.gettempdir()
        self.launch_file = os.path.join(self.tmp_dir,'frame_drop_corrector.launch')
        self.corrector_popen = None
        framerate_dict = file_tools.read_frame_rates()
        self.framerate = float(framerate_dict[framerate_mode])

        rospy.on_shutdown(self.clean_up)
        rospy.init_node('frame_drop_corrector_master')

        self.camera_srv = rospy.Service(
                'frame_drop_corrector_master',
                CommandString,
                self.handle_corrector_srv,
                )

    def handle_corrector_srv(self,req):
        """
        Handles requests to launch/kill the frame drop corrector nodes.
        """
        command = req.command.lower()
        response = True
        message = ''
        if command == 'start':
            if self.corrector_popen is None:
                self.launch_correctors()
            else:
                response = False
                message = 'frame drop correctors already running'
        elif command == 'stop':
            if self.corrector_popen is not None:
                self.kill_correctors()
            else:
                response = False
                message = 'frame drop correctors not running'
        return CommandStringResponse(response,message)

    def launch_correctors(self):
        """
        Launches the image stitcher  nodes.
        """
        if self.corrector_popen is None:
            launch.create_frame_drop_corrector_launch(self.launch_file,self.framerate)
            self.corrector_popen = subprocess.Popen(['roslaunch',self.launch_file])

    def kill_correctors(self):
        """
        Kills the image stitchers nodes.
        """
        if self.corrector_popen is not None:
            self.corrector_popen.send_signal(subprocess.signal.SIGINT)
            self.corrector_popen = None
            try:
                os.remove(self.launch_file)
            except OSError, e:
                rospy.logwarn('Error removing frame drop corrector launch file: {0}'.format(str(e)))

    def run(self):
        rospy.spin()

    def clean_up(self):
        self.kill_correctors()

# -----------------------------------------------------------------------------
if __name__ == '__main__':

    framerate_mode = sys.argv[1]
    node = Frame_Drop_Corrector_Master(framerate_mode)
    node.run()
