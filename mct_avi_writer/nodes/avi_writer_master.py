#!/usr/bin/env python
from __future__ import print_function
import roslib
roslib.load_manifest('mct_avi_writer')
import rospy

import os
import os.path
import tempfile
import subprocess

from mct_xml_tools import launch

# Services
from mct_msg_and_srv.srv import CommandString 
from mct_msg_and_srv.srv import CommandStringResponse

class AVI_Writer_Master(object):

    def __init__(self):
        self.tmp_dir = tempfile.gettempdir()
        self.launch_file = os.path.join(self.tmp_dir,'avi_writer.launch')
        self.avi_writer_popen = None

        rospy.on_shutdown(self.clean_up)
        rospy.init_node('avi_writer_master')

        self.camera_srv = rospy.Service(
                'avi_writer_master',
                CommandString,
                self.handle_avi_writer_srv,
                )

    def handle_avi_writer_srv(self,req):
        """
        Handles requests to launch/kill the avi writer nodes.
        """
        command = req.command.lower()
        response = True
        message = ''
        if command == 'start':
            if self.avi_writer_popen is None:
                self.launch_avi_writers()
            else:
                response = False
                message = 'avi writers already running'
        elif command == 'stop':
            if self.avi_writer_popen is not None:
                self.kill_avi_writers()
            else:
                response = False
                message = 'avi writers not running'
        return CommandStringResponse(response,message)

    def launch_avi_writers(self):
        """
        Launches the avi writer nodes.
        """
        if self.avi_writer_popen is None:
            launch.create_avi_writer_launch(self.launch_file)
            self.avi_writer_popen = subprocess.Popen(['roslaunch',self.launch_file])

    def kill_avi_writers(self):
        """
        Kills the avi writer nodes.
        """
        if self.avi_writer_popen is not None:
            self.avi_writer_popen.send_signal(subprocess.signal.SIGINT)
            self.avi_writer_popen = None
            try:
                os.remove(self.launch_file)
            except OSError, e:
                rospy.logwarn('Error removing avi writers launch file: {0}'.format(str(e)))

    def run(self):
        rospy.spin()

    def clean_up(self):
        self.kill_avi_writers()

# -----------------------------------------------------------------------------
if __name__ == '__main__':

    node = AVI_Writer_Master()
    node.run()
