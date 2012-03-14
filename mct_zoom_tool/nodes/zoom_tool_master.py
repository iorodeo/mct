#!/usr/bin/env python
from __future__ import print_function
import roslib
roslib.load_manifest('mct_zoom_tool')
import rospy

import os
import os.path
import tempfile
import subprocess

from mct_xml_tools import launch

# Services
from mct_msg_and_srv.srv import CommandString 
from mct_msg_and_srv.srv import CommandStringResponse

class Zoom_Tool_Master(object):

    def __init__(self):
        self.tmp_dir = tempfile.gettempdir()
        self.launch_file = os.path.join(self.tmp_dir,'zoom_tool.launch')
        self.zoom_tool_popen = None

        rospy.on_shutdown(self.clean_up)
        rospy.init_node('zoom_tool_master')

        self.camera_srv = rospy.Service(
                'zoom_tool_master',
                CommandString,
                self.handle_zoom_tool_srv,
                )

    def handle_zoom_tool_srv(self,req):
        """
        Handles requests to launch/kill the zoom tool nodes.
        """
        command = req.command.lower()
        response = True
        message = ''
        if command == 'start':
            if self.zoom_tool_popen is None:
                self.launch_zoom_tools()
            else:
                response = False
                message = 'zoom tools already running'
        elif command == 'stop':
            if self.zoom_tool_popen is not None:
                self.kill_zoom_tools()
            else:
                response = False
                message = 'zoom tools not running'
        return CommandStringResponse(response,message)

    def launch_zoom_tools(self):
        """
        Launches the zoom tool nodes.
        """
        if self.zoom_tool_popen is None:
            launch.create_zoom_tool_launch(self.launch_file)
            self.zoom_tool_popen = subprocess.Popen(['roslaunch',self.launch_file])

    def kill_zoom_tools(self):
        """
        Kills the zoom tool nodes.
        """
        if self.zoom_tool_popen is not None:
            self.zoom_tool_popen.send_signal(subprocess.signal.SIGINT)
            self.zoom_tool_popen = None
            try:
                os.remove(self.launch_file)
            except OSError, e:
                rospy.logwarn('Error removing zoom tool launch file: {0}'.format(str(e)))

    def run(self):
        rospy.spin()

    def clean_up(self):
        self.kill_zoom_tools()

# -----------------------------------------------------------------------------
if __name__ == '__main__':

    node = Zoom_Tool_Master()
    node.run()

