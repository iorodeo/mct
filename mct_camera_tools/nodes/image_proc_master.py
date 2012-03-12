#!/usr/bin/env python
from __future__ import print_function
import roslib
roslib.load_manifest('mct_camera_tools')
import rospy

import os
import os.path
import tempfile
import subprocess

from mct_xml_tools import launch

# Services
from mct_msg_and_srv.srv import CommandString 
from mct_msg_and_srv.srv import CommandStringResponse


class Image_Proc_Master(object):
    """
    Image proc master node. Provides service which launches/kills image_proc nodes for
    every camera with a calibratoin.
    """

    def __init__(self):
        self.tmp_dir = tempfile.gettempdir()
        self.launch_file = os.path.join(self.tmp_dir,'image_proc.launch')
        self.image_proc_popen = None

        rospy.on_shutdown(self.clean_up)
        rospy.init_node('image_proc_master')

        self.camera_srv = rospy.Service(
                'image_proc_master',
                CommandString,
                self.handle_image_proc_srv,
                )

    def handle_image_proc_srv(self,req):
        """
        Handles requests to lauch/kill the image proc nodes.
        """
        command = req.command.lower()
        response = True
        message = ''
        if command == 'start':
            if self.image_proc_popen is None:
                self.launch_image_proc_nodes()
            else:
                response = False
                message = 'image proc nodes already running'

        elif command == 'stop':
            if self.image_proc_popen is not None:
                self.kill_image_proc_nodes()
            else:
                response = False
                message = 'image proc nodes not running'
        else:
            response = False
            message = 'uknown command: {0}'.format(command)
        return CommandStringResponse(response,message)


    def launch_image_proc_nodes(self):
        """
        Launches the image_proc nodes.
        """
        if self.image_proc_popen is None:
            launch.create_image_proc_launch(self.launch_file)
            self.image_proc_popen = subprocess.Popen(['roslaunch',self.launch_file])

    def kill_image_proc_nodes(self):
        """
        Kills the image_proc nodes.
        """
        if self.image_proc_popen is not None:
            self.image_proc_popen.send_signal(subprocess.signal.SIGINT)
            self.image_proc_popen = None
            try:
                os.remove(self.launch_file)
            except OSError, e:
                rospy.logwarn('Error removing image_proc launch file: {0}'.format(str(e)))

    def clean_up(self):
        self.kill_image_proc_nodes()

    def run(self):
        rospy.spin()

# -----------------------------------------------------------------------------
if __name__ == '__main__':
    node = Image_Proc_Master()
    node.run()
