#!/usr/bin/env python
from __future__ import print_function
import roslib
roslib.load_manifest('mct_camera_tools')
import rospy

import os
import os.path
import tempfile
import subprocess

import mct_introspection

# Services
from mct_msg_and_srv.srv import CommandString 
from mct_msg_and_srv.srv import CommandStringResponse

class Camera_Master(object):
    """
    Launches cameras found in camera_assignment.yaml file.
    """
    def __init__(self):

        # Get location of machine file
        self.camera_assignment = mct_introspection.get_camera_assignment()
        self.tmp_dir = tempfile.gettempdir()
        self.launch_popen = None

        rospy.on_shutdown(self.clean_up)
        rospy.init_node('camera1394_master')

        self.camera_srv = rospy.Service(
                'camera_nodes_control',
                CommandString,
                self.handle_cameras,
                )

    def run(self):
        pass

    def handle_camera_srv(self,request):
        request = request.lower()
        response = True 
        message = '' 
        if request == 'start':
            if self.launch_popen is None:
                self.launch_camera_nodes()
            else:
                repsonse = False
                message = 'camera nodes already started'
        elif request == 'stop':
            if self.launch_popen is not None:
                self.kill_camera_nodes()
            else:
                response = False
                message = 'camera nodes not running'
        else:
            response = False
            message = 'uknown request: {0}'.format(request)
        return CommandStringResponse(response,message)

    def launch_camera_nodes(self):
        pass

    def kill_camera_nodes(self):
        pass

    def clean_up(self):
        pass

# -----------------------------------------------------------------------------
if __name__ == '__main__':
    node = Camera_Master()
    node.run()


