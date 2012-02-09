#!/usr/bin/env python
from __future__ import print_function
import roslib
roslib.load_manifest('mct_camera_tools')
import rospy

import os
import os.path
import tempfile
import subprocess

import mct_xml_tools
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
        self.launch_file = os.path.join(self.tmp_dir,'camera.launch')
        self.camera_assignment = None
        self.camera_popen = None

        rospy.on_shutdown(self.clean_up)
        rospy.init_node('camera1394_master')

        self.camera_srv = rospy.Service(
                'camera_master',
                CommandString,
                self.handle_camera_srv,
                )

    def run(self):
        rospy.spin()

    def handle_camera_srv(self,request):
        """
        Handes requests for camera services. Starts and stops the camera nodes.
        """
        command = request.command
        command = command.lower()
        response = True 
        message = '' 
        if command in ('start', 'start_w_trig'):
            if self.camera_popen is None:
                if command == 'start_w_trig':
                    trigger=True
                else:
                    trigger=False
                self.launch_camera_nodes(trigger=trigger)
            else:
                repsonse = False
                message = 'camera nodes already started'
        elif command == 'stop':
            if self.camera_popen is not None:
                print('stop')
                self.kill_camera_nodes()
            else:
                response = False
                message = 'camera nodes not running'
        else:
            response = False
            message = 'uknown request: {0}'.format(request)
        return CommandStringResponse(response,message)

    def launch_camera_nodes(self,trigger=True):
        """
        Creates launch and yaml files and launches camera nodes.
        """
        self.camera_assignment = mct_introspection.get_camera_assignment()
        mct_xml_tools.launch.create_camera_yaml(
                self.tmp_dir, 
                self.camera_assignment
                )
        mct_xml_tools.launch.create_camera_launch(
                self.launch_file, 
                self.camera_assignment,
                trigger=trigger
                )
        self.camera_popen = subprocess.Popen(['roslaunch',self.launch_file])

    def kill_camera_nodes(self):
        """
        Kills camera nodes and deletest launch and yaml files.
        """
        if self.camera_popen is not None:
            self.camera_popen.send_signal(subprocess.signal.SIGINT)
            self.camera_popen = None
            try:
                os.remove(self.launch_file)
            except OSError, e:
                rospy.logwarn('Error removing camera launch file: {0}'.format(str(e)))
            for k,v in self.camera_assignment.iteritems():
                try:
                    os.remove(v['yaml_file'])
                except OSError, e:
                    rospy.logwarn('Error removing camera yaml files: {0}'.format(str(e)))
            self.camera_assignment = None


    def clean_up(self):
        self.kill_camera_nodes()

# -----------------------------------------------------------------------------
if __name__ == '__main__':
    node = Camera_Master()
    node.run()


