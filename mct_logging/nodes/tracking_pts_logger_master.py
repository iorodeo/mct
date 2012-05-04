#!/usr/bin/env python
from __future__ import print_function
import roslib
roslib.load_manifest('mct_logging')
import rospy

import os
import os.path
import tempfile
import subprocess

from mct_xml_tools import launch

# Services
from mct_msg_and_srv.srv import CommandString 
from mct_msg_and_srv.srv import CommandStringResponse

class TrackingPtsLogger_Master(object):

    def __init__(self):
        self.tmp_dir = tempfile.gettempdir()
        self.launch_file = os.path.join(self.tmp_dir,'tracking_pts_logger.launch')
        self.logger_popen = None

        rospy.on_shutdown(self.clean_up)
        rospy.init_node('tracking_pts_logger_master')

        self.camera_srv = rospy.Service(
                'tracking_pts_logger_master',
                CommandString,
                self.handle_tracking_pts_logger_srv,
                )

    def handle_tracking_pts_logger_srv(self,req):
        """
        Handles requests to launch/kill the tracking_pts_logger nodes.
        """
        command = req.command.lower()
        response = True
        message = ''
        if command == 'start':
            if self.logger_popen is None:
                self.launch_loggers()
            else:
                response = False
                message = 'tracking pts loggers already running'
        elif command == 'stop':
            if self.logger_popen is not None:
                self.kill_loggers()
            else:
                response = False
                message = 'tracking pts loggers not running'
        return CommandStringResponse(response,message)

    def launch_loggers(self):
        """
        Launches the tracking_pts_logger nodes.
        """
        if self.logger_popen is None:
            launch.create_tracking_pts_logger_launch(self.launch_file)
            self.logger_popen = subprocess.Popen(['roslaunch',self.launch_file])

    def kill_loggers(self):
        """
        Kills the tracking_pts_logger nodes.
        """
        if self.logger_popen is not None:
            self.logger_popen.send_signal(subprocess.signal.SIGINT)
            self.logger_popen = None
            try:
                os.remove(self.launch_file)
            except OSError, e:
                rospy.logwarn('Error removing tracking_pts_logger launch file: {0}'.format(str(e)))

    def run(self):
        rospy.spin()

    def clean_up(self):
        self.kill_loggers()

# -----------------------------------------------------------------------------
if __name__ == '__main__':

    node = TrackingPtsLogger_Master()
    node.run()
