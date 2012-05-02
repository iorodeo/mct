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


class ThreePointTracker_Master(object):

    """
    Master controller for three point tracker nodes. Launches three point
    trackers for all cameras which belong to a tracking region in the current
    layout.
    """

    def __init__(self):
        self.tmp_dir = tempfile.gettempdir()
        self.launch_file = os.path.join(self.tmp_dir,'three_point_tracker.launch')
        self.tracker_popen = None

        rospy.on_shutdown(self.clean_up)
        rospy.init_node('three_point_tracker_master')

        self.camera_srv = rospy.Service(
                'three_point_tracker_master',
                CommandString,
                self.handle_calibrator_srv,
                )

    def handle_calibrator_srv(self,req):
        """
        Handles requests to launch/kill the three point tracker nodes
        """
        command = req.command.lower()
        response = True
        message = ''
        if command == 'start':
            if self.tracker_popen is None:
                self.launch_trackers()
            else:
                response = False
                message = 'three point trackers already running'
        elif command == 'stop':
            if self.tracker_popen is not None:
                self.kill_trackers()
            else:
                response = False
                message = 'three point trackers not running'
        return CommandStringResponse(response,message)

    def launch_trackers(self):
        """
        Launches the tracker nodes.
        """
        if self.tracker_popen is None:
            launch.create_three_point_tracker_launch(self.launch_file)
            self.tracker_popen = subprocess.Popen(['roslaunch',self.launch_file])

    def kill_trackers(self):
        """
        Kills the tracker nodes.
        """
        if self.tracker_popen is not None:
            self.tracker_popen.send_signal(subprocess.signal.SIGINT)
            self.tracker_popen = None
            try:
                os.remove(self.launch_file)
            except OSError, e:
                rospy.logwarn('Error removing three point tracker launch file: {0}'.format(str(e)))

    def run(self):
        rospy.spin()

    def clean_up(self):
        self.kill_trackers()

# -----------------------------------------------------------------------------
if __name__ == '__main__':

    node = ThreePointTracker_Master()
    node.run()
