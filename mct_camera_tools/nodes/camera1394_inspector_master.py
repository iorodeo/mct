#!/usr/bin/env python
import roslib
roslib.load_manifest('mct_camera_tools')
import rospy
import os
import os.path
import tempfile

class Camera_Inspector_Master(object):

    def __init__(self):
        self.machine_file = '/home/albert/ros/multi_cam_tracker_config/machine/mct.machine'
        tmp_dir = tempfile.gettempdir()
        file_path, file_name = os.path.split(__file__)
        self.launch_template = os.path.join(file_path, 'templates')
        self.launch_file = os.path.join(tmp_dir,'inspector_nodes.launch')
        self.launch_pid = None
        rospy.init_node('camera_inspector_master')
        self.launch_inspector_nodes()

    def run(self):
        rospy.spin()

    def create_launch_file(self):
        pass

    def delete_launch_file(self):
        pass

    def launch_inspector_nodes(self):
        pass

    def kill_inspector_nodes(self):
        pass

    def clean_up(self):
        pass

# -----------------------------------------------------------------------------
if __name__ == '__main__':

    node = Camera_Inspector_Master()
    node.run()
