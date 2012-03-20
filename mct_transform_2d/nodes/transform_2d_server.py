#!/usr/bin/env python
from __future__ import print_function 
import roslib
roslib.load_manifest('mct_transform_2d')
import rospy
import tf

from mct_utilities import file_tools

class Transform2DServer(object):

    def __init__(self):
        self.ready = False
        rospy.init_node('transform_2d_server')
        self.ready = True

    def run(self):

        while not rospy.is_shutdown():
            rospy.sleep(1.0)

        #rospy.spin()

# -----------------------------------------------------------------------------
if __name__ == '__main__':

    node = Transform2DServer()
    node.run()
