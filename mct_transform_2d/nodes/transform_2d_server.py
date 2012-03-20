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
        self.tf_listener = tf.TransformListener()
        self.ready = True

    def run(self):
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            self.tf_listener.waitForTransform('tracking_plane_1', 'tracking_plane_4', now, rospy.Duration(10.0)) 
            transform = self.tf_listener.lookupTransform('tracking_plane_1', 'tracking_plane_4', now)
            print(transform)
            rospy.sleep(1.0)

        #rospy.spin()

# -----------------------------------------------------------------------------
if __name__ == '__main__':

    node = Transform2DServer()
    node.run()
