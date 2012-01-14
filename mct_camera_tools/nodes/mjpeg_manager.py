#!/usr/bin/env python
import roslib
roslib.load_manifest('mct_camera_tools')
import rospy

class MJPEG_Manager(object):

    def __init__(self):

        self.mjpeg_started = False
        rospy.on_shutdown(self.clean_up)
        rospy.init_node('mjpeg_manager')

    def run(self):
        rospy.spin()

    def clean_up(self):
        pass

# -----------------------------------------------------------------------------
if __name__ == '__main__':

    node = MJPEG_Manager()
    node.run()
