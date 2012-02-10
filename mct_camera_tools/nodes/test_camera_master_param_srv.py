from __future__ import print_function
import roslib
roslib.load_manifest('mct_camera_tools')
import rospy
import sys
from mct_camera_tools import camera_master

camera_master.set_camera_launch_param('calibration', False)
print(camera_master.get_camera_launch_param())

#camera_master.set_camera_launch_param('tracking', True)
#print(camera_master.get_camera_launch_param())
#camera_master.set_camera_launch_param('default', False)
#print(camera_master.get_camera_launch_param())
