#!/usr/bin/env python
from __future__ import print_function
import roslib
roslib.load_manifest('mct_tracking_2d')
import rospy
import sys

import dynamic_reconfigure.client
import mct_introspection

rospy.init_node('camera_config', anonymous=True) 
camera_nodes = mct_introspection.get_camera_nodes()

for node in camera_nodes:

    client = dynamic_reconfigure.client.Client(node)
    config = client.get_configuration()

    print(node)
    print(config['brightness'])
    print(config['shutter'])
    print(config['gain'])
    print()

