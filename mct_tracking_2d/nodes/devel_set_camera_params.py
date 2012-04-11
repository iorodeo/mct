#!/usr/bin/env python
from __future__ import print_function
import roslib
roslib.load_manifest('mct_tracking_2d')
import rospy
import sys

import dynamic_reconfigure.client
import mct_introspection
from mct_utilities import file_tools

rospy.init_node('camera_config', anonymous=True) 

camera_params = {
        'brightness': 800,
        'shutter': 250,
        'gain': 300,
        }

try:
    region = sys.argv[1]
except IndexError:
    region = 'maze'

region_dict = file_tools.read_tracking_2d_regions()
camera_list = region_dict[region]

# Get list of camers nodes in region
node_list = mct_introspection.get_nodes()

camera_nodes = []
if 0:
    for camera in camera_list:
        for node in node_list:
            node_split = node.split('/')
            if camera in node_split and 'camera1394_node' in node_split: 
                camera_nodes.append(node)
else:
    for node in node_list:
        node_split = node.split('/')
        if 'camera1394_node' in node_split:
            camera_nodes.append(node)

for node in camera_nodes:
    print('updating:', node)
    client = dynamic_reconfigure.client.Client(node)
    client.update_configuration(camera_params)






    

