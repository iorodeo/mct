from __future__ import print_function
import roslib
roslib.load_manifest('mct_web_apps')
import rospy
import sys
import time

import mct_introspection
from mct_utilities import file_tools
from mct_utilities import region_tools
from mct_camera_tools import camera_master
from mct_camera_tools import image_proc_master
from mct_camera_trigger import camera_trigger

regions_dict = file_tools.read_tracking_2d_regions()
camera_pairs_dict = file_tools.read_tracking_2d_camera_pairs()
region_tools.check_regions_and_camera_pairs(regions_dict, camera_pairs_dict)

# Stop camera triggers
print(' * stopping camera triggers ... ',end='')
sys.stdout.flush()
camera_trigger.stop()
print('done')

file_tools.rsync_camera_calibrations(verbose=True)

# Start camera nodes and wait until they are ready
print(' * starting camera nodes ... ',end='')
sys.stdout.flush()
camera_master.set_camera_launch_param(
        frame_rate='camera_driver',
        trigger=True
        )
camera_master.start_cameras()
while not mct_introspection.camera_nodes_ready(mode='tracking'):
    time.sleep(0.2)
print('done')

# Delay until all camera nodes are ready 
print(' * camera nodes sync delay ... ', end='')
sys.stdout.flush()
time.sleep(10)
print('done')

print(' * starting camera triggers ... ', end='')
sys.stdout.flush()
frame_rates = file_tools.read_frame_rates()
#camera_trigger.start(frame_rates['tracking_2d'])
camera_trigger.start(10)
print('done')

# Start image_proc nodes and wait until they are ready
print(' * starting image proc nodes ... ', end='')
sys.stdout.flush()
image_proc_master.start_image_proc()
while not mct_introspection.image_proc_nodes_ready():
    time.sleep(0.2)
print('done')

# Wait for rectified images to be ready - required for launching transform 
# calibrators.
print(' * waiting for image rect topics ...', end='')
sys.stdout.flush()
while not mct_introspection.image_rect_ready():
    time.sleep(0.2)
print('done')


