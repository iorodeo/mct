from __future__ import print_function
import roslib
roslib.load_manifest('mct_web_apps')
import rospy
import mct_introspection
import sys
import time

from mct_camera_tools import camera_master
from mct_camera_tools import image_proc_master

regions_dict = file_tools.read_tracking_2d_regions()
camera_pairs_dict = file_tools.read_tracking_2d_camera_pairs()
region_tools.check_regions_and_camera_pairs(regions_dict, camera_pairs_dict)

file_tools.rsync_camera_calibrations(verbose=True)

# Start camera nodes and wait until they are ready
print(' * starting camera nodes ... ',end='')
sys.stdout.flush()
camera_master.set_camera_launch_param(
        frame_rate='tracking_cameras',
        trigger=True
        )
camera_master.start_cameras()
while not mct_introspection.camera_nodes_ready(mode='calibration'):
    time.sleep(0.2)
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

