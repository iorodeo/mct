from __future__ import print_function
import roslib
roslib.load_manifest('mct_web_apps')
import rospy
import mct_introspection
import sys
import time

from mct_camera_tools import camera_master
from mct_camera_tools import image_proc_master
from mct_transform_2d import transform_2d_calibrator_master
from mct_transform_2d import transform_2d_calibrator
from mct_camera_tools import mjpeg_servers
from mct_utilities import file_tools

file_tools.rsync_camera_calibrations(verbose=True)

# Start camera nodes and wait until they are ready
print(' * starting camera nodes ... ',end='')
sys.stdout.flush()
camera_master.set_camera_launch_param(
        frame_rate='homography_calibration',
        trigger=False
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

# Start transform 2d calibrator nodes and wait until ready
print(' * starting transform 2d calibrators ... ', end='')
sys.stdout.flush()
transform_2d_calibrator_master.start()
while not mct_introspection.transform_2d_calibrator_nodes_ready():
    time.sleep(0.2)
print('done')

## Start mjpeg servers and throttleing 
#print(' * starting mjpeg servers ... ',end='')
#sys.stdout.flush()
#mjpeg_servers.set_transport('image_homography_calibration')
#mjpeg_servers.start_servers()
#print('done')
